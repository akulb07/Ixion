"""Sampled pack-voltage fixtures, independent of MCU ADC circuitry."""

import hashlib
import math
from collections.abc import Callable
from typing import Literal

from pydantic import Field, StrictBool, model_serializer, model_validator

from roboforge.config import Nonnegative, Positive, Real, Schema
from roboforge.core import finite, nonnegative

from .signal_clock import SignalClock


class VoltageErrorConfig(Schema):
    gain: Positive = Field(default=1, le=10)
    offset_v: Real = Field(default=0, ge=-1000, le=1000)
    noise_bound_v: Nonnegative = Field(default=0, le=1000)
    seed: int = Field(strict=True, ge=0, le=2**32 - 1, default=0)


class VoltageMonitorConfig(Schema):
    full_scale_v: Positive = Field(default=10, le=1000)
    resolution_bits: int = Field(strict=True, ge=2, le=24, default=12)
    conversion_s: Positive = Field(default=0.003, le=1)
    poll_every_steps: int = Field(strict=True, gt=0, le=10_000, default=10)
    errors: VoltageErrorConfig | None = None

    @model_serializer(mode="wrap")
    def preserve_ideal_encoding(self, handler):
        data = handler(self)
        if self.errors is None:
            data.pop("errors", None)
        return data


class VoltageObservation(Schema):
    time_s: Nonnegative
    sample_time_s: Nonnegative
    raw_count: int = Field(strict=True, ge=0, le=2**24 - 1)
    voltage_v: Nonnegative
    over_range: StrictBool

    @model_validator(mode="after")
    def check_time(self):
        if self.sample_time_s >= self.time_s:
            raise ValueError("voltage conversion must finish after sampling")
        return self


class VoltageCapture(Schema):
    source_voltage_v: Nonnegative
    observation: VoltageObservation
    unclipped_voltage_v: Real | None = None
    noise_v: Real | None = None

    @model_serializer(mode="wrap")
    def preserve_ideal_encoding(self, handler):
        data = handler(self)
        for name in ("unclipped_voltage_v", "noise_v"):
            if getattr(self, name) is None:
                data.pop(name, None)
        return data


class VoltageMonitorReport(Schema):
    model: Literal["ideal_pack_voltage_monitor", "pack_voltage_monitor_with_errors"] = (
        "ideal_pack_voltage_monitor"
    )
    config: VoltageMonitorConfig
    readings: tuple[VoltageCapture, ...] = Field(max_length=10_000)
    pending: VoltageCapture | None
    busy_skips: int = Field(strict=True, ge=0, le=10_000)

    @model_validator(mode="after")
    def check_model(self):
        if (self.model == "pack_voltage_monitor_with_errors") != (self.config.errors is not None):
            raise ValueError("voltage monitor model disagrees with configured errors")
        return self


def quantize_voltage(voltage_v: float, config: VoltageMonitorConfig) -> tuple[int, float, bool]:
    voltage = nonnegative(voltage_v, "sampled pack voltage")
    maximum = 2**config.resolution_bits - 1
    count = math.floor(min(voltage / config.full_scale_v, 1) * maximum + 0.5)
    return count, count * config.full_scale_v / maximum, voltage > config.full_scale_v


def _conversion(voltage_v, config, sample_index):
    voltage = nonnegative(voltage_v, "sampled pack voltage")
    evidence = {}
    if config.errors is not None:
        errors = config.errors
        # One stable draw per accepted capture, including a pending conversion.
        digest = hashlib.sha256(
            f"ixion-voltage-noise-v1:{errors.seed}:{sample_index}".encode("ascii")
        ).digest()
        unit_noise = 2 * (int.from_bytes(digest[:8], "big") / (2**64 - 1)) - 1
        noise = errors.noise_bound_v * unit_noise
        voltage = finite(voltage * errors.gain + errors.offset_v + noise, "voltage monitor input")
        evidence = {"unclipped_voltage_v": voltage, "noise_v": noise}
    count, decoded, over_range = quantize_voltage(max(0, voltage), config)
    return count, decoded, over_range, evidence


class VoltageMonitor:
    def __init__(
        self,
        clock: SignalClock,
        config: VoltageMonitorConfig,
        on_reading: Callable[[VoltageObservation], None],
    ):
        if not callable(on_reading):
            raise ValueError("voltage monitor needs a callable reading consumer")
        self._clock, self._config, self._on_reading = clock, config, on_reading
        self._readings = []
        self._pending = None
        self._busy_skips = 0

    def poll(self, voltage_v: float) -> bool:
        if self._pending is not None:
            self._busy_skips += 1
            if self._busy_skips > 10_000:
                raise ValueError("voltage monitor exceeds busy-skip budget")
            return False
        if len(self._readings) >= 10_000:
            raise ValueError("voltage monitor exceeds conversion budget")
        voltage = nonnegative(voltage_v, "sampled pack voltage")
        count, decoded, over_range, evidence = _conversion(
            voltage, self._config, len(self._readings)
        )
        end = finite(self._clock.time_s + self._config.conversion_s, "voltage delivery time")
        capture = VoltageCapture(
            source_voltage_v=voltage,
            observation=VoltageObservation(
                time_s=end,
                sample_time_s=self._clock.time_s,
                raw_count=count,
                voltage_v=decoded,
                over_range=over_range,
            ),
            **evidence,
        )

        def deliver():
            self._pending = None
            self._readings.append(capture)
            self._on_reading(capture.observation)

        self._clock.call_at(end, deliver)
        self._pending = capture
        return True

    def report(self) -> dict:
        return VoltageMonitorReport(
            model="ideal_pack_voltage_monitor"
            if self._config.errors is None
            else "pack_voltage_monitor_with_errors",
            config=self._config,
            readings=tuple(self._readings),
            pending=self._pending,
            busy_skips=self._busy_skips,
        ).model_dump(mode="json")


def project_voltage_monitor(raw, config, samples):
    report = VoltageMonitorReport.model_validate(raw)
    if report.config != VoltageMonitorConfig.model_validate(config):
        raise ValueError("voltage monitor conditions disagree with scenario")
    source = {s["time_s"]: s["pack_voltage_v"] for s in samples}
    end = samples[-1]["time_s"]
    last = -1.0
    available = 0.0
    for index, capture in enumerate(
        (*report.readings, *((report.pending,) if report.pending is not None else ()))
    ):
        observation = capture.observation
        if observation.sample_time_s < available:
            raise ValueError("voltage conversions cannot overlap")
        if (
            observation.sample_time_s <= last
            or source.get(observation.sample_time_s) != capture.source_voltage_v
        ):
            raise ValueError("voltage capture disagrees with recorded sample order or source")
        last = observation.sample_time_s
        available = observation.time_s
        count, voltage, over_range, evidence = _conversion(
            capture.source_voltage_v, report.config, index
        )
        if (capture.unclipped_voltage_v, capture.noise_v) != (
            evidence.get("unclipped_voltage_v"),
            evidence.get("noise_v"),
        ):
            raise ValueError("voltage capture disagrees with declared sensor errors")
        if (observation.raw_count, observation.voltage_v, observation.over_range) != (
            count,
            voltage,
            over_range,
        ):
            raise ValueError("voltage reading disagrees with quantization")
        if observation.time_s != finite(
            observation.sample_time_s + report.config.conversion_s, "voltage completion"
        ):
            raise ValueError("voltage delivery disagrees with conversion latency")
    if any(c.observation.time_s > end for c in report.readings):
        raise ValueError("voltage report invents a delivered reading beyond run end")
    if report.pending is not None and report.pending.observation.time_s <= end:
        raise ValueError("completed voltage reading cannot remain pending")
    return report.model_dump(mode="json")
