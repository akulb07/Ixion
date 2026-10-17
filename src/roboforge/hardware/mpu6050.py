"""Explicit MPU6050 register subset, fed by caller-published physical samples."""

import math

from roboforge.config import Nonnegative, Real, Schema
from roboforge.core import finite, nonnegative, positive

REGISTER_MAP_SOURCE = (
    "https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf"
)
ACCEL_LSB_PER_G = (16384, 8192, 4096, 2048)
GYRO_LSB_PER_DEG_S = (131, 65.5, 32.8, 16.4)


class IMUSample(Schema):
    time_s: Nonnegative
    specific_force_m_s2: tuple[Real, Real, Real]
    angular_velocity_rad_s: tuple[Real, Real, Real]
    temperature_c: Real


def register_span(register: int, length: int) -> range:
    if type(register) is not int or not 0 <= register <= 255:
        raise ValueError("register must be an integer byte")
    if type(length) is not int or not 1 <= length <= 14 or register + length > 256:
        raise ValueError("register transfer requires 1 to 14 bytes without address wrap")
    return range(register, register + length)


class MPU6050:
    """Ideal latched measurements; no internal sampling clock, DLPF, FIFO or DMP.

    Supported configuration changes invalidate the measurement. Wake/reset take
    effect immediately in this subset; the caller must publish a fresh sample.
    """

    def __init__(self, gravity_m_s2: float = 9.80665):
        self._gravity = positive(gravity_m_s2, "MPU gravity reference")
        self._registers = {0x1B: 0, 0x1C: 0, 0x6B: 0x40, 0x75: 0x68}
        self._time = 0.0
        self._sample_time = None
        self._data = None
        self._diagnostics = ()

    @property
    def sleeping(self) -> bool:
        return bool(self._registers[0x6B] & 0x40)

    @property
    def sample_time_s(self) -> float | None:
        return self._sample_time

    @property
    def diagnostics(self) -> tuple[str, ...]:
        return self._diagnostics

    def _check_time(self, time_s):
        time = nonnegative(time_s, "MPU time")
        if time < self._time:
            raise ValueError("MPU operations cannot move backwards in time")
        return time

    def publish(self, sample: IMUSample) -> None:
        time = self._check_time(sample.time_s)
        if self.sleeping:
            raise ValueError("wake the MPU before publishing a measurement")
        accel_scale = ACCEL_LSB_PER_G[self._registers[0x1C] >> 3]
        gyro_scale = GYRO_LSB_PER_DEG_S[self._registers[0x1B] >> 3]
        diagnostics = []

        def quantize(value, label):
            raw = round(finite(value, f"MPU {label} conversion"))
            clipped = min(32767, max(-32768, raw))
            if clipped != raw:
                diagnostics.append(f"{label}_clipped")
            return clipped.to_bytes(2, "big", signed=True)

        force = [
            quantize(value / self._gravity * accel_scale, f"accel_{axis}")
            for axis, value in zip("xyz", sample.specific_force_m_s2)
        ]
        temperature = quantize((sample.temperature_c - 36.53) * 340, "temperature")
        angular = [
            quantize(math.degrees(value) * gyro_scale, f"gyro_{axis}")
            for axis, value in zip("xyz", sample.angular_velocity_rad_s)
        ]
        data = b"".join((*force, temperature, *angular))
        self._data, self._sample_time = data, time
        self._diagnostics, self._time = tuple(diagnostics), time

    def read_registers(self, register: int, length: int, time_s: float) -> bytes:
        span = register_span(register, length)
        time = self._check_time(time_s)
        if any(address not in self._registers and not 0x3B <= address <= 0x48 for address in span):
            raise ValueError("unsupported MPU6050 register read")
        if any(0x3B <= address <= 0x48 for address in span) and self._data is None:
            raise ValueError("MPU measurement unavailable; wake and publish a fresh sample")
        data = bytes(
            self._registers[address] if address in self._registers else self._data[address - 0x3B]
            for address in span
        )
        self._time = time
        return data

    def write_registers(self, register: int, data: bytes, time_s: float) -> None:
        if not isinstance(data, bytes):
            raise ValueError("MPU register writes require bytes")
        span = register_span(register, len(data))
        time = self._check_time(time_s)
        for address, value in zip(span, data):
            if address in {0x1B, 0x1C}:
                if value not in {0, 8, 16, 24}:
                    raise ValueError(
                        "MPU range writes support only FS_SEL/AFS_SEL; other bits are unsupported"
                    )
            elif address == 0x6B:
                if value not in {0, 1, 0x40, 0x41, 0x80}:
                    raise ValueError("unsupported MPU power/clock mode")
            else:
                raise ValueError("unsupported or read-only MPU6050 register write")
        updated = dict(self._registers)
        if register == 0x6B and data == b"\x80":
            updated = {0x1B: 0, 0x1C: 0, 0x6B: 0x40, 0x75: 0x68}
        else:
            updated.update(zip(span, data))
        self._registers, self._time = updated, time
        self._data = self._sample_time = None
        self._diagnostics = ()


def decode_motion_burst(
    data: bytes, accel_range: int = 0, gyro_range: int = 0, gravity_m_s2: float = 9.80665
) -> dict:
    """Decode the supported 14-byte burst using the ranges chosen by host code."""
    if not isinstance(data, bytes) or len(data) != 14:
        raise ValueError("motion burst requires exactly 14 bytes")
    if any(type(value) is not int or not 0 <= value <= 3 for value in (accel_range, gyro_range)):
        raise ValueError("IMU range selectors must be integers from zero to three")
    gravity = positive(gravity_m_s2, "MPU gravity reference")
    counts = [int.from_bytes(data[i : i + 2], "big", signed=True) for i in range(0, 14, 2)]
    return {
        "raw_counts": counts,
        "specific_force_m_s2": [
            finite(c / ACCEL_LSB_PER_G[accel_range] * gravity, "decoded force") for c in counts[:3]
        ],
        "temperature_c": counts[3] / 340 + 36.53,
        "angular_velocity_rad_s": [
            math.radians(c / GYRO_LSB_PER_DEG_S[gyro_range]) for c in counts[4:]
        ],
    }
