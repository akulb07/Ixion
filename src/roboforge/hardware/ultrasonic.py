"""Ideal HC-SR04 trigger/echo timing for an explicitly supplied target distance."""

from typing import Literal

from pydantic import model_validator

from roboforge.config import Nonnegative, Positive, Schema
from roboforge.core import finite, nonnegative


class UltrasonicConfig(Schema):
    response_delay_s: Nonnegative
    sound_speed_m_s: Positive = 340
    minimum_distance_m: Positive = 0.02
    maximum_distance_m: Positive = 4
    minimum_trigger_s: Positive = 10e-6
    minimum_cycle_s: Positive = 0.06
    timeout_s: Positive = 0.03

    @model_validator(mode="after")
    def check_window(self):
        if self.minimum_distance_m >= self.maximum_distance_m:
            raise ValueError("ultrasonic range must increase")
        if (
            self.response_delay_s + 2 * self.maximum_distance_m / self.sound_speed_m_s
            > self.timeout_s
        ):
            raise ValueError("timeout must cover response delay and maximum-range echo")
        return self


class EchoMeasurement(Schema):
    status: Literal["echo", "no_echo", "short_trigger"]
    trigger_time_s: Nonnegative
    trigger_width_s: Nonnegative
    echo_rise_s: Nonnegative | None = None
    echo_fall_s: Nonnegative | None = None
    timeout_time_s: Nonnegative | None = None


class UltrasonicSensor:
    """No scene geometry, electrical behavior or clone-specific timeout waveform.

    None/out-of-range distance produces no echo pulse and a caller-visible timeout.
    Response delay and no-echo behavior are explicit simulation assumptions.
    """

    def __init__(self, config: UltrasonicConfig):
        self.config = config
        self._next_trigger = 0.0
        self._last_time = 0.0

    def trigger(self, time_s: float, width_s: float, distance_m: float | None) -> EchoMeasurement:
        time = nonnegative(time_s, "trigger time")
        width = nonnegative(width_s, "trigger width")
        distance = None if distance_m is None else nonnegative(distance_m, "target distance")
        if time < self._last_time:
            raise ValueError("ultrasonic trigger time cannot move backwards")
        if time < self._next_trigger:
            raise ValueError("ultrasonic measurement busy or minimum cycle not elapsed")
        if width < self.config.minimum_trigger_s:
            result = EchoMeasurement(
                status="short_trigger", trigger_time_s=time, trigger_width_s=width
            )
            self._last_time = time
            return result
        end = finite(time + width, "trigger end")
        timeout = finite(end + self.config.timeout_s, "echo timeout")
        rise = fall = None
        status = "no_echo"
        if (
            distance is not None
            and self.config.minimum_distance_m <= distance <= self.config.maximum_distance_m
        ):
            rise = finite(end + self.config.response_delay_s, "echo rise")
            fall = finite(rise + 2 * distance / self.config.sound_speed_m_s, "echo fall")
            if fall <= rise:
                raise ValueError("echo pulse is below timestamp precision")
            status = "echo"
        result = EchoMeasurement(
            status=status,
            trigger_time_s=time,
            trigger_width_s=width,
            echo_rise_s=rise,
            echo_fall_s=fall,
            timeout_time_s=timeout,
        )
        next_trigger = finite(
            max(time + self.config.minimum_cycle_s, timeout if fall is None else fall),
            "next trigger time",
        )
        self._last_time, self._next_trigger = time, next_trigger
        return result
