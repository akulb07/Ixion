"""PID and encoder-feedback wheel control; no ground-truth state access."""

from dataclasses import dataclass
from math import pi

from roboforge.config import PIDConfig, WheelControllerConfig
from roboforge.core import finite, positive
from roboforge.robotics import WheelSpeeds
from roboforge.sensors.readings import EncoderReading, SensorReading


@dataclass(frozen=True, slots=True)
class PIDSample:
    setpoint: float
    measurement: float
    error: float
    proportional: float
    integral: float
    derivative: float
    feedforward: float
    unclamped: float
    output: float
    saturated: bool


class PID:
    """Conditional integration anti-windup, filtered derivative on measurement."""

    def __init__(self, config: PIDConfig = PIDConfig()) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self._measurement: float | None = None
        self._derivative = 0.0

    def step(
        self, setpoint: float, measurement: float, dt: float, feedforward: float = 0.0
    ) -> PIDSample:
        setpoint = finite(setpoint, "PID setpoint")
        measurement = finite(measurement, "PID measurement")
        dt = positive(dt, "PID timestep")
        feedforward = finite(feedforward, "PID feedforward")
        c = self.config
        error = finite(setpoint - measurement, "PID error")
        p = finite(c.kp * error, "PID proportional")
        raw_d = (
            0.0
            if self._measurement is None
            else finite(-(measurement - self._measurement) / dt, "PID derivative")
        )
        alpha = dt / (c.derivative_time_constant + dt)
        filtered = finite(self._derivative + alpha * (raw_d - self._derivative), "PID filter")
        d = finite(c.kd * filtered, "PID derivative term")
        increment = finite(c.ki * error * dt, "PID integral increment")
        candidate = max(-c.integral_limit, min(c.integral_limit, self.integral + increment))
        proposed = finite(p + candidate + d + feedforward, "PID proposed output")
        # Reject integration only when it pushes farther into output saturation.
        if (proposed > c.output_max and increment > 0) or (
            proposed < c.output_min and increment < 0
        ):
            candidate = self.integral
        unclamped = finite(p + candidate + d + feedforward, "PID output")
        output = max(c.output_min, min(c.output_max, unclamped))
        self.integral = candidate
        self._measurement = measurement
        self._derivative = filtered
        return PIDSample(
            setpoint,
            measurement,
            error,
            p,
            candidate,
            d,
            feedforward,
            unclamped,
            output,
            output != unclamped,
        )


@dataclass(frozen=True, slots=True)
class WheelControlSample:
    time: float
    capture_time: float
    measurement_dt: float
    left: PIDSample
    right: PIDSample


class EncoderWheelController:
    """Hold output until a new complete cumulative encoder pair is delivered.

    Two complete samples establish a rate; dropout gaps use their full capture
    interval. Startup output is zero. Setpoints take effect on the next update.
    """

    def __init__(self, config: WheelControllerConfig) -> None:
        self.config = config
        self.left, self.right = PID(config.left), PID(config.right)
        self.output = WheelSpeeds(0, 0)
        self._previous: EncoderReading | None = None
        self.samples: list[WheelControlSample] = []

    def update(
        self, time: float, target: WheelSpeeds, readings: tuple[SensorReading, ...]
    ) -> WheelSpeeds:
        time = finite(time, "control time")
        for reading in readings:
            if not isinstance(reading, EncoderReading) or reading.sensor != self.config.encoder:
                continue
            if reading.delivery_time > time:
                raise ValueError("cannot control from an undelivered encoder reading")
            if reading.left_ticks is None or reading.right_ticks is None:
                continue
            previous = self._previous
            if previous is not None:
                if reading.capture_time <= previous.capture_time:
                    raise ValueError("encoder captures must strictly increase")
                if reading.ticks_per_revolution != previous.ticks_per_revolution:
                    raise ValueError("encoder resolution changed")
                dt = reading.capture_time - previous.capture_time
                scale = 2 * pi / reading.ticks_per_revolution / dt
                left = self.left.step(
                    target.left,
                    (reading.left_ticks - previous.left_ticks) * scale,
                    dt,
                    self.config.feedforward * target.left,
                )
                right = self.right.step(
                    target.right,
                    (reading.right_ticks - previous.right_ticks) * scale,
                    dt,
                    self.config.feedforward * target.right,
                )
                self.output = WheelSpeeds(left.output, right.output)
                self.samples.append(WheelControlSample(time, reading.capture_time, dt, left, right))
            self._previous = reading
        return self.output
