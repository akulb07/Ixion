"""Deterministic discrete wheel-rate actuators, independent of robot geometry."""

from collections import deque
from dataclasses import dataclass
from math import expm1

from roboforge.config import ActuatorConfig, WheelActuatorConfig
from roboforge.core import finite, positive
from roboforge.robotics import WheelSpeeds


@dataclass(frozen=True, slots=True)
class ActuatorSample:
    """Rates held during an attempted simulation interval, starting at time."""

    time: float
    requested: WheelSpeeds
    delayed: WheelSpeeds
    target: WheelSpeeds
    applied: WheelSpeeds
    speed_limited: tuple[bool, bool]
    acceleration_limited: tuple[bool, bool]


def _wheel(request: float, previous: float, dt: float, config: WheelActuatorConfig):
    target = 0.0 if abs(request) <= config.deadzone else finite(request * config.gain, "gain")
    bounded = target
    if config.max_speed is not None:
        bounded = max(-config.max_speed, min(config.max_speed, target))
    speed_limited = bounded != target
    alpha = -expm1(-dt / config.time_constant) if config.time_constant else 1.0
    change = finite((bounded - previous) * alpha, "actuator response")
    limited = change
    if config.max_acceleration is not None:
        limit = finite(config.max_acceleration * dt, "acceleration step")
        limited = max(-limit, min(limit, change))
    return bounded, finite(previous + limited, "wheel rate"), speed_limited, limited != change


class WheelActuators:
    """Fresh instance per run. Delay N holds zero for the first N samples."""

    def __init__(self, config: ActuatorConfig = ActuatorConfig()) -> None:
        self.config = config
        self.applied = WheelSpeeds(0, 0)
        self._queue: deque[WheelSpeeds] = deque()

    def step(self, requested: WheelSpeeds, dt: float, time: float) -> ActuatorSample:
        dt = positive(dt, "actuator timestep")
        time = finite(time, "actuator timestamp")
        if time < 0:
            raise ValueError("actuator timestamp must be nonnegative")
        # Compute before mutating so invalid numerical input leaves state intact.
        delayed = (
            requested
            if not self.config.delay_steps
            else (
                self._queue[0] if len(self._queue) == self.config.delay_steps else WheelSpeeds(0, 0)
            )
        )
        left = _wheel(delayed.left, self.applied.left, dt, self.config.left)
        right = _wheel(delayed.right, self.applied.right, dt, self.config.right)
        sample = ActuatorSample(
            time,
            requested,
            delayed,
            WheelSpeeds(left[0], right[0]),
            WheelSpeeds(left[1], right[1]),
            (left[2], right[2]),
            (left[3], right[3]),
        )
        if self.config.delay_steps:
            if len(self._queue) == self.config.delay_steps:
                self._queue.popleft()
            self._queue.append(requested)
        self.applied = sample.applied
        return sample
