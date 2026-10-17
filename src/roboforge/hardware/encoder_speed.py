"""Windowed wheel-speed observations from decoded encoder counts."""

import math

from roboforge.config import Nonnegative, Positive, Real, Schema
from roboforge.core import finite

from .mounted_encoder import resolve_mounted_encoder
from .project import RobotProject
from .rover_firmware import FirmwareInputs


class EncoderSpeedConfig(Schema):
    counts_per_wheel_revolution: tuple[Positive, Positive]
    wheel_radius_m: Positive
    minimum_interval_s: Positive = 0.02

    @classmethod
    def from_project(cls, project: RobotProject, left_motor: str, right_motor: str):
        if left_motor == right_motor:
            raise ValueError("speed estimation requires two distinct wheel motors")
        mounts = tuple(
            resolve_mounted_encoder(project, motor) for motor in (left_motor, right_motor)
        )
        return cls(
            counts_per_wheel_revolution=tuple(
                4 * mount.quadrature.cycles_per_revolution * mount.angle_ratio for mount in mounts
            ),
            wheel_radius_m=project.assembly.wheel_radius_m,
        )


class EncoderSpeedObservation(Schema):
    time_s: Nonnegative
    interval_start_s: Nonnegative
    interval_s: Positive
    count_delta: tuple[int, int]
    wheel_speed_m_s: tuple[Real, Real]
    mean_speed_m_s: Real


class EncoderSpeedEstimator:
    """Count-window averages, not instantaneous ground-truth velocity.

    Short windows accumulate from the last emitted observation. Failed or
    non-increasing updates don't change the anchor or last accepted input time.
    """

    def __init__(self, config: EncoderSpeedConfig):
        self.config = config
        self._anchor = self._latest = None

    def update(self, inputs: FirmwareInputs) -> EncoderSpeedObservation | None:
        if self._latest is None:
            self._anchor = self._latest = inputs
            return None
        if inputs.time_s <= self._latest.time_s:
            raise ValueError("encoder speed observations require strictly increasing time")
        elapsed = finite(inputs.time_s - self._anchor.time_s, "encoder window")
        rounding = 4 * max(math.ulp(inputs.time_s), math.ulp(self._anchor.time_s))
        if self.config.minimum_interval_s - elapsed > rounding:
            self._latest = inputs
            return None
        deltas = (
            inputs.left_encoder_count - self._anchor.left_encoder_count,
            inputs.right_encoder_count - self._anchor.right_encoder_count,
        )
        try:
            speeds = tuple(
                finite(
                    delta / resolution * math.tau * self.config.wheel_radius_m / elapsed,
                    "encoder speed",
                )
                for delta, resolution in zip(deltas, self.config.counts_per_wheel_revolution)
            )
        except OverflowError as exc:
            raise ValueError("encoder count delta exceeds numeric range") from exc
        result = EncoderSpeedObservation(
            time_s=inputs.time_s,
            interval_start_s=self._anchor.time_s,
            interval_s=elapsed,
            count_delta=deltas,
            wheel_speed_m_s=speeds,
            mean_speed_m_s=finite(sum(speeds) / 2, "mean encoder speed"),
        )
        self._anchor = self._latest = inputs
        return result
