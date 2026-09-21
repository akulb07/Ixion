"""Immutable ground truth values and an ideal differential-drive robot model."""

import math
from dataclasses import dataclass
from typing import Literal

from roboforge.config import RobotConfig
from roboforge.core import nonnegative
from roboforge.geometry import Pose2
from roboforge.robotics import BodyTwist2, DifferentialDrive, WheelSpeeds


@dataclass(frozen=True, slots=True)
class RobotState:
    """Ground truth after an applied command; vx/vy are in the world frame.

    This type is owned by simulation. Future estimators will return a distinct
    estimate type and will receive measurements, never this object by default.
    """

    pose: Pose2
    wheels: WheelSpeeds
    twist: BodyTwist2
    time: float

    def __post_init__(self) -> None:
        if not isinstance(self.pose, Pose2) or not isinstance(self.wheels, WheelSpeeds):
            raise ValueError("state requires Pose2 and WheelSpeeds values")
        if not isinstance(self.twist, BodyTwist2):
            raise ValueError("state requires a BodyTwist2 value")
        object.__setattr__(self, "time", nonnegative(self.time, "state time"))

    @property
    def vx(self) -> float:
        return self.twist.linear * math.cos(self.pose.theta)

    @property
    def vy(self) -> float:
        return self.twist.linear * math.sin(self.pose.theta)

    @property
    def omega(self) -> float:
        return self.twist.angular


class DifferentialDriveRobot:
    """Ideal kinematic robot; commands take effect immediately, without limits."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.kinematics = DifferentialDrive(config.wheel_radius, config.wheel_separation)

    def initial_state(self) -> RobotState:
        return RobotState(
            self.config.initial_pose.to_pose(), WheelSpeeds(0, 0), BodyTwist2(0, 0), 0
        )

    def step(
        self,
        state: RobotState,
        wheels: WheelSpeeds,
        dt: float,
        time: float,
        method: Literal["exact", "euler"],
    ) -> RobotState:
        """Advance using a timestamp supplied by the authoritative simulation clock."""
        dt = nonnegative(dt, "timestep")
        time = nonnegative(time, "state time")
        if not math.isclose(time, state.time + dt, rel_tol=1e-12, abs_tol=1e-12):
            raise ValueError("state time must match the previous timestamp plus timestep")
        pose = self.kinematics.integrate(state.pose, wheels, dt, method)
        return RobotState(pose, wheels, self.kinematics.forward(wheels), time)
