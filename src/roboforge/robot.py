"""Immutable ground truth values and an ideal differential-drive robot model."""

import math
from dataclasses import dataclass
from typing import Literal

from roboforge.config import RobotConfig
from roboforge.core import nonnegative, positive
from roboforge.geometry import Pose2, Transform2, Vector2
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


@dataclass(frozen=True, slots=True)
class MassProperties:
    """Descriptive SI mass/inertia values; no force dynamics are applied yet."""

    total_mass: float
    body_yaw_inertia: float
    total_yaw_inertia: float
    wheel_spin_inertia: float

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, positive(getattr(self, name), name))


class DifferentialDriveRobot:
    """Ideal kinematic robot; commands take effect immediately, without limits."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.kinematics = DifferentialDrive(config.wheel_radius, config.wheel_separation)
        # Uniform circular body; thin solid wheel disks with spin axes along y.
        body_inertia = config.body_yaw_inertia
        if body_inertia is None:
            body_inertia = config.body_mass * config.footprint_radius**2 / 2
        wheel_spin = config.wheel_mass * config.wheel_radius**2 / 2
        wheel_yaw = wheel_spin / 2 + config.wheel_mass * (config.wheel_separation / 2) ** 2
        self.mass_properties = MassProperties(
            config.body_mass + 2 * config.wheel_mass,
            body_inertia,
            body_inertia + 2 * wheel_yaw,
            wheel_spin,
        )

    def frame_transforms(self, pose: Pose2) -> tuple[Transform2, ...]:
        """World-from-base and world-from-mount transforms in stable order."""
        base = Transform2.from_pose(pose, target_frame="world", source_frame="base")
        mounts = [
            Transform2("base", "left_wheel", Vector2(0, self.config.wheel_separation / 2)),
            Transform2("base", "right_wheel", Vector2(0, -self.config.wheel_separation / 2)),
        ]
        mounts.extend(
            Transform2.from_pose(mount.pose.to_pose(), target_frame="base", source_frame=mount.name)
            for mount in self.config.mounts
        )
        return (base, *(base @ mount for mount in mounts))

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
