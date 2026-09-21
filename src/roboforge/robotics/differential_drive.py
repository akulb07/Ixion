"""Ideal no-slip differential-drive model and constant-twist exact solution."""

import math
from dataclasses import dataclass
from typing import Literal

from roboforge.core import finite, nonnegative, positive
from roboforge.geometry import Pose2


@dataclass(frozen=True, slots=True)
class WheelSpeeds:
    """Signed left/right wheel rates in rad/s; positive means forward rolling."""

    left: float
    right: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "left", finite(self.left, "left wheel rate"))
        object.__setattr__(self, "right", finite(self.right, "right wheel rate"))


@dataclass(frozen=True, slots=True)
class BodyTwist2:
    """Forward speed (m/s) and counterclockwise yaw rate (rad/s); lateral speed 0."""

    linear: float
    angular: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "linear", finite(self.linear, "linear speed"))
        object.__setattr__(self, "angular", finite(self.angular, "angular speed"))


def _sinc(value: float) -> float:
    """sin(z)/z with continuous value 1 at zero (unnormalized sinc)."""
    if abs(value) < 1e-4:
        # O(z^6) truncation < 2e-28 at this threshold, below float64 roundoff.
        squared = value * value
        return 1.0 - squared / 6.0 + squared * squared / 120.0
    return math.sin(value) / value


@dataclass(frozen=True, slots=True)
class DifferentialDrive:
    """Equal rigid wheels; base origin is the midpoint of the wheel axle.

    No actuator limits, slip, dynamics, collision, or stochastic state here.
    These belong to later simulator/sensor modules.
    """

    wheel_radius: float
    wheel_separation: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "wheel_radius", positive(self.wheel_radius, "wheel radius"))
        object.__setattr__(
            self, "wheel_separation", positive(self.wheel_separation, "wheel separation")
        )

    def forward(self, wheels: WheelSpeeds) -> BodyTwist2:
        """Map wheel angular rates to the body-frame twist."""
        left = finite(self.wheel_radius * wheels.left, "left tangential speed")
        right = finite(self.wheel_radius * wheels.right, "right tangential speed")
        return BodyTwist2(left / 2.0 + right / 2.0, (right - left) / self.wheel_separation)

    def inverse(self, twist: BodyTwist2) -> WheelSpeeds:
        """Map a feasible nonholonomic body twist to wheel angular rates."""
        half_difference = finite(
            twist.angular * (self.wheel_separation / 2.0), "half wheel speed difference"
        )
        return WheelSpeeds(
            (twist.linear - half_difference) / self.wheel_radius,
            (twist.linear + half_difference) / self.wheel_radius,
        )

    def derivative(self, pose: Pose2, wheels: WheelSpeeds) -> tuple[float, float, float]:
        """Return world-frame (x_dot, y_dot, theta_dot), not a wrapped pose."""
        twist = self.forward(wheels)
        return (
            twist.linear * math.cos(pose.theta),
            twist.linear * math.sin(pose.theta),
            twist.angular,
        )

    def integrate(
        self,
        pose: Pose2,
        wheels: WheelSpeeds,
        dt: float,
        method: Literal["exact", "euler"] = "exact",
    ) -> Pose2:
        """No-slip pose after dt seconds with constant wheel rates.

        Uses midpoint heading and sinc to avoid division by near-zero yaw rate.
        This is a kinematic analytical reference, not a simulation scheduler.
        """
        dt = nonnegative(dt, "timestep")
        if method not in ("exact", "euler"):
            raise ValueError("integration method must be 'exact' or 'euler'")
        if dt == 0.0:
            return pose
        twist = self.forward(wheels)
        delta = finite(twist.angular * dt, "heading increment")
        distance = finite(twist.linear * dt, "travel distance")
        if method == "euler":
            return Pose2(
                pose.x + distance * math.cos(pose.theta),
                pose.y + distance * math.sin(pose.theta),
                pose.theta + delta,
            )
        half_delta = delta / 2.0
        chord = distance * _sinc(half_delta)
        heading = pose.theta + half_delta
        return Pose2(
            pose.x + chord * math.cos(heading),
            pose.y + chord * math.sin(heading),
            pose.theta + delta,
        )
