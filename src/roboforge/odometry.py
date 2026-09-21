"""Encoder-only dead reckoning. The caller supplies the initial pose prior."""

from dataclasses import dataclass
from math import pi

from roboforge.geometry import Pose2
from roboforge.robotics import BodyTwist2, DifferentialDrive, WheelSpeeds
from roboforge.sensors.readings import EncoderReading


@dataclass(frozen=True, slots=True)
class OdometryEstimate:
    time: float
    pose: Pose2
    twist: BodyTwist2
    sample_interval: float
    distance: float


class EncoderOdometry:
    """Constant-twist integration between complete cumulative count pairs.

    Dropouts produce no estimate; recovery integrates the complete gap. There is
    no encoder rollover: adapters must unwrap hardware counts before this layer.
    """

    def __init__(
        self, wheel_radius: float, wheel_separation: float, initial_pose: Pose2 = Pose2(0, 0, 0)
    ) -> None:
        self.drive = DifferentialDrive(wheel_radius, wheel_separation)
        self.pose = initial_pose
        self._previous: EncoderReading | None = None
        self._last_seen: EncoderReading | None = None
        self.distance = 0.0

    def update(self, reading: EncoderReading) -> OdometryEstimate | None:
        last = self._last_seen
        if last is not None:
            if reading.sensor != last.sensor or reading.frame != last.frame:
                raise ValueError("odometry cannot mix encoder streams")
            if reading.ticks_per_revolution != last.ticks_per_revolution:
                raise ValueError("encoder resolution changed")
            if reading.capture_time <= last.capture_time or reading.sequence <= last.sequence:
                raise ValueError("encoder time and sequence must strictly increase")
        if reading.left_ticks is None or reading.right_ticks is None:
            self._last_seen = reading
            return None
        previous = self._previous
        if previous is None:
            estimate = OdometryEstimate(reading.capture_time, self.pose, BodyTwist2(0, 0), 0, 0)
        else:
            dt = reading.capture_time - previous.capture_time
            radians_per_tick = 2 * pi / reading.ticks_per_revolution
            wheels = WheelSpeeds(
                (reading.left_ticks - previous.left_ticks) * radians_per_tick / dt,
                (reading.right_ticks - previous.right_ticks) * radians_per_tick / dt,
            )
            pose = self.drive.integrate(self.pose, wheels, dt)
            twist = self.drive.forward(wheels)
            estimate = OdometryEstimate(
                reading.capture_time, pose, twist, dt, self.distance + abs(twist.linear) * dt
            )
        self.pose, self.distance = estimate.pose, estimate.distance
        self._previous = self._last_seen = reading
        return estimate
