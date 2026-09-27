"""Encoder-guided path following at simulation tick boundaries."""

from dataclasses import dataclass

from roboforge.config import RunConfig
from roboforge.geometry import Pose2, Vector2
from roboforge.odometry import EncoderOdometry
from roboforge.planning import PlanningWorld
from roboforge.robotics import WheelSpeeds
from roboforge.sensors.readings import EncoderReading, SensorReading
from roboforge.tracking import PurePursuit, TrackingSample


@dataclass(frozen=True, slots=True)
class NavigationSample:
    time: float
    estimate: Pose2
    estimate_time: float | None
    measurement_fresh: bool
    tracking: TrackingSample
    requested: WheelSpeeds


class PathFollower:
    """The only pose prior is the configured start; updates use delivered counts."""

    def __init__(self, config: RunConfig):
        self.config = config.navigation
        if self.config is None:
            raise ValueError("navigation configuration required")
        nav = self.config
        path = tuple(Vector2(p.x, p.y) for p in nav.path)
        world = PlanningWorld(config.environment, config.robot.footprint_radius + nav.clearance)
        if not all(world.segment_free(a, b) for a, b in zip(path, path[1:])):
            raise ValueError("navigation path intersects an obstacle or clearance boundary")
        self.odometry = EncoderOdometry(
            config.robot.wheel_radius,
            config.robot.wheel_separation,
            config.robot.initial_pose.to_pose(),
        )
        self.tracker = PurePursuit(
            path, nav.lookahead, nav.max_speed, nav.max_yaw_rate, nav.goal_tolerance
        )
        self.estimate_time = None
        self.samples: list[NavigationSample] = []
        self.reached = False

    def update(self, time: float, readings: tuple[SensorReading, ...]) -> WheelSpeeds:
        for reading in readings:
            if isinstance(reading, EncoderReading) and reading.sensor == self.config.encoder:
                if reading.delivery_time > time:
                    raise ValueError("navigation cannot use undelivered encoder readings")
                estimate = self.odometry.update(reading)
                if estimate is not None:
                    self.estimate_time = estimate.time
        fresh = (
            self.estimate_time is not None
            and time - self.estimate_time <= self.config.max_sensor_age
        )
        tracking = self.tracker.update(self.odometry.pose)
        self.reached = fresh and tracking.reached
        requested = self.odometry.drive.inverse(tracking.command) if fresh else WheelSpeeds(0, 0)
        self.samples.append(
            NavigationSample(
                time, self.odometry.pose, self.estimate_time, fresh, tracking, requested
            )
        )
        return requested
