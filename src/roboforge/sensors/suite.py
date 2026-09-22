"""Simulation-only sensor adapter; algorithms consume readings, not this object."""

import math

from roboforge.config import EncoderConfig, ImuConfig, LidarConfig, RunConfig
from roboforge.core import nonnegative
from roboforge.faults import FaultEngine
from roboforge.geometry import Vector2
from roboforge.physics import KinematicMotion
from roboforge.robot import DifferentialDriveRobot
from roboforge.robotics import BodyTwist2
from roboforge.sensors.noise import NoiseProcess
from roboforge.sensors.raycast import raycast
from roboforge.sensors.readings import EncoderReading, ImuReading, LidarReading, SensorReading


class SensorSuite:
    """Capture using integer sample indices; preserve delivery times and dropouts.

    Samples at a segment endpoint see the just-completed segment (left limit).
    Velocity discontinuities are averaged over successive IMU sampling intervals.
    """

    def __init__(self, config: RunConfig):
        self.config = config
        self.robot = DifferentialDriveRobot(config.robot)
        self.indices = {sensor.name: 0 for sensor in config.sensors}
        self.previous = {}
        self.processes = {}
        self.wheel_angles = [0.0, 0.0]
        self.time = 0.0
        self.readings: list[SensorReading] = []
        self.delivered: set[tuple[str, int]] = set()
        self.initialized = False
        self.delivery_clock = 0.0
        self.faults = FaultEngine(config)

    def _noise(self, sensor, channel, model, value, interval):
        key = sensor.name + ":" + channel
        if key not in self.processes:
            self.processes[key] = NoiseProcess(model, self.config.seed, key)
        return self.processes[key].sample(value, interval)

    def capture_initial(self) -> None:
        if self.initialized:
            raise ValueError("sensor suite is already initialized")
        self.initialized = True
        for sensor in self.config.sensors:
            self._capture(
                sensor, 0.0, self.config.robot.initial_pose.to_pose(), (0.0, 0.0), BodyTwist2(0, 0)
            )
            self.indices[sensor.name] = 1

    def advance(self, motion: KinematicMotion, *, start_time: float | None = None) -> None:
        if not self.initialized:
            raise ValueError("capture_initial must precede sensor advancement")
        if start_time is not None:
            start_time = nonnegative(start_time, "sensor interval start")
            if not math.isclose(start_time, self.time, rel_tol=1e-12, abs_tol=1e-12):
                raise ValueError("sensor interval must continue from the preceding motion")
            self.time = start_time
        end = self.time + motion.dt
        twist = motion.drive.forward(motion.wheels)
        shaft = motion.encoder_wheels or motion.wheels
        for sensor in self.config.sensors:
            while self.indices[sensor.name] / sensor.rate_hz <= end + 8 * math.ulp(max(1.0, end)):
                capture_time = self.indices[sensor.name] / sensor.rate_hz
                elapsed = min(motion.dt, max(0.0, capture_time - self.time))
                pose = motion.pose_at(elapsed / motion.dt) if motion.dt else motion.start
                angles = (
                    self.wheel_angles[0] + shaft.left * elapsed,
                    self.wheel_angles[1] + shaft.right * elapsed,
                )
                self._capture(sensor, capture_time, pose, angles, twist)
                self.indices[sensor.name] += 1
        self.wheel_angles[0] += shaft.left * motion.dt
        self.wheel_angles[1] += shaft.right * motion.dt
        self.time = end

    def _capture(self, sensor, time, pose, angles, twist):
        old_time, old_twist = self.previous.get(sensor.name, (time, twist))
        interval = time - old_time
        stamp = dict(
            sensor=sensor.name,
            sequence=self.indices[sensor.name],
            capture_time=time,
            delivery_time=time + sensor.latency,
        )
        if isinstance(sensor, EncoderConfig):
            ticks = []
            for label, angle in zip(("left", "right"), angles):
                value = self._noise(
                    sensor, label, sensor.noise, angle * (1 + sensor.scale_error), interval
                )
                ticks.append(
                    None if value is None else round(value * sensor.ticks_per_revolution / math.tau)
                )
            reading = EncoderReading(
                **stamp,
                frame="base",
                ticks_per_revolution=sensor.ticks_per_revolution,
                left_ticks=ticks[0],
                right_ticks=ticks[1],
            )
        else:
            frames = {frame.source_frame: frame for frame in self.robot.frame_transforms(pose)}
            mount = frames[sensor.frame]
            base = frames["base"]
            local = base.inverse() @ mount
            if isinstance(sensor, ImuConfig):
                dv = (twist.linear - old_twist.linear) / interval if interval else 0.0
                alpha = (twist.angular - old_twist.angular) / interval if interval else 0.0
                x, y = local.translation.x, local.translation.y
                acceleration = Vector2(
                    dv - alpha * y - twist.angular**2 * x,
                    twist.linear * twist.angular + alpha * x - twist.angular**2 * y,
                ).rotated(-local.theta)
                reading = ImuReading(
                    **stamp,
                    frame=sensor.frame,
                    gyro_z=self._noise(sensor, "gyro", sensor.gyro_noise, twist.angular, interval),
                    acceleration_x=self._noise(
                        sensor, "ax", sensor.acceleration_noise, acceleration.x, interval
                    ),
                    acceleration_y=self._noise(
                        sensor, "ay", sensor.acceleration_noise, acceleration.y, interval
                    ),
                )
            elif isinstance(sensor, LidarConfig):
                angles_scan = (
                    (0.0,)
                    if sensor.rays == 1
                    else tuple(
                        -sensor.field_of_view / 2 + i * sensor.field_of_view / sensor.rays
                        for i in range(sensor.rays)
                    )
                )
                ranges, hits = [], []
                for index, angle in enumerate(angles_scan):
                    hit = raycast(
                        self.config.environment,
                        mount.translation,
                        mount.theta + angle,
                        sensor.max_range,
                    )
                    distance = self._noise(
                        sensor, f"ray:{index}", sensor.noise, hit.distance, interval
                    )
                    if distance is None or hit.distance < sensor.min_range:
                        distance = None
                    elif hit.object_id is None:
                        distance = sensor.max_range
                    elif not sensor.min_range <= distance <= sensor.max_range:
                        distance = None
                    ranges.append(distance)
                    hits.append(distance is not None and hit.object_id is not None)
                reading = LidarReading(
                    **stamp,
                    frame=sensor.frame,
                    angles=angles_scan,
                    ranges=tuple(ranges),
                    hits=tuple(hits),
                    max_range=sensor.max_range,
                )
        self.previous[sensor.name] = (time, twist)
        self.readings.append(self.faults.reading(reading) if self.config.faults else reading)

    def deliver(self, until_time: float) -> tuple[SensorReading, ...]:
        """Deliver each captured reading at most once, never before its latency expires."""
        until_time = nonnegative(until_time, "delivery time")
        if until_time < self.delivery_clock:
            raise ValueError("delivery time must be monotonic")
        self.delivery_clock = until_time
        ready = sorted(
            (
                item
                for item in self.readings
                if item.delivery_time <= until_time
                and (item.sensor, item.sequence) not in self.delivered
            ),
            key=lambda item: (item.delivery_time, item.sensor, item.sequence),
        )
        self.delivered.update((item.sensor, item.sequence) for item in ready)
        return tuple(ready)
