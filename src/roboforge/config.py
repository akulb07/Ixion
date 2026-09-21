"""Strict, immutable robot, world and kinematic simulation configuration."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Annotated, Literal

import yaml
from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, model_validator

from roboforge.core import finite
from roboforge.geometry import Pose2


def _real(value: object) -> float:
    return finite(value, "value")


Real = Annotated[float, BeforeValidator(_real)]
Positive = Annotated[Real, Field(gt=0)]
Steps = Annotated[int, Field(strict=True, gt=0)]


class Schema(BaseModel):
    """Reject misspelled fields and assignment; collection fields use tuples."""

    model_config = ConfigDict(extra="forbid", frozen=True, validate_default=True)


class PoseConfig(Schema):
    x: Real = 0.0
    y: Real = 0.0
    theta: Real = 0.0

    def to_pose(self) -> Pose2:
        return Pose2(self.x, self.y, self.theta)


class FrameMount(Schema):
    """A static named mount in the robot base frame; not a sensor model."""

    name: str = Field(min_length=1, pattern=r"^[a-z][a-z0-9_]*$")
    pose: PoseConfig = PoseConfig()


class RobotConfig(Schema):
    model: Literal["differential_drive"] = "differential_drive"
    name: str = Field(default="differential_bot", min_length=1)
    wheel_radius: Positive = 0.05
    wheel_separation: Positive = 0.30
    footprint_radius: Positive = 0.20
    initial_pose: PoseConfig = PoseConfig()
    body_mass: Positive = 10.0
    wheel_mass: Positive = 0.25
    body_yaw_inertia: Positive | None = None
    mounts: tuple[FrameMount, ...] = (
        FrameMount(name="lidar", pose=PoseConfig(x=0.1)),
        FrameMount(name="imu"),
    )

    @model_validator(mode="after")
    def check_mount_names(self) -> RobotConfig:
        names = {"world", "base", "left_wheel", "right_wheel"}
        for mount in self.mounts:
            if mount.name in names:
                raise ValueError(f"duplicate or reserved mount name: {mount.name}")
            names.add(mount.name)
        return self


class Rectangle(Schema):
    """Axis-aligned rectangle; x/y identify its lower-left corner."""

    type: Literal["rectangle"] = "rectangle"
    x: Real
    y: Real
    width: Positive
    height: Positive


class Circle(Schema):
    """Circle; x/y identify its centre."""

    type: Literal["circle"] = "circle"
    x: Real
    y: Real
    radius: Positive


Obstacle = Annotated[Rectangle | Circle, Field(discriminator="type")]


class Environment(Schema):
    """Immutable world description; geometry queries live in CollisionWorld."""

    name: str = Field(default="empty_room", min_length=1)
    width: Positive = 10.0
    height: Positive = 10.0
    obstacles: tuple[Obstacle, ...] = ()

    @model_validator(mode="after")
    def check_obstacle_bounds(self) -> Environment:
        for index, obstacle in enumerate(self.obstacles):
            if isinstance(obstacle, Rectangle):
                bounds = (
                    obstacle.x,
                    obstacle.y,
                    obstacle.x + obstacle.width,
                    obstacle.y + obstacle.height,
                )
            else:
                bounds = (
                    obstacle.x - obstacle.radius,
                    obstacle.y - obstacle.radius,
                    obstacle.x + obstacle.radius,
                    obstacle.y + obstacle.radius,
                )
            if not (
                0 <= bounds[0] <= bounds[2] <= self.width
                and 0 <= bounds[1] <= bounds[3] <= self.height
            ):
                raise ValueError(f"obstacle {index} must fit inside environment bounds")
        return self


class WheelCommand(Schema):
    """Piecewise constant angular wheel rates applied for an integer step count."""

    left: Real
    right: Real
    steps: Steps


class CollisionConfig(Schema):
    """Opt-in conservative swept collision checking, in metres."""

    mode: Literal["disabled", "stop"] = "disabled"
    spatial_tolerance: Positive = 1e-6
    max_queries: Steps = 100000


class SimulationConfig(Schema):
    dt: Positive = 0.01
    integrator: Literal["exact", "euler"] = "exact"
    collision: CollisionConfig = CollisionConfig()


Nonnegative = Annotated[Real, Field(ge=0)]
Probability = Annotated[Real, Field(ge=0, le=1)]
SensorName = Annotated[str, Field(min_length=1, pattern=r"^[a-z][a-z0-9_]*$")]


class NoiseConfig(Schema):
    stddev: Nonnegative = 0.0
    bias: Real = 0.0
    dropout: Probability = 0.0
    bias_walk: Nonnegative = 0.0


class WheelActuatorConfig(Schema):
    max_speed: Positive | None = None
    max_acceleration: Positive | None = None
    time_constant: Nonnegative = 0.0
    deadzone: Nonnegative = 0.0
    gain: Positive = 1.0


class ActuatorConfig(Schema):
    delay_steps: Annotated[int, Field(strict=True, ge=0)] = 0
    left: WheelActuatorConfig = WheelActuatorConfig()
    right: WheelActuatorConfig = WheelActuatorConfig()


class SensorBase(Schema):
    name: SensorName
    rate_hz: Positive = 10.0
    latency: Nonnegative = 0.0


class EncoderConfig(SensorBase):
    type: Literal["encoder"] = "encoder"
    name: SensorName = "encoders"
    ticks_per_revolution: Steps = 2048
    scale_error: Real = Field(default=0.0, gt=-1)
    noise: NoiseConfig = NoiseConfig()


class ImuConfig(SensorBase):
    type: Literal["imu"] = "imu"
    name: SensorName = "imu"
    frame: str = "imu"
    gyro_noise: NoiseConfig = NoiseConfig()
    acceleration_noise: NoiseConfig = NoiseConfig()


class LidarConfig(SensorBase):
    type: Literal["lidar"] = "lidar"
    name: SensorName = "lidar"
    frame: str = "lidar"
    rays: Steps = 180
    min_range: Nonnegative = 0.05
    max_range: Positive = 12.0
    field_of_view: Positive = Field(default=6.283185307179586, le=6.283185307179586)
    noise: NoiseConfig = NoiseConfig()

    @model_validator(mode="after")
    def range_order(self) -> LidarConfig:
        if self.min_range >= self.max_range:
            raise ValueError("LiDAR min_range must be below max_range")
        return self


SensorConfig = Annotated[EncoderConfig | ImuConfig | LidarConfig, Field(discriminator="type")]


class RunConfig(Schema):
    schema_version: Literal[1] = 1
    name: str = Field(default="foundation_demo", min_length=1)
    seed: Annotated[int, Field(strict=True, ge=0)] = 42
    robot: RobotConfig = RobotConfig()
    environment: Environment = Environment()
    simulation: SimulationConfig = SimulationConfig()
    commands: tuple[WheelCommand, ...] = Field(min_length=1)
    sensors: tuple[SensorConfig, ...] = ()
    actuators: ActuatorConfig = ActuatorConfig()

    @model_validator(mode="after")
    def check_run(self) -> RunConfig:
        pose = self.robot.initial_pose
        if not (0 <= pose.x <= self.environment.width and 0 <= pose.y <= self.environment.height):
            raise ValueError("initial robot centre must lie inside environment bounds")
        finite(sum(command.steps for command in self.commands) * self.simulation.dt, "run duration")
        names = set()
        frames = {"base", "left_wheel", "right_wheel", *(mount.name for mount in self.robot.mounts)}
        for sensor in self.sensors:
            if sensor.name in names:
                raise ValueError(f"duplicate sensor name: {sensor.name}")
            names.add(sensor.name)
            if hasattr(sensor, "frame") and sensor.frame not in frames:
                raise ValueError(f"sensor {sensor.name} references missing frame {sensor.frame}")
        return self


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loading that rejects duplicate keys instead of silently overriding."""


def _unique_mapping(loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False) -> dict:
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ValueError("configuration mapping keys must be strings")
        if key in mapping:
            raise ValueError(f"duplicate configuration key: {key}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)
# PyYAML's YAML-1.1 resolver treats JSON numbers such as 1e-6 as strings.
# Resolve unquoted exponent notation locally; quoted numeric strings stay invalid.
_UniqueKeyLoader.add_implicit_resolver(
    "tag:yaml.org,2002:float",
    re.compile(r"^[-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)[eE][-+]?[0-9]+$"),
    list("-+0123456789."),
)


def load_config(path: str | Path) -> RunConfig:
    """Load YAML or JSON safely and report file/field context for invalid inputs."""
    path = Path(path)
    try:
        data = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
        return RunConfig.model_validate(data)
    except (OSError, ValueError, yaml.YAMLError, RecursionError) as exc:
        raise ValueError(f"Invalid configuration {path}: {exc}") from exc
