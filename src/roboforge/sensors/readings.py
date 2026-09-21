"""Validated immutable measurement contracts suitable for future hardware adapters."""

from typing import Annotated, Literal

from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Real, Schema, Steps


class Reading(Schema):
    sensor: str = Field(min_length=1)
    frame: str = Field(min_length=1)
    sequence: Annotated[int, Field(strict=True, ge=0)]
    capture_time: Nonnegative
    delivery_time: Nonnegative

    @model_validator(mode="after")
    def causal_time(self):
        if self.delivery_time < self.capture_time:
            raise ValueError("delivery_time must not precede capture_time")
        return self


class EncoderReading(Reading):
    kind: Literal["encoder"] = "encoder"
    ticks_per_revolution: Steps
    left_ticks: Annotated[int, Field(strict=True)] | None
    right_ticks: Annotated[int, Field(strict=True)] | None


class ImuReading(Reading):
    kind: Literal["imu"] = "imu"
    gyro_z: Real | None
    acceleration_x: Real | None
    acceleration_y: Real | None


class LidarReading(Reading):
    kind: Literal["lidar"] = "lidar"
    angles: tuple[Real, ...]
    ranges: tuple[Nonnegative | None, ...]
    hits: tuple[bool, ...]
    max_range: Nonnegative

    @model_validator(mode="after")
    def consistent_scan(self):
        if not (len(self.angles) == len(self.ranges) == len(self.hits)):
            raise ValueError("LiDAR angle/range/hit arrays must have the same length")
        for distance, hit in zip(self.ranges, self.hits):
            if distance is None and hit:
                raise ValueError("invalid LiDAR rays cannot be hits")
            if distance is not None and distance > self.max_range:
                raise ValueError("LiDAR range must not exceed max_range")
        return self


SensorReading = EncoderReading | ImuReading | LidarReading
