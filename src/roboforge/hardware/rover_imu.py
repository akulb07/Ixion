"""Aligned, body-mounted IMU capture and delayed reads for the straight rover fixture."""

import math
from collections.abc import Callable

from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Real, Schema
from roboforge.core import finite

from .board_i2c import MPU6050I2C, resolve_mpu6050_wiring
from .mpu6050 import MPU6050, IMUSample, decode_motion_burst
from .project import Identifier, RobotProject
from .rolling import RollingSample
from .scheduled_i2c import ScheduledMPU6050I2C
from .signal_clock import SignalClock


class RoverIMUConfig(Schema):
    component: Identifier = "imu"
    bus: Identifier = "sensors"
    poll_every_steps: int = Field(strict=True, gt=0, le=10_000, default=10)
    accel_range_selector: int = Field(strict=True, ge=0, le=3, default=0)
    gyro_range_selector: int = Field(strict=True, ge=0, le=3, default=0)
    temperature_c: Real


class IMUObservation(Schema):
    """Decoded register values delivered to host code, with simulation timing metadata."""

    time_s: Nonnegative
    sample_time_s: Nonnegative
    specific_force_m_s2: tuple[Real, Real, Real]
    angular_velocity_rad_s: tuple[Real, Real, Real]
    temperature_c: Real

    @model_validator(mode="after")
    def check_sample_time(self):
        if self.sample_time_s > self.time_s:
            raise ValueError("IMU sample cannot come from after its delivery time")
        return self


class RoverIMU:
    def __init__(
        self,
        project: RobotProject,
        mcu: str,
        config: RoverIMUConfig,
        clock: SignalClock,
        gravity_m_s2: float,
        on_event: Callable[[dict], None] | None = None,
        *,
        on_reading: Callable[[IMUObservation], None] | None = None,
        grade_rad: float = 0,
    ):
        if on_event is not None and not callable(on_event):
            raise ValueError("IMU event callback must be callable")
        if on_reading is not None and not callable(on_reading):
            raise ValueError("IMU reading callback must be callable")
        grade = finite(grade_rad, "IMU fixture grade")
        if not -math.pi / 2 < grade < math.pi / 2:
            raise ValueError("IMU fixture grade must be between -pi/2 and pi/2")
        wiring = resolve_mpu6050_wiring(project, mcu, config.bus, config.component)
        parts = {c.id: c for c in project.components}
        mounts = [
            p for p in project.assembly.links if p.kind == "mount" and p.target == config.component
        ]
        if len(mounts) != 1 or parts[mounts[0].source].category != "chassis":
            raise ValueError("rover IMU requires exactly one mount from a chassis")
        self.wiring = wiring
        self._config, self._clock, self._gravity = config, clock, gravity_m_s2
        self._grade = grade
        self._device = MPU6050(gravity_m_s2)
        self._bus = ScheduledMPU6050I2C(MPU6050I2C(wiring, self._device), clock)
        self._ready_time = None
        self._latest = None
        self._capture_count = self._busy_polls = self._initializing_polls = 0
        self._readings = []
        self._on_event = on_event
        self._on_reading = on_reading
        self._bus.read_registers(wiring.address, 0x75, 1, self._identity)

    def _identity(self, transfer):
        if transfer.data != (0x68,):
            raise ValueError("unexpected IMU identity during rover initialization")
        self._bus.write_registers(self.wiring.address, 0x6B, b"\x00", self._awake)

    def _awake(self, transfer):
        self._bus.write_registers(
            self.wiring.address,
            0x1B,
            bytes((self._config.gyro_range_selector << 3, self._config.accel_range_selector << 3)),
            self._configured,
        )

    def _configured(self, transfer):
        self._ready_time = transfer.completion_time_s
        if self._on_event is not None:
            self._on_event({"event": "imu_ready", "time_s": self._clock.time_s})

    def capture(self, sample: RollingSample) -> None:
        """Capture lane-forward/surface-normal specific force at a constant grade."""
        if sample.shaft.time_s != self._clock.time_s:
            raise ValueError("IMU capture must occur at the shared physics timestamp")
        if self._device.sleeping:
            return
        physical = IMUSample(
            time_s=self._clock.time_s,
            specific_force_m_s2=(
                sample.acceleration_m_s2 + self._gravity * math.sin(self._grade),
                0,
                self._gravity * math.cos(self._grade),
            ),
            angular_velocity_rad_s=(0, 0, 0),
            temperature_c=self._config.temperature_c,
        )
        self._device.publish(physical)
        self._latest = physical
        self._capture_count += 1

    def poll(self) -> None:
        if self._ready_time is None or self._latest is None:
            self._initializing_polls += 1
            return
        if self._bus.busy:
            self._busy_polls += 1
            return
        physical = self._latest
        diagnostics = self._device.diagnostics

        def receive(transfer):
            decoded = decode_motion_burst(
                bytes(transfer.data),
                self._config.accel_range_selector,
                self._config.gyro_range_selector,
                self._gravity,
            )
            self._readings.append(
                {
                    "time_s": self._clock.time_s,
                    "transfer": transfer.model_dump(),
                    "physical_sample": physical.model_dump(),
                    "decoded": decoded,
                    "diagnostics": diagnostics,
                }
            )
            if self._on_event is not None:
                self._on_event(
                    {
                        "event": "imu_read",
                        "time_s": self._clock.time_s,
                        "sample_time_s": transfer.sample_time_s,
                        "specific_force_m_s2": list(decoded["specific_force_m_s2"]),
                    }
                )
            if self._on_reading is not None:
                self._on_reading(
                    IMUObservation(
                        time_s=self._clock.time_s,
                        sample_time_s=transfer.sample_time_s,
                        specific_force_m_s2=decoded["specific_force_m_s2"],
                        angular_velocity_rad_s=decoded["angular_velocity_rad_s"],
                        temperature_c=decoded["temperature_c"],
                    )
                )

        self._bus.read_registers(self.wiring.address, 0x3B, 14, receive)

    def report(self) -> dict:
        return {
            "fixture_grade_rad": self._grade,
            "wiring": self.wiring.model_dump(),
            "ready_time_s": self._ready_time,
            "capture_count": self._capture_count,
            "polls_skipped_initializing": self._initializing_polls,
            "polls_skipped_busy": self._busy_polls,
            "readings": list(self._readings),
            "completed_transfers": [t.model_dump() for t in self._bus.completed],
            "pending_transfer": None
            if self._bus.pending is None
            else self._bus.pending.model_dump(),
            "limitations": [
                "Sensor x follows travel and z follows the surface normal at a constant declared grade; no independent mount rotation or offset acceleration",
                "Physics endpoints publish ideal samples; no internal MPU sample clock, filter, noise or bias",
                "Straight-line model has zero angular velocity; temperature is a constant scenario input",
                "Nonblocking polling skips busy requests; only completed reads reach observers",
                "Logical I2C timing; analog bus behavior and IMU power consumption are not modeled",
            ],
        }
