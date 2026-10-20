"""Host voltage-feedback braking policy, with sonar fallback."""

from pydantic import Field

from roboforge.config import Nonnegative, Positive, Schema

from .rover_firmware import (
    BrakeRequest,
    FirmwareInputs,
    FirmwareLog,
    FirmwareResponse,
    ObstacleBrakeFirmware,
)
from .voltage_monitor import VoltageObservation


class VoltageGuardConfig(Schema):
    minimum_voltage_v: Positive = 8
    maximum_sample_age_s: Nonnegative = 0.01
    log_decisions: bool = Field(strict=True, default=False)


class VoltageBrakeFirmware(ObstacleBrakeFirmware, VoltageGuardConfig):
    def on_voltage(self, observation: VoltageObservation, inputs: FirmwareInputs):
        reason = (
            "voltage_monitor_range"
            if observation.over_range
            else "voltage_sample_stale"
            if inputs.time_s - observation.sample_time_s > self.maximum_sample_age_s
            else "low_voltage"
            if observation.voltage_v <= self.minimum_voltage_v
            else None
        )
        if reason is None:
            return None
        request = BrakeRequest(reason=reason)
        if not self.log_decisions:
            return request
        return FirmwareResponse(
            brake_request=request,
            logs=(
                FirmwareLog(
                    text=f"Brake requested: {reason}; decoded {observation.voltage_v:.6f} V; sample age {inputs.time_s - observation.sample_time_s:.6f} s",
                    level="warning",
                ),
            ),
        )
