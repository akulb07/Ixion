"""Explicit power conditions for the bounded rover fixture."""

from typing import Literal

from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Probability, Real, Schema

from .driver import DriverInputs

DriverMode = Literal["forward", "reverse", "brake", "coast", "standby"]


class RoverPowerInterval(Schema):
    time_s: Nonnegative
    speed_m_s: Nonnegative
    soc: Probability
    pack_current_a: Nonnegative
    pack_voltage_v: Nonnegative
    pack_power_w: Nonnegative
    auxiliary_power_w: Nonnegative
    driver_modes: tuple[DriverMode, DriverMode]


class RoverPowerResponse(Schema):
    time_s: Nonnegative
    action: Literal["coast", "brake"]
    distance_m: Nonnegative
    terminal_energy_j: Nonnegative
    motor_inputs: tuple[DriverInputs, DriverInputs]


class RoverPowerConditions(Schema):
    initial_soc: Probability = Field(default=1, gt=0)
    auxiliary_current_a: Nonnegative = 0
    vcc_v: Real = Field(default=3.3, ge=2.7, le=5.5)
    minimum_pack_voltage_v: Real | None = Field(default=None, gt=2.5, le=13.5)
    voltage_limit_action: Literal["end", "coast", "brake"] = "end"

    @model_validator(mode="after")
    def check_response(self):
        if self.voltage_limit_action != "end" and self.minimum_pack_voltage_v is None:
            raise ValueError("voltage-limit response requires a declared voltage limit")
        return self

    @property
    def uses_default_power(self) -> bool:
        return all(
            getattr(self, name) == field.default
            for name, field in RoverPowerConditions.model_fields.items()
        )
