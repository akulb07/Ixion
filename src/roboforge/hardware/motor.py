"""Quasi-static brushed DC gearmotor estimates, independent of a physics engine."""

import math

from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Positive, Real, Schema
from roboforge.core import finite, positive

from .project import Component


class MotorPoint(Schema):
    """Signed operating point. Positive current/power enters the motor."""

    voltage_v: Real
    current_a: Real
    rotor_speed_rad_s: Real
    output_speed_rad_s: Real
    rotor_rpm: Real
    output_rpm: Real
    back_emf_v: Real
    electromagnetic_torque_nm: Real
    output_torque_nm: Real
    load_torque_nm: Real
    net_output_torque_nm: Real
    electrical_power_w: Real
    output_power_w: Real
    copper_loss_w: Nonnegative
    friction_loss_w: Nonnegative
    gearbox_loss_w: Nonnegative


class DCMotor(Schema):
    """Instantaneous electrical equilibrium with viscous rotor friction.

    One SI constant enforces Kt = Ke. Zero applied voltage is a shorted motor,
    not an open-circuit/coasting command. No inductance or time integration.
    """

    winding_resistance_ohm: Positive
    motor_constant_si: Positive
    rotor_friction_nm_per_rad_s: Nonnegative = 0
    gear_ratio: Positive = Field(default=1, ge=1)
    gear_efficiency: Positive = Field(default=1, le=1)

    def operating_point(
        self, voltage_v: float, output_speed_rad_s: float, load_torque_nm: float = 0
    ) -> MotorPoint:
        """Evaluate at prescribed output speed; positive load subtracts torque.

        Load is signed in the output coordinate, not automatically opposed to
        speed. The returned net torque does not integrate inertia or accelerate
        a robot. Backdriving uses inverse gearbox efficiency to conserve energy.
        """
        voltage = finite(voltage_v, "motor voltage")
        speed = finite(output_speed_rad_s, "output speed")
        load = finite(load_torque_nm, "load torque")
        rotor_speed = finite(speed * self.gear_ratio, "rotor speed")
        emf = finite(self.motor_constant_si * rotor_speed, "back EMF")
        current = finite((voltage - emf) / self.winding_resistance_ohm, "current")
        torque = finite(self.motor_constant_si * current, "electromagnetic torque")
        friction = finite(self.rotor_friction_nm_per_rad_s * rotor_speed, "friction torque")
        shaft_torque = finite(torque - friction, "rotor shaft torque")
        shaft_power = finite(shaft_torque * rotor_speed, "rotor shaft power")
        factor = self.gear_efficiency if shaft_power >= 0 else 1 / self.gear_efficiency
        output_torque = finite(shaft_torque * self.gear_ratio * factor, "output torque")
        output_power = finite(output_torque * speed, "output power")
        # Roundoff can leave a tiny negative difference for a lossless gear.
        gearbox_loss = max(0.0, finite(shaft_power - output_power, "gearbox loss"))
        return MotorPoint(
            voltage_v=voltage,
            current_a=current,
            rotor_speed_rad_s=rotor_speed,
            output_speed_rad_s=speed,
            rotor_rpm=rotor_speed * 60 / math.tau,
            output_rpm=speed * 60 / math.tau,
            back_emf_v=emf,
            electromagnetic_torque_nm=torque,
            output_torque_nm=output_torque,
            load_torque_nm=load,
            net_output_torque_nm=output_torque - load,
            electrical_power_w=voltage * current,
            output_power_w=output_power,
            copper_loss_w=current * current * self.winding_resistance_ohm,
            friction_loss_w=friction * rotor_speed,
            gearbox_loss_w=gearbox_loss,
        )


class MotorDatasheet(Schema):
    """Common gearmotor inputs. RPM and stall torque are at the gearbox output."""

    nominal_voltage_v: Positive
    output_no_load_rpm: Positive
    no_load_current_a: Nonnegative
    stall_current_a: Positive
    output_stall_torque_nm: Positive
    gear_ratio: Positive = Field(ge=1)

    @model_validator(mode="after")
    def current_order(self):
        if self.no_load_current_a >= self.stall_current_a:
            raise ValueError("no-load current must be below stall current")
        return self

    def derive(self) -> DCMotor:
        """Fit R, K, viscous friction and effective gear efficiency.

        Reject inconsistent data rather than silently inventing efficiency above
        one. This fit does not identify inertia, inductance or thermal behavior.
        """
        resistance = positive(self.nominal_voltage_v / self.stall_current_a, "derived resistance")
        omega = positive(
            self.output_no_load_rpm * math.tau / 60 * self.gear_ratio, "derived rotor speed"
        )
        constant = positive(
            (self.nominal_voltage_v - self.no_load_current_a * resistance) / omega,
            "derived motor constant",
        )
        friction = constant * self.no_load_current_a / omega
        denominator = positive(
            constant * self.stall_current_a * self.gear_ratio, "ideal stall torque"
        )
        efficiency = self.output_stall_torque_nm / denominator
        if not math.isfinite(efficiency) or efficiency > 1:
            raise ValueError(
                "datasheet values imply gear efficiency above one; check shaft units and ratings"
            )
        return DCMotor(
            winding_resistance_ohm=resistance,
            motor_constant_si=constant,
            rotor_friction_nm_per_rad_s=friction,
            gear_ratio=self.gear_ratio,
            gear_efficiency=efficiency,
        )

    @classmethod
    def from_component(cls, component: Component) -> "MotorDatasheet":
        """Read explicitly named/unit-tagged gearmotor parameters from a project."""
        if component.category != "motor":
            raise ValueError("selected component must be a motor")
        parameters = {p.name: p for p in component.parameters}
        required = {
            "nominal_voltage": ("nominal_voltage_v", "V"),
            "output_no_load_speed": ("output_no_load_rpm", "rpm"),
            "no_load_current": ("no_load_current_a", "A"),
            "stall_current": ("stall_current_a", "A"),
            "output_stall_torque": ("output_stall_torque_nm", "N*m"),
            "gear_ratio": ("gear_ratio", "1"),
        }
        values = {}
        for name, (field, unit) in required.items():
            parameter = parameters.get(name)
            if parameter is None or parameter.unit != unit:
                raise ValueError(f"motor requires {name} with unit {unit}")
            values[field] = parameter.value
        return cls(**values)
