"""TB6612 channel control and averaged motor coupling, not switching simulation."""

from typing import Literal

from pydantic import StrictBool

from roboforge.config import Nonnegative, Probability, Real, Schema
from roboforge.core import finite

from .motor import DCMotor, MotorPoint


class DriverInputs(Schema):
    standby: StrictBool = False  # STBY high enables the chip.
    in1: StrictBool = False
    in2: StrictBool = False
    pwm_duty: Probability = 1

    def resolve(self) -> tuple[str, float | None]:
        """Return mode and signed average supply fraction; None is high impedance."""
        if not self.standby:
            return "standby", None
        if self.in1 and self.in2:
            return "brake", 0.0
        if not self.in1 and not self.in2:
            if self.pwm_duty != 1:
                raise ValueError(
                    "stop mode is supported only with PWM high; low/low/PWM-low is not specified in the control table"
                )
            return "coast", None
        if self.pwm_duty == 0:
            return "brake", 0.0
        return ("forward", self.pwm_duty) if self.in1 else ("reverse", -self.pwm_duty)


class DriverPoint(Schema):
    mode: Literal["forward", "reverse", "brake", "coast", "standby"]
    high_impedance: bool
    commanded_voltage_v: Real | None
    supply_current_a: Real
    supply_power_w: Real
    conduction_loss_w: Nonnegative
    motor: MotorPoint
    diagnostics: tuple[str, ...]


class TB6612Driver(Schema):
    """One channel with caller-supplied supplies and prescribed shaft speed.

    A second channel can use the same model independently. Shared package heating,
    PWM ripple, dead time, switching loss and supply coupling are not modeled.
    Resistance defaults to the datasheet's typical 0.5-ohm bridge approximation;
    the same resistance in brake mode is an explicit modeling assumption.
    """

    bridge_resistance_ohm: Nonnegative = 0.5

    def evaluate(
        self,
        motor: DCMotor,
        inputs: DriverInputs,
        vm_v: float,
        vcc_v: float,
        output_speed_rad_s: float,
    ) -> DriverPoint:
        """Resolve a supported truth-table command and solve average winding current."""
        vm = finite(vm_v, "VM")
        vcc = finite(vcc_v, "VCC")
        speed = finite(output_speed_rad_s, "output speed")
        if not 2.5 <= vm <= 13.5 or not 2.7 <= vcc <= 5.5:
            raise ValueError("TB6612 estimate requires VM 2.5–13.5 V and VCC 2.7–5.5 V")
        mode, fraction = inputs.resolve()
        command = None if fraction is None else fraction * vm
        rotor_speed = finite(speed * motor.gear_ratio, "rotor speed")
        emf = finite(motor.motor_constant_si * rotor_speed, "back EMF")
        diagnostics = ["thermal_and_switching_effects_unmodeled"]
        if command is None:
            # An ideal open circuit develops back EMF, with zero armature current.
            point = motor.operating_point(emf, speed)
            current, supply_current, loss = 0.0, 0.0, 0.0
            if abs(emf) > vm:
                diagnostics.append("open_circuit_diode_clamping_unmodeled")
        else:
            current = finite(
                (command - emf) / (motor.winding_resistance_ohm + self.bridge_resistance_ohm),
                "driver current",
            )
            terminal = finite(
                command - current * self.bridge_resistance_ohm, "motor terminal voltage"
            )
            point = motor.operating_point(terminal, speed)
            supply_current = finite(command / vm * current, "supply current")
            loss = finite(current * current * self.bridge_resistance_ohm, "driver loss")
            if 0 < inputs.pwm_duty < 1 and mode in {"forward", "reverse"}:
                diagnostics.append("averaged_pwm_ripple_and_rms_loss_unmodeled")
            if supply_current < 0:
                diagnostics.append("regenerative_supply_acceptance_unmodeled")
        if vm < 5:
            diagnostics.append("bridge_resistance_low_voltage_extrapolation")
        # Screening reference from the non-PWM operating-range table, not a clamp
        # or permission to use the absolute-maximum pulse-current ratings.
        reference_current = 1.0 if vm >= 4.5 else 0.4
        if abs(current) > reference_current:
            diagnostics.append("non_pwm_operating_current_reference_exceeded")
        return DriverPoint(
            mode=mode,
            high_impedance=command is None,
            commanded_voltage_v=command,
            supply_current_a=supply_current,
            supply_power_w=vm * supply_current,
            conduction_loss_w=loss,
            motor=point,
            diagnostics=tuple(diagnostics),
        )
