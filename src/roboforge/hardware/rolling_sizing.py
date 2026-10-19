"""Inverse sizing for one forward, symmetric rolling operating point."""

import math
from typing import Literal

from pydantic import Field

from roboforge.config import Nonnegative, Positive, Real, Schema
from roboforge.core import finite

from .battery import BatteryModel, BatteryState
from .driver import DriverInputs, TB6612Driver
from .motor import DCMotor
from .powertrain import MotorChannel, Powertrain, PowertrainPoint
from .rolling import RollingBody, grade_force_n, rolling_inertia


class RollingDemand(Schema):
    speed_m_s: Nonnegative
    acceleration_m_s2: Nonnegative = 0
    max_pwm_duty: Positive = Field(default=1, le=1)
    auxiliary_current_a: Nonnegative = 0
    vcc_v: Real = Field(default=3.3, ge=2.7, le=5.5)


class SizingCheck(Schema):
    code: str
    status: Literal["passed", "failed", "not_assessed"]
    actual: Real | None
    minimum: Real | None = None
    maximum: Real | None = None
    unit: str
    explanation: str


class RollingSizingResult(Schema):
    status: Literal["checks_passed", "checks_failed"]
    required_contact_force_per_wheel_n: Nonnegative
    traction_limit_per_wheel_n: Nonnegative
    traction_margin_per_wheel_n: Real
    required_output_torque_per_wheel_nm: Nonnegative
    required_motor_current_per_channel_a: Nonnegative
    required_average_bridge_voltage_v: Nonnegative | None
    required_pwm_duty: Nonnegative | None
    pwm_margin: Real | None
    command: DriverInputs | None
    point: PowertrainPoint | None
    checks: tuple[SizingCheck, ...]


def size_rolling_drive(
    motor: DCMotor,
    battery: BatteryModel,
    driver: TB6612Driver,
    state: BatteryState,
    body: RollingBody,
    demand: RollingDemand,
) -> RollingSizingResult:
    """Solve equal motor torque and a shared sagged supply at prescribed speed.

    Only nonnegative speed, acceleration and grade are supported. The lower-duty,
    higher-voltage battery solution is selected. This is an instantaneous demand,
    not a trajectory, controller, thermal assessment or proof of hill holding.
    """
    if body.grade_rad < 0:
        raise ValueError("rolling sizing supports nonnegative uphill grades only")
    radius = body.wheel_radius_m
    omega = finite(demand.speed_m_s / radius, "required shaft speed")
    alpha = finite(demand.acceleration_m_s2 / radius, "required shaft acceleration")
    force = finite(
        (body.total_mass_kg * demand.acceleration_m_s2 + grade_force_n(body)) / 2,
        "required contact force",
    )
    traction = finite(
        body.friction_coefficient
        * body.total_mass_kg
        * body.gravity_m_s2
        * math.cos(body.grade_rad)
        * body.driven_weight_fraction
        / 2,
        "traction limit",
    )
    torque = finite(
        rolling_inertia(body) * alpha + grade_force_n(body) * radius / 2,
        "required output torque",
    )
    rotor_speed = finite(omega * motor.gear_ratio, "required rotor speed")
    current = finite(
        (
            torque / (motor.gear_ratio * motor.gear_efficiency)
            + motor.rotor_friction_nm_per_rad_s * rotor_speed
        )
        / motor.motor_constant_si,
        "required motor current",
    )
    checks = []

    def check(code, actual, unit, explanation, minimum=None, maximum=None):
        assessed = actual is not None and (minimum is not None or maximum is not None)
        passed = (
            assessed
            and (minimum is None or actual >= minimum)
            and (maximum is None or actual <= maximum)
        )
        checks.append(
            SizingCheck(
                code=code,
                status="not_assessed" if not assessed else "passed" if passed else "failed",
                actual=actual,
                minimum=minimum,
                maximum=maximum,
                unit=unit,
                explanation=explanation,
            )
        )

    check(
        "traction",
        force,
        "N",
        "Required contact force per driven wheel; no slip model.",
        maximum=traction,
    )
    check(
        "battery_charge",
        state.soc,
        "1",
        "Positive charge is required for a powered estimate.",
        minimum=math.nextafter(0, 1),
    )
    ocv = battery.open_circuit_voltage(state.soc)
    available = finite(
        ocv - battery.internal_resistance_ohm * demand.auxiliary_current_a, "auxiliary-loaded OCV"
    )
    voltage = duty = bridge_voltage = None
    command = point = None
    if current == 0:
        # An ideal lossless wheel with no force demand can coast. No active bridge
        # voltage or holding behavior is needed, even if its back EMF is nonzero.
        duty, voltage = 0.0, available
        command = DriverInputs(standby=True)
        check(
            "battery_power_balance",
            available,
            "V",
            "Auxiliary load must leave positive pack voltage.",
            minimum=math.nextafter(0, 1),
        )
        check(
            "coasting_back_emf",
            motor.motor_constant_si * rotor_speed,
            "V",
            "Open-circuit diode clamping is outside this model.",
            maximum=max(0, available),
        )
    else:
        bridge_voltage = finite(
            motor.motor_constant_si * rotor_speed
            + current * (motor.winding_resistance_ohm + driver.bridge_resistance_ohm),
            "required average bridge voltage",
        )
        # V = OCV - Rb*(2*d*I + Iaux), d*V = E + I*(Rm+Rd).
        discriminant = finite(
            available * available - 8 * battery.internal_resistance_ohm * current * bridge_voltage,
            "battery demand discriminant",
        )
        solvable = available > 0 and discriminant >= 0
        check(
            "battery_power_balance",
            1 if solvable else 0,
            "1",
            "A positive higher-voltage battery solution must exist for this demand.",
            minimum=1,
        )
        if solvable:
            # Rationalized small root avoids cancellation, including Rb == 0.
            duty = finite(
                2 * bridge_voltage / (available + math.sqrt(discriminant)), "required PWM duty"
            )
            voltage = finite(bridge_voltage / duty, "required pack voltage")
            if duty <= 1:
                command = DriverInputs(standby=True, in1=True, pwm_duty=duty)
    check(
        "pwm_ceiling",
        duty,
        "1",
        "Required duty must fit the declared controller ceiling; it is never clamped.",
        maximum=demand.max_pwm_duty,
    )
    check(
        "driver_vm",
        voltage,
        "V",
        "TB6612 modeled operating voltage range, at the selected battery solution.",
        minimum=2.5,
        maximum=13.5,
    )
    if command is not None and state.soc > 0 and voltage is not None and 2.5 <= voltage <= 13.5:
        system = Powertrain(
            battery=battery,
            driver=driver,
            channels=(MotorChannel(motor=motor, inputs=command, output_speed_rad_s=omega),) * 2,
        )
        point = system.operating_point(state, demand.vcc_v, demand.auxiliary_current_a)
    # These remain screening bounds, never simulated protection or RMS heating.
    pack_current = None if point is None else point.battery.current_a
    check(
        "battery_continuous_current",
        pack_current,
        "A",
        "Declared whole-pack continuous rating; unknown stays unassessed.",
        maximum=battery.max_continuous_current_a,
    )
    check(
        "battery_burst_current",
        pack_current,
        "A",
        "Declared whole-pack burst rating; no burst duration or BMS model.",
        maximum=battery.max_burst_current_a,
    )
    reference = None if voltage is None else 1.0 if voltage >= 4.5 else 0.4
    check(
        "driver_non_pwm_current_reference",
        current,
        "A",
        "Non-PWM operating-table screening reference only; averaged PWM current is not RMS current or a thermal rating.",
        maximum=reference,
    )
    check(
        "motor_continuous_current",
        current,
        "A",
        "The fitted motor model has no continuous-current or thermal rating.",
    )
    return RollingSizingResult(
        status="checks_failed" if any(c.status == "failed" for c in checks) else "checks_passed",
        required_contact_force_per_wheel_n=force,
        traction_limit_per_wheel_n=traction,
        traction_margin_per_wheel_n=finite(traction - force, "traction margin"),
        required_output_torque_per_wheel_nm=torque,
        required_motor_current_per_channel_a=current,
        required_average_bridge_voltage_v=bridge_voltage,
        required_pwm_duty=duty,
        pwm_margin=None if duty is None else finite(demand.max_pwm_duty - duty, "PWM margin"),
        command=command if point is not None else None,
        point=point,
        checks=tuple(checks),
    )
