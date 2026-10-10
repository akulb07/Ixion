import pytest
from pydantic import ValidationError

from roboforge.hardware.driver import DriverInputs, TB6612Driver
from roboforge.hardware.motor import DCMotor


def evaluate(in1=True, in2=False, duty=1, speed=0, standby=True, vm=6, vcc=3.3):
    return TB6612Driver().evaluate(
        DCMotor(winding_resistance_ohm=2, motor_constant_si=0.1),
        DriverInputs(standby=standby, in1=in1, in2=in2, pwm_duty=duty),
        vm,
        vcc,
        speed,
    )


def test_direction_and_loss_at_stall():
    forward = evaluate()
    reverse = evaluate(False, True)
    assert forward.mode == "forward" and reverse.mode == "reverse"
    assert forward.motor.current_a == pytest.approx(2.4)
    assert reverse.motor.current_a == pytest.approx(-2.4)
    assert forward.motor.voltage_v == pytest.approx(4.8)
    assert forward.supply_power_w == pytest.approx(
        forward.motor.electrical_power_w + forward.conduction_loss_w
    )
    assert "non_pwm_operating_current_reference_exceeded" in forward.diagnostics


def test_pwm_scaling_and_supply_current():
    point = evaluate(duty=0.5)
    assert point.motor.current_a == pytest.approx(1.2)
    assert point.supply_current_a == pytest.approx(0.6)
    assert "averaged_pwm_ripple_and_rms_loss_unmodeled" in point.diagnostics


@pytest.mark.parametrize(
    "in1,in2,duty", [(True, True, 1), (True, True, 0), (True, False, 0), (False, True, 0)]
)
def test_short_brake_is_not_coasting(in1, in2, duty):
    point = evaluate(in1, in2, duty, speed=10)
    assert point.mode == "brake" and not point.high_impedance
    assert point.motor.current_a == pytest.approx(-0.4)
    assert point.motor.output_torque_nm < 0
    assert point.supply_current_a == 0


@pytest.mark.parametrize("standby", [False, True])
def test_high_impedance_has_no_armature_current(standby):
    point = evaluate(False, False, 1, speed=10, standby=standby)
    assert point.mode == ("coast" if standby else "standby")
    assert point.high_impedance and point.commanded_voltage_v is None
    assert point.motor.current_a == 0 and point.supply_power_w == 0


@pytest.mark.parametrize("voltage,speed,duty", [(6, 10, 1), (6, 100, 1), (6, -10, 0.5)])
def test_signed_power_balance(voltage, speed, duty):
    point = evaluate(vm=voltage, speed=speed, duty=duty)
    assert point.supply_power_w == pytest.approx(
        point.motor.electrical_power_w + point.conduction_loss_w
    )


def test_regeneration_is_reported_not_clipped():
    point = evaluate(speed=100)
    assert point.supply_current_a < 0
    assert "regenerative_supply_acceptance_unmodeled" in point.diagnostics


def test_unspecified_stop_input_combination_is_rejected():
    with pytest.raises(ValueError, match="PWM high"):
        evaluate(False, False, 0)


@pytest.mark.parametrize("vm,vcc", [(2, 3.3), (14, 3.3), (6, 2), (6, 6), (float("nan"), 3.3)])
def test_unsupported_supplies_rejected(vm, vcc):
    with pytest.raises(ValueError):
        evaluate(vm=vm, vcc=vcc)


def test_logic_inputs_are_strict_and_duty_is_bounded():
    with pytest.raises(ValidationError):
        DriverInputs(standby=1)
    with pytest.raises(ValidationError):
        DriverInputs(pwm_duty=1.1)
