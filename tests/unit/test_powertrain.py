import pytest

from roboforge.hardware.battery import BatteryModel, BatteryState, OCVPoint
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.motor import DCMotor
from roboforge.hardware.powertrain import MotorChannel, Powertrain


def system(count=2, duty=1, speed=0, resistance=0.2, inputs=None):
    battery = BatteryModel(
        chemistry="LiPo",
        capacity_ah=2,
        internal_resistance_ohm=resistance,
        ocv_curve=(OCVPoint(soc=0, voltage_v=6), OCVPoint(soc=1, voltage_v=8)),
    )
    channel = MotorChannel(
        motor=DCMotor(winding_resistance_ohm=2, motor_constant_si=0.1),
        inputs=inputs or DriverInputs(standby=True, in1=True, pwm_duty=duty),
        output_speed_rad_s=speed,
    )
    return Powertrain(battery=battery, channels=(channel,) * count)


def test_two_stalled_motors_share_one_sagged_supply():
    point = system().operating_point(BatteryState())
    expected = 8 / (1 + 0.2 * 2 / 2.5)
    assert point.battery.terminal_voltage_v == pytest.approx(expected)
    assert point.battery.current_a == pytest.approx(2 * expected / 2.5)
    assert all(c.motor.current_a == pytest.approx(expected / 2.5) for c in point.channels)
    assert (
        point.battery.terminal_voltage_v
        < system(1).operating_point(BatteryState()).battery.terminal_voltage_v
    )


def test_pwm_supply_current_differs_from_winding_current():
    point = system(duty=0.5).operating_point(BatteryState())
    assert point.battery.terminal_voltage_v == pytest.approx(8 / 1.04)
    assert point.battery.current_a == pytest.approx(
        sum(c.motor.current_a * 0.5 for c in point.channels)
    )


@pytest.mark.parametrize("duty,speed,auxiliary", [(1, 0, 0), (0.5, 10, 0.2), (1, 20, 0.1)])
def test_pack_and_channel_power_balance(duty, speed, auxiliary):
    point = system(duty=duty, speed=speed).operating_point(
        BatteryState(), auxiliary_current_a=auxiliary
    )
    loads = (
        sum(c.motor.electrical_power_w + c.conduction_loss_w for c in point.channels)
        + point.auxiliary_power_w
    )
    assert point.battery.power_w == pytest.approx(loads)
    assert point.battery.open_circuit_voltage_v * point.battery.current_a == pytest.approx(
        loads + point.battery.internal_loss_w
    )


def test_ideal_battery_has_no_sag():
    point = system(resistance=0).operating_point(BatteryState())
    assert point.battery.terminal_voltage_v == 8


def test_standby_draws_only_declared_auxiliary_current():
    point = system(inputs=DriverInputs(), speed=10).operating_point(
        BatteryState(), auxiliary_current_a=0.2
    )
    assert point.battery.current_a == 0.2
    assert point.battery.terminal_voltage_v == pytest.approx(7.96)
    assert all(c.high_impedance for c in point.channels)


def test_braking_circulates_motor_current_without_pack_current():
    point = system(inputs=DriverInputs(standby=True, in1=True, in2=True), speed=10).operating_point(
        BatteryState()
    )
    assert point.battery.current_a == 0
    assert all(c.motor.current_a < 0 for c in point.channels)


def test_reverse_at_rest_draws_positive_pack_current():
    point = system(inputs=DriverInputs(standby=True, in2=True)).operating_point(BatteryState())
    assert point.battery.current_a > 0
    assert all(c.motor.current_a < 0 for c in point.channels)


def test_regenerative_channel_is_rejected_even_with_other_loads():
    with pytest.raises(ValueError, match="supply sink"):
        system(speed=100).operating_point(BatteryState(), auxiliary_current_a=10)


def test_undervoltage_and_depletion_do_not_fabricate_results():
    with pytest.raises(ValueError, match="requires VM"):
        system(resistance=10).operating_point(BatteryState())
    with pytest.raises(ValueError, match="depleted"):
        system().operating_point(BatteryState(soc=0))


def test_negative_auxiliary_current_rejected():
    with pytest.raises(ValueError):
        system().operating_point(BatteryState(), auxiliary_current_a=-1)
