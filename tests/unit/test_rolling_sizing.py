import math

import pytest

from roboforge.hardware.battery import BatteryState, OCVPoint
from roboforge.hardware.powertrain import MotorChannel, Powertrain
from roboforge.hardware.rolling import run_rolling_bench
from roboforge.hardware.rolling_runtime import RollingRuntime
from roboforge.hardware.rolling_sizing import RollingDemand, size_rolling_drive
from tests.unit.test_rolling import setup


def sizing(*, body_changes=None, battery_changes=None, motor_changes=None, soc=1, **kwargs):
    system, body = setup()
    body = type(body).model_validate({**body.model_dump(), **(body_changes or {})})
    battery = type(system.battery).model_validate(
        {**system.battery.model_dump(), **(battery_changes or {})}
    )
    motor = type(system.channels[0].motor).model_validate(
        {**system.channels[0].motor.model_dump(), **(motor_changes or {})}
    )
    demand = RollingDemand(speed_m_s=kwargs.pop("speed_m_s", 0.3), **kwargs)
    result = size_rolling_drive(motor, battery, system.driver, BatteryState(soc=soc), body, demand)
    return result, motor, battery, system.driver, body, demand


def checks(result):
    return {c.code: c for c in result.checks}


def test_inverse_matches_independent_force_torque_and_constant_supply_equations():
    result, motor, battery, driver, body, demand = sizing(
        body_changes={"grade_rad": 0.1}, acceleration_m_s2=0.2
    )
    force = (2 * 0.2 + 2 * 9.80665 * math.sin(0.1)) / 2
    torque = 0.002 * 0.2 / 0.1 + force * 0.1
    current = torque / 0.1
    voltage = 0.1 * 0.3 / 0.1 + current * 2.5
    assert result.required_contact_force_per_wheel_n == pytest.approx(force)
    assert result.required_output_torque_per_wheel_nm == pytest.approx(torque)
    assert result.required_motor_current_per_channel_a == pytest.approx(current)
    assert result.required_pwm_duty == pytest.approx(voltage / 8)
    assert result.point.channels[0].motor.output_torque_nm == pytest.approx(torque)
    assert result.point.battery.current_a == pytest.approx(2 * current * voltage / 8)


@pytest.mark.parametrize("resistance", [0, 1e-12, 0.15, 1])
def test_inverse_replays_through_existing_shared_supply_and_rolling_runtime(resistance):
    result, motor, battery, driver, body, demand = sizing(
        body_changes={"grade_rad": 0.05},
        battery_changes={"internal_resistance_ohm": resistance},
        acceleration_m_s2=0.1,
        auxiliary_current_a=0.1,
    )
    assert result.point is not None
    system = Powertrain(
        battery=battery,
        driver=driver,
        channels=(
            MotorChannel(
                motor=motor,
                inputs=result.command,
                output_speed_rad_s=demand.speed_m_s / body.wheel_radius_m,
            ),
        )
        * 2,
    )
    direct = system.operating_point(BatteryState(), auxiliary_current_a=0.1)
    assert direct == result.point
    assert direct.channels[0].motor.current_a == pytest.approx(
        result.required_motor_current_per_channel_a
    )
    # Runtime has no auxiliary-load input, so use the solved voltage with a
    # constant supply to independently check the mechanical demand.
    terminal = direct.battery.terminal_voltage_v
    constant = battery.model_copy(
        update={
            "internal_resistance_ohm": 0,
            "ocv_curve": (OCVPoint(soc=0, voltage_v=terminal), OCVPoint(soc=1, voltage_v=terminal)),
        }
    )
    runtime = RollingRuntime(system.model_copy(update={"battery": constant}), BatteryState(), body)
    assert runtime.sample.acceleration_m_s2 == pytest.approx(demand.acceleration_m_s2)
    assert runtime.sample.contact_force_per_wheel_n == pytest.approx(
        result.required_contact_force_per_wheel_n
    )


def test_steady_uphill_operating_point_preserves_speed_over_a_rolling_trace():
    result, motor, battery, driver, body, demand = sizing(body_changes={"grade_rad": 0.05})
    system = Powertrain(
        battery=battery,
        driver=driver,
        channels=(
            MotorChannel(
                motor=motor,
                inputs=result.command,
                output_speed_rad_s=demand.speed_m_s / body.wheel_radius_m,
            ),
        )
        * 2,
    )
    trace = run_rolling_bench(system, BatteryState(), body, 0.2, 0.001)
    assert trace.samples[-1].speed_m_s == pytest.approx(demand.speed_m_s)
    assert trace.samples[-1].distance_m == pytest.approx(demand.speed_m_s * 0.2)


def test_sag_gear_losses_rotor_friction_and_auxiliary_load_balance_power():
    result, motor, battery, driver, body, demand = sizing(
        body_changes={"grade_rad": 0.1},
        battery_changes={"internal_resistance_ohm": 0.15},
        motor_changes={
            "gear_ratio": 2,
            "gear_efficiency": 0.8,
            "rotor_friction_nm_per_rad_s": 0.001,
        },
        acceleration_m_s2=0.2,
        auxiliary_current_a=0.1,
    )
    point = result.point
    assert point.battery.terminal_voltage_v < point.battery.open_circuit_voltage_v
    assert point.battery.current_a == pytest.approx(
        2 * result.required_pwm_duty * result.required_motor_current_per_channel_a + 0.1
    )
    assert point.battery.terminal_voltage_v == pytest.approx(8 - 0.15 * point.battery.current_a)
    mechanical = (
        body.total_mass_kg * demand.acceleration_m_s2
        + 2
        * body.rotating_inertia_per_wheel_kg_m2
        * demand.acceleration_m_s2
        / body.wheel_radius_m**2
        + body.total_mass_kg * body.gravity_m_s2 * math.sin(body.grade_rad)
    ) * demand.speed_m_s
    losses = sum(
        c.conduction_loss_w
        + c.motor.copper_loss_w
        + c.motor.friction_loss_w
        + c.motor.gearbox_loss_w
        for c in point.channels
    )
    assert sum(c.motor.output_power_w for c in point.channels) == pytest.approx(mechanical)
    assert 8 * point.battery.current_a == pytest.approx(
        mechanical + losses + point.auxiliary_power_w + point.battery.internal_loss_w
    )


def test_lower_duty_higher_voltage_root_is_selected_when_two_solutions_exist():
    result, *_ = sizing(
        speed_m_s=0,
        acceleration_m_s2=5 / 6,
        motor_changes={"winding_resistance_ohm": 0.5},
        battery_changes={"internal_resistance_ohm": 5},
    )
    assert result.required_motor_current_per_channel_a == pytest.approx(1)
    assert result.required_pwm_duty == pytest.approx(2 / (8 + math.sqrt(24)))
    assert result.point.battery.terminal_voltage_v == pytest.approx((8 + math.sqrt(24)) / 2)


def test_impossible_sag_demand_is_explained_without_inventing_an_operating_point():
    result, *_ = sizing(acceleration_m_s2=1, battery_changes={"internal_resistance_ohm": 20})
    assert result.status == "checks_failed"
    assert checks(result)["battery_power_balance"].status == "failed"
    assert checks(result)["driver_vm"].status == "not_assessed"
    assert result.required_pwm_duty is result.point is result.command is None
    assert result.required_motor_current_per_channel_a > 0


def test_excessive_auxiliary_load_cannot_select_a_negative_voltage_root():
    result, *_ = sizing(
        acceleration_m_s2=0.1,
        auxiliary_current_a=100,
        battery_changes={"internal_resistance_ohm": 1},
    )
    assert checks(result)["battery_power_balance"].status == "failed"
    assert result.required_pwm_duty is None


def test_speed_beyond_full_pwm_keeps_the_required_duty_and_rejects_the_demand():
    result, *_ = sizing(speed_m_s=20, acceleration_m_s2=0.2)
    assert result.required_pwm_duty > 1
    assert result.pwm_margin < 0
    assert checks(result)["pwm_ceiling"].status == "failed"
    assert result.point is result.command is None


def test_controller_ceiling_does_not_clamp_the_calculated_duty():
    result, *_ = sizing(acceleration_m_s2=0.1, max_pwm_duty=0.01)
    assert result.required_pwm_duty > 0.01
    assert result.command.pwm_duty == result.required_pwm_duty
    assert result.point is not None
    assert checks(result)["pwm_ceiling"].status == "failed"


def test_traction_failure_retains_the_electrical_solution_for_diagnosis():
    result, *_ = sizing(body_changes={"friction_coefficient": 0.001}, acceleration_m_s2=0.2)
    assert result.traction_margin_per_wheel_n < 0
    assert checks(result)["traction"].status == "failed"
    assert result.point is not None


@pytest.mark.parametrize("ocv", [2, 14])
def test_unsupported_driver_supply_is_a_failed_bound_without_driver_execution(ocv):
    result, *_ = sizing(
        acceleration_m_s2=0.1,
        battery_changes={
            "ocv_curve": [{"soc": 0, "voltage_v": ocv}, {"soc": 1, "voltage_v": ocv}],
        },
    )
    assert checks(result)["driver_vm"].status == "failed"
    assert result.point is None


def test_depleted_pack_does_not_publish_a_powered_point():
    result, *_ = sizing(acceleration_m_s2=0.1, soc=0)
    assert checks(result)["battery_charge"].status == "failed"
    assert result.point is None


def test_unknown_ratings_stay_unassessed_and_known_exceedances_fail_screening():
    missing, *_ = sizing(
        acceleration_m_s2=0.1,
        battery_changes={
            "max_continuous_current_a": None,
            "max_burst_current_a": None,
        },
    )
    assert missing.status == "checks_passed"
    assert checks(missing)["motor_continuous_current"].status == "not_assessed"
    assert checks(missing)["battery_continuous_current"].status == "not_assessed"
    rated, *_ = sizing(
        acceleration_m_s2=0.1,
        battery_changes={
            "max_continuous_current_a": 0.0001,
            "max_burst_current_a": 0.0002,
        },
    )
    assert rated.status == "checks_failed"
    assert checks(rated)["battery_continuous_current"].status == "failed"
    assert checks(rated)["battery_burst_current"].status == "failed"
    assert rated.required_pwm_duty == missing.required_pwm_duty


def test_current_reference_is_screening_and_does_not_clamp_torque():
    result, *_ = sizing(acceleration_m_s2=1)
    assert result.required_motor_current_per_channel_a > 1
    assert checks(result)["driver_non_pwm_current_reference"].status == "failed"
    assert result.point.channels[0].motor.current_a > 1


@pytest.mark.parametrize("speed", [0, 0.3])
def test_lossless_zero_force_demand_coasts_instead_of_synthesizing_regeneration(speed):
    result, *_ = sizing(speed_m_s=speed, auxiliary_current_a=0.1)
    assert result.required_pwm_duty == 0
    assert result.required_average_bridge_voltage_v is None
    assert result.point.channels[0].mode == "coast"
    assert result.point.battery.current_a == 0.1


def test_lossless_coasting_beyond_the_supply_flags_unmodeled_diode_clamping():
    result, *_ = sizing(speed_m_s=20)
    assert checks(result)["coasting_back_emf"].status == "failed"
    assert result.status == "checks_failed"


@pytest.mark.parametrize(
    "changes",
    [
        {"speed_m_s": -1},
        {"speed_m_s": True},
        {"speed_m_s": float("nan")},
        {"acceleration_m_s2": -1},
        {"max_pwm_duty": 0},
        {"max_pwm_duty": 1.1},
        {"auxiliary_current_a": -1},
        {"vcc_v": 2.6},
        {"vcc_v": 5.6},
    ],
)
def test_invalid_or_unsupported_demands_are_rejected(changes):
    with pytest.raises(ValueError):
        RollingDemand.model_validate({"speed_m_s": 0.3, **changes})


def test_downhill_sizing_is_rejected_before_applying_motoring_equations():
    with pytest.raises(ValueError, match="nonnegative uphill"):
        sizing(body_changes={"grade_rad": -0.1})
