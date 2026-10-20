import math

import pytest

from roboforge.hardware.battery import BatteryState, OCVPoint
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.rolling import rolling_load
from roboforge.hardware.rolling_runtime import RollingRuntime
from roboforge.hardware.shaft_dynamics import ShaftLoad
from roboforge.hardware.voltage_limit import locate_voltage_limit
from tests.unit.test_rolling import setup


def fixture():
    system, body = setup(DriverInputs(standby=False))
    battery = system.battery.model_copy(
        update={
            "capacity_ah": 1e-5,
            "ocv_curve": (OCVPoint(soc=0, voltage_v=6), OCVPoint(soc=1, voltage_v=8)),
        }
    )
    return system.model_copy(update={"battery": battery}), body


def test_voltage_crossing_matches_analytic_constant_current_discharge_inside_one_step():
    system, body = fixture()
    runtime = RollingRuntime(system, BatteryState(), body, auxiliary_current_a=0.2)
    limited = runtime.step_until_voltage_limit(0.15, 7)
    event = limited.event
    # Half of 1e-5 Ah discharged at 0.2 A takes 0.09 s.
    assert event.phase == "physics_step"
    assert event.time_s == pytest.approx(0.09, abs=2e-10)
    assert event.time_s <= 0.09 <= event.bracket_end_time_s + 1e-15
    assert event.bracket_end_time_s - event.time_s <= event.time_tolerance_s * 1.001
    assert event.voltage_v > 7 >= event.bracket_end_voltage_v
    assert limited.step.end.shaft.state.soc == pytest.approx(0.5, abs=1e-8)
    assert runtime.sample.speed_m_s == 0
    assert runtime.sample.shaft.point.battery.current_a == 0.2
    assert runtime.time_s == event.time_s


def test_limit_prevents_a_step_from_entering_unsupported_driver_voltage():
    system, body = fixture()
    system = system.model_copy(
        update={
            "battery": system.battery.model_copy(
                update={"ocv_curve": (OCVPoint(soc=0, voltage_v=1), OCVPoint(soc=1, voltage_v=8))}
            )
        }
    )
    runtime = RollingRuntime(system, BatteryState(), body, auxiliary_current_a=0.2)
    limited = runtime.step_until_voltage_limit(0.17, 3)
    assert limited.event is not None
    assert runtime.sample.shaft.point.battery.terminal_voltage_v > 3
    assert limited.event.bracket_end_voltage_v <= 3


def test_command_voltage_limit_retains_precommand_state_and_records_candidate_inputs():
    system, body = setup(DriverInputs(standby=False))
    system = system.model_copy(
        update={"battery": system.battery.model_copy(update={"internal_resistance_ohm": 0.5})}
    )
    runtime = RollingRuntime(system, BatteryState(), body)
    before = runtime.sample, runtime.system
    drive = DriverInputs(standby=True, in1=True)
    result = runtime.step_until_voltage_limit(0.01, 7, (drive, drive))
    assert result.step is None
    assert result.event.phase == "command_boundary"
    assert result.event.time_s == 0
    assert result.event.voltage_v < 7
    assert result.event.motor_inputs == (drive, drive)
    assert (runtime.sample, runtime.system) == before


def test_no_crossing_uses_the_same_step_as_the_ordinary_runtime():
    system, body = setup()
    state = BatteryState()
    first, second = RollingRuntime(system, state, body), RollingRuntime(system, state, body)
    expected = first.step(0.01)
    actual = second.step_until_voltage_limit(0.01, 6)
    assert actual.event is None
    assert actual.step == expected


def test_algebraic_voltage_is_explicitly_not_an_accepted_driver_point():
    system, body = fixture()
    system = system.model_copy(
        update={"battery": system.battery.model_copy(update={"internal_resistance_ohm": 1})}
    )
    assert system.algebraic_supply_voltage(1, 7) == 1
    with pytest.raises(ValueError, match="VM"):
        system.operating_point(BatteryState(), auxiliary_current_a=7)


def test_a_limit_does_not_hide_charge_exhaustion_when_voltage_never_crosses():
    system, body = fixture()
    runtime = RollingRuntime(system, BatteryState(), body, auxiliary_current_a=0.2)
    before = runtime.sample, runtime.system
    with pytest.raises(ValueError, match="remaining charge"):
        runtime.step_until_voltage_limit(1, 3)
    assert (runtime.sample, runtime.system) == before


@pytest.mark.parametrize("threshold", [2.5, 0, 14, True, float("nan"), float("inf")])
def test_unsupported_thresholds_are_rejected_without_advancing(threshold):
    system, body = fixture()
    runtime = RollingRuntime(system, BatteryState(), body)
    before = runtime.sample
    with pytest.raises(ValueError):
        runtime.step_until_voltage_limit(0.1, threshold)
    assert runtime.sample == before


def test_one_load_per_channel_is_required():
    system, body = fixture()
    with pytest.raises(ValueError, match="one load"):
        locate_voltage_limit(system, BatteryState(), (rolling_load(body),), 0.1, 7)


def test_ocv_knots_find_an_internal_dip_even_when_endpoint_voltage_has_recovered():
    command = DriverInputs(standby=True, in1=True, pwm_duty=0.5)
    system, _ = setup(command)
    battery = system.battery.model_copy(
        update={
            "internal_resistance_ohm": 1,
            "capacity_ah": 0.0001,
            "ocv_curve": (
                OCVPoint(soc=0, voltage_v=7),
                OCVPoint(soc=0.5, voltage_v=7),
                OCVPoint(soc=1, voltage_v=8),
            ),
        }
    )
    system = system.model_copy(update={"battery": battery})
    # The held-Euler acceleration raises back EMF while the first OCV segment
    # drops steeply. The voltage dip at SOC 0.5 occurs before endpoint recovery.
    event = locate_voltage_limit(
        system, BatteryState(), (ShaftLoad(inertia_kg_m2=0.001),) * 2, 0.2, 6.55
    )
    assert event.time_s == pytest.approx(0.0675, abs=3e-10)
    assert event.bracket_end_voltage_v <= 6.55
    initial = system.operating_point(BatteryState())
    alpha = initial.channels[0].motor.output_torque_nm / 0.001
    soc = 1 - initial.battery.current_a * 0.2 / (0.0001 * 3600)
    endpoint = system.model_copy(
        update={
            "channels": tuple(
                c.model_copy(update={"output_speed_rad_s": alpha * 0.2}) for c in system.channels
            )
        }
    )
    assert endpoint.algebraic_supply_voltage(soc) > 6.55


def test_a_very_early_crossing_still_has_a_positive_supported_time_at_the_origin():
    system, body = fixture()
    event = locate_voltage_limit(
        system,
        BatteryState(),
        (rolling_load(body),) * 2,
        0.1,
        math.nextafter(8, 0),
        auxiliary_current_a=0.2,
    )
    assert 0 < event.time_s < 1e-12
    assert event.voltage_v > event.threshold_v >= event.bracket_end_voltage_v


def test_crossing_below_absolute_clock_resolution_is_an_explicit_error():
    system, body = fixture()
    with pytest.raises(ValueError, match="below simulation time resolution"):
        locate_voltage_limit(
            system,
            BatteryState(),
            (rolling_load(body),) * 2,
            0.1,
            math.nextafter(8, 0),
            time_s=1e12,
            auxiliary_current_a=0.2,
        )
