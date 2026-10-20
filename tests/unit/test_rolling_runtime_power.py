import pytest

from roboforge.hardware.battery import BatteryState, OCVPoint
from roboforge.hardware.driver import DriverInputs, TB6612Driver
from roboforge.hardware.rolling import run_rolling_bench
from roboforge.hardware.rolling_runtime import RollingRuntime
from roboforge.hardware.rolling_sizing import RollingDemand, size_rolling_drive
from tests.unit.test_rolling import setup


def test_incremental_power_conditions_match_bench_from_initial_state_onward():
    system, body = setup()
    battery = system.battery.model_copy(update={"internal_resistance_ohm": 0.15})
    system = system.model_copy(update={"battery": battery})
    state = BatteryState(soc=0.5)
    expected = run_rolling_bench(system, state, body, 0.1, 0.01, 5, 0.2)
    runtime = RollingRuntime(system, state, body, vcc_v=5, auxiliary_current_a=0.2)
    assert runtime.sample == expected.samples[0]
    for index in range(10):
        result = runtime.step(0.01)
        end = expected.samples[index + 1]
        assert result.end.speed_m_s == pytest.approx(end.speed_m_s)
        assert result.end.shaft.state.soc == pytest.approx(end.shaft.state.soc)
        assert result.end.shaft.point.battery.current_a == pytest.approx(
            end.shaft.point.battery.current_a
        )


def test_shared_auxiliary_sag_reduces_initial_acceleration():
    system, body = setup()
    system = system.model_copy(
        update={"battery": system.battery.model_copy(update={"internal_resistance_ohm": 0.2})}
    )
    bare = RollingRuntime(system, BatteryState(), body)
    loaded = RollingRuntime(system, BatteryState(), body, auxiliary_current_a=0.5)
    assert (
        loaded.sample.shaft.point.battery.terminal_voltage_v
        < bare.sample.shaft.point.battery.terminal_voltage_v
    )
    assert loaded.sample.acceleration_m_s2 < bare.sample.acceleration_m_s2
    assert loaded.sample.shaft.point.auxiliary_current_a == 0.5


def test_auxiliary_current_keeps_discharging_the_pack_after_motor_braking():
    brake = DriverInputs(standby=True, in1=True, in2=True)
    system, body = setup(brake)
    runtime = RollingRuntime(system, BatteryState(soc=0.5), body, auxiliary_current_a=0.25)
    result = runtime.step(0.1)
    capacity = system.battery.capacity_ah
    assert result.end.speed_m_s == 0
    assert result.end.shaft.point.battery.current_a == 0.25
    assert result.end.shaft.state.soc == pytest.approx(0.5 - 0.25 * 0.1 / (capacity * 3600))
    assert all(c.supply_current_a == 0 for c in result.end.shaft.point.channels)


def test_inverse_sizing_and_runtime_now_share_auxiliary_load_and_vcc_conditions():
    system, body = setup()
    body = body.model_copy(update={"grade_rad": 0.05})
    system = system.model_copy(
        update={"battery": system.battery.model_copy(update={"internal_resistance_ohm": 0.15})}
    )
    state = BatteryState(soc=0.5)
    demand = RollingDemand(speed_m_s=0.3, acceleration_m_s2=0.1, auxiliary_current_a=0.2, vcc_v=5)
    result = size_rolling_drive(
        system.channels[0].motor, system.battery, system.driver, state, body, demand
    )
    channels = tuple(
        c.model_copy(
            update={
                "inputs": result.command,
                "output_speed_rad_s": demand.speed_m_s / body.wheel_radius_m,
            }
        )
        for c in system.channels
    )
    runtime = RollingRuntime(
        system.model_copy(update={"channels": channels}),
        state,
        body,
        vcc_v=5,
        auxiliary_current_a=0.2,
    )
    assert runtime.sample.shaft.point == result.point
    assert runtime.sample.acceleration_m_s2 == pytest.approx(0.1)


def test_supplied_vcc_is_forwarded_on_initial_and_subsequent_solves(monkeypatch):
    voltages = []
    evaluate = TB6612Driver.evaluate

    def capture(self, motor, inputs, vm_v, vcc_v, output_speed_rad_s):
        voltages.append(vcc_v)
        return evaluate(self, motor, inputs, vm_v, vcc_v, output_speed_rad_s)

    monkeypatch.setattr(TB6612Driver, "evaluate", capture)
    system, body = setup()
    runtime = RollingRuntime(system, BatteryState(), body, vcc_v=5)
    runtime.step(0.01)
    assert len(voltages) >= 6 and set(voltages) == {5}


def test_auxiliary_depletion_failure_leaves_runtime_state_unchanged():
    system, body = setup(DriverInputs(standby=False))
    system = system.model_copy(
        update={"battery": system.battery.model_copy(update={"capacity_ah": 1e-7})}
    )
    runtime = RollingRuntime(system, BatteryState(soc=0.5), body, auxiliary_current_a=0.2)
    before = runtime.sample, runtime.system
    with pytest.raises(ValueError, match="remaining charge"):
        runtime.step(0.001)
    assert (runtime.sample, runtime.system) == before


def test_constant_supply_auxiliary_energy_and_charge_have_analytic_values():
    system, body = setup(DriverInputs(standby=False))
    battery = system.battery.model_copy(
        update={
            "internal_resistance_ohm": 0,
            "ocv_curve": (OCVPoint(soc=0, voltage_v=8), OCVPoint(soc=1, voltage_v=8)),
        }
    )
    runtime = RollingRuntime(
        system.model_copy(update={"battery": battery}),
        BatteryState(soc=0.5),
        body,
        auxiliary_current_a=0.2,
    )
    energy = 0
    for _ in range(10):
        step = runtime.step(0.01)
        energy += step.start.shaft.point.battery.power_w * 0.01
    assert energy == pytest.approx(8 * 0.2 * 0.1)
    assert runtime.sample.shaft.state.soc == pytest.approx(
        0.5 - 0.2 * 0.1 / (battery.capacity_ah * 3600)
    )


@pytest.mark.parametrize(
    "power",
    [
        {"auxiliary_current_a": -1},
        {"auxiliary_current_a": True},
        {"auxiliary_current_a": float("nan")},
        {"vcc_v": 2.6},
        {"vcc_v": True},
    ],
)
def test_invalid_runtime_power_conditions_are_rejected(power):
    system, body = setup()
    with pytest.raises(ValueError):
        RollingRuntime(system, BatteryState(), body, **power)
