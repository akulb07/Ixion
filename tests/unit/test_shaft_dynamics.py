import math

import pytest

from roboforge.hardware.battery import BatteryState, OCVPoint
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.shaft_dynamics import ShaftLoad, run_shaft_bench
from tests.unit.test_powertrain import system


def constant_supply(inputs=None, speed=0):
    model = system(count=1, resistance=0, inputs=inputs, speed=speed)
    return model.model_copy(
        update={
            "battery": model.battery.model_copy(
                update={
                    "ocv_curve": (OCVPoint(soc=0, voltage_v=8), OCVPoint(soc=1, voltage_v=8)),
                }
            )
        }
    )


def bench(model=None, inertia=0.01, torque=0, step=0.01):
    return run_shaft_bench(
        model or constant_supply(),
        BatteryState(),
        (ShaftLoad(inertia_kg_m2=inertia, load_torque_nm=torque),),
        1,
        step,
    )


def test_acceleration_converges_to_analytic_solution():
    # J*w' = K*(V-K*w)/R, R includes the bridge. w(0)=0.
    exact = 80 * (1 - math.exp(-(0.1**2) / (2.5 * 0.01)))
    coarse = bench(step=0.1).samples[-1].speed_rad_s[0]
    fine = bench().samples[-1].speed_rad_s[0]
    assert abs(fine - exact) < abs(coarse - exact) / 5
    assert fine == pytest.approx(exact, rel=0.002)


def test_reverse_symmetry():
    reverse = constant_supply(DriverInputs(standby=True, in2=True))
    forward, backward = bench(), bench(reverse)
    assert backward.samples[-1].speed_rad_s[0] == pytest.approx(-forward.samples[-1].speed_rad_s[0])
    assert backward.samples[-1].state.soc == pytest.approx(forward.samples[-1].state.soc)
    assert backward.samples[-1].displacement_rad[0] == pytest.approx(
        -forward.samples[-1].displacement_rad[0]
    )


def test_displacement_converges_to_analytic_solution():
    rate = 0.1**2 / (2.5 * 0.01)
    exact = 80 * (1 - (1 - math.exp(-rate)) / rate)
    coarse = bench(step=0.1).samples[-1].displacement_rad[0]
    fine = bench().samples[-1].displacement_rad[0]
    assert abs(fine - exact) < abs(coarse - exact) / 5


def test_coasting_constant_speed_integrates_short_final_interval():
    model = constant_supply(DriverInputs(standby=True), speed=20)
    trace = run_shaft_bench(model, BatteryState(), (ShaftLoad(inertia_kg_m2=0.01),), 0.025, 0.01)
    assert trace.samples[0].displacement_rad == (0,)
    assert trace.samples[-1].displacement_rad[0] == pytest.approx(0.5)
    assert trace.samples[-1].speed_rad_s == (20,)


def test_mechanical_energy_error_converges():
    # Euler's finite-step kinetic-energy residual must shrink with timestep.
    def residual(step):
        trace = bench(step=step)
        work = sum(
            a.point.channels[0].motor.output_torque_nm * a.speed_rad_s[0] * (b.time_s - a.time_s)
            for a, b in zip(trace.samples, trace.samples[1:])
        )
        kinetic = 0.5 * 0.01 * trace.samples[-1].speed_rad_s[0] ** 2
        return abs(kinetic - work)

    assert residual(0.01) < residual(0.1) / 5


def test_inertia_and_applied_load_reduce_acceleration():
    normal = bench().samples[-1].speed_rad_s[0]
    assert 0 < bench(inertia=0.02).samples[-1].speed_rad_s[0] < normal
    assert 0 < bench(torque=0.1).samples[-1].speed_rad_s[0] < normal


def test_shared_supply_recovers_as_shafts_accelerate_and_charge_is_accounted():
    model, state = system(), BatteryState()
    loads = (ShaftLoad(inertia_kg_m2=0.01),) * 2
    trace = run_shaft_bench(model, state, loads, 0.025, 0.01)
    assert [s.time_s for s in trace.samples] == [0, 0.01, 0.02, 0.025]
    assert trace.samples[-1].point.battery.current_a < trace.samples[0].point.battery.current_a
    assert (
        trace.samples[-1].point.battery.terminal_voltage_v
        > trace.samples[0].point.battery.terminal_voltage_v
    )
    used = sum(
        a.point.battery.current_a * (b.time_s - a.time_s) / 3600
        for a, b in zip(trace.samples, trace.samples[1:])
    )
    assert trace.samples[-1].state.soc == pytest.approx(1 - used / model.battery.capacity_ah)
    assert trace == run_shaft_bench(model, state, loads, 0.025, 0.01)
    assert state.soc == 1
    assert all(c.output_speed_rad_s == 0 for c in model.channels)


def test_brake_dissipates_rotational_energy_without_pack_draw():
    trace = bench(constant_supply(DriverInputs(standby=True, in1=True, in2=True), speed=20))
    assert 0 < trace.samples[-1].speed_rad_s[0] < 20
    assert all(s.point.battery.current_a == 0 for s in trace.samples)


@pytest.mark.parametrize(
    "duration,step", [(0, 1), (1, 0), (1, -1), (10001, 1), (True, 1), (1, float("nan"))]
)
def test_invalid_time_budget(duration, step):
    with pytest.raises(ValueError):
        run_shaft_bench(
            constant_supply(), BatteryState(), (ShaftLoad(inertia_kg_m2=1),), duration, step
        )


def test_load_count_and_nonpositive_inertia_rejected():
    with pytest.raises(ValueError):
        run_shaft_bench(constant_supply(), BatteryState(), (), 1, 0.1)
    with pytest.raises(ValueError):
        ShaftLoad(inertia_kg_m2=0)


def test_coarse_step_cannot_silently_continue_into_unsupported_regeneration():
    with pytest.raises(ValueError, match="regenerative"):
        run_shaft_bench(constant_supply(), BatteryState(), (ShaftLoad(inertia_kg_m2=0.0001),), 1, 1)
