import math

import pytest

from roboforge.hardware.battery import BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.rolling import RollingBody, run_rolling_bench
from roboforge.hardware.rolling_runtime import RollingRuntime
from tests.unit.test_rolling import setup


@pytest.mark.parametrize("grade", [-0.1, 0.1])
def test_grade_obeys_translational_and_wheel_torque_balance(grade):
    system, original = setup()
    body = RollingBody.model_validate({**original.model_dump(), "grade_rad": grade})
    trace = run_rolling_bench(system, BatteryState(), body, 0.2, 0.001)
    for sample in trace.samples:
        force = sample.contact_force_per_wheel_n
        acceleration = sample.acceleration_m_s2
        gravity = body.total_mass_kg * body.gravity_m_s2 * math.sin(grade)
        assert 2 * force - gravity == pytest.approx(body.total_mass_kg * acceleration)
        torque = sample.shaft.point.channels[0].motor.output_torque_nm
        assert torque == pytest.approx(
            body.rotating_inertia_per_wheel_kg_m2 * sample.shaft.acceleration_rad_s2[0]
            + force * body.wheel_radius_m
        )
        assert sample.traction_limit_per_wheel_n == pytest.approx(
            body.friction_coefficient
            * body.total_mass_kg
            * body.gravity_m_s2
            * math.cos(grade)
            * body.driven_weight_fraction
            / 2
        )


@pytest.mark.parametrize("grade", [-0.1, 0.1])
def test_unpowered_grade_matches_constant_gravity_acceleration_and_releases_potential_energy(grade):
    system, original = setup(DriverInputs(standby=False))
    body = original.model_copy(update={"grade_rad": grade})
    mass = body.total_mass_kg
    effective_mass = mass + 2 * body.rotating_inertia_per_wheel_kg_m2 / body.wheel_radius_m**2
    acceleration = -mass * body.gravity_m_s2 * math.sin(grade) / effective_mass
    errors = []
    for step in (0.01, 0.001):
        end = run_rolling_bench(system, BatteryState(), body, 0.2, step).samples[-1]
        assert end.speed_m_s == pytest.approx(acceleration * 0.2)
        assert end.acceleration_m_s2 == pytest.approx(acceleration)
        kinetic = 0.5 * effective_mass * end.speed_m_s**2
        potential = mass * body.gravity_m_s2 * math.sin(grade) * end.distance_m
        errors.append(abs(kinetic + potential))
        assert end.shaft.state.soc == 1
    assert errors[1] < errors[0] / 5


def test_uphill_reduces_speed_and_increases_current_after_the_same_command_duration():
    system, body = setup()
    level = run_rolling_bench(system, BatteryState(), body, 0.5, 0.001)
    hill = run_rolling_bench(
        system, BatteryState(), body.model_copy(update={"grade_rad": 0.1}), 0.5, 0.001
    )
    assert hill.samples[0].shaft.point == level.samples[0].shaft.point
    assert hill.samples[-1].speed_m_s < level.samples[-1].speed_m_s
    assert (
        hill.samples[-1].shaft.point.battery.current_a
        > level.samples[-1].shaft.point.battery.current_a
    )


def test_incremental_grade_runtime_matches_bench_including_initial_acceleration():
    system, body = setup()
    body = body.model_copy(update={"grade_rad": 0.1})
    expected = run_rolling_bench(system, BatteryState(), body, 0.1, 0.01)
    runtime = RollingRuntime(system, BatteryState(), body)
    assert runtime.sample == expected.samples[0]
    for index in range(10):
        result = runtime.step(0.01)
        end = expected.samples[index + 1]
        assert result.end.distance_m == pytest.approx(end.distance_m)
        assert result.end.speed_m_s == pytest.approx(end.speed_m_s)
        assert result.end.acceleration_m_s2 == pytest.approx(end.acceleration_m_s2)
        assert result.end.shaft.state.soc == pytest.approx(end.shaft.state.soc)


def test_insufficient_grade_traction_is_rejected_instead_of_clamped():
    system, body = setup(DriverInputs(standby=False))
    body = body.model_copy(update={"grade_rad": 0.4, "friction_coefficient": 0.001})
    with pytest.raises(ValueError, match="traction exceeded at 0 s"):
        run_rolling_bench(system, BatteryState(), body, 0.1, 0.001)


@pytest.mark.parametrize("grade", [-math.pi / 2, math.pi / 2, float("nan"), float("inf"), True])
def test_vertical_nonfinite_and_boolean_grades_are_rejected(grade):
    _, body = setup()
    with pytest.raises(ValueError):
        RollingBody.model_validate({**body.model_dump(), "grade_rad": grade})
