import math

import pytest

from roboforge.hardware.battery import BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.rolling import RollingBody, run_rolling_bench
from tests.unit.test_shaft_dynamics import constant_supply


def setup(inputs=None):
    motor = constant_supply(inputs).channels[0]
    system = constant_supply().model_copy(update={"channels": (motor, motor)})
    body = RollingBody(
        total_mass_kg=2,
        wheel_radius_m=0.1,
        rotating_inertia_per_wheel_kg_m2=0.002,
        friction_coefficient=1,
        driven_weight_fraction=0.8,
    )
    return system, body


def test_motor_torque_accelerates_mass_and_rotating_parts():
    system, body = setup()
    trace = run_rolling_bench(system, BatteryState(), body, 1, 0.01)
    for sample in trace.samples:
        torque = sample.shaft.point.channels[0].motor.output_torque_nm
        alpha = sample.shaft.acceleration_rad_s2[0]
        force = sample.contact_force_per_wheel_n
        assert torque == pytest.approx(
            body.rotating_inertia_per_wheel_kg_m2 * alpha + force * body.wheel_radius_m
        )
        assert 2 * force == pytest.approx(body.total_mass_kg * sample.acceleration_m_s2)
    assert trace.samples[-1].distance_m > 0
    assert trace.samples[-1].shaft.state.soc < 1


def test_speed_converges_to_constant_supply_analytic_solution():
    system, body = setup()
    inertia = 0.002 + 2 * 0.1**2 / 2
    exact = 8 * (1 - math.exp(-(0.1**2) / (2.5 * inertia)))
    coarse = run_rolling_bench(system, BatteryState(), body, 1, 0.1).samples[-1].speed_m_s
    fine = run_rolling_bench(system, BatteryState(), body, 1, 0.01).samples[-1].speed_m_s
    assert abs(fine - exact) < abs(coarse - exact) / 5


def test_heavier_vehicle_accelerates_more_slowly():
    system, body = setup()
    heavy = body.model_copy(update={"total_mass_kg": 4})
    normal = run_rolling_bench(system, BatteryState(), body, 1, 0.01)
    loaded = run_rolling_bench(system, BatteryState(), heavy, 1, 0.01)
    assert loaded.samples[-1].speed_m_s < normal.samples[-1].speed_m_s


def test_reverse_travel_preserves_force_sign():
    system, body = setup(DriverInputs(standby=True, in2=True))
    last = run_rolling_bench(system, BatteryState(), body, 1, 0.01).samples[-1]
    assert last.speed_m_s < 0
    assert last.distance_m < 0
    assert last.contact_force_per_wheel_n < 0


def test_low_traction_aborts_instead_of_fabricating_no_slip_motion():
    system, body = setup()
    slippery = body.model_copy(update={"friction_coefficient": 0.01})
    with pytest.raises(ValueError, match="traction exceeded at 0 s"):
        run_rolling_bench(system, BatteryState(), slippery, 1, 0.01)


def test_turning_and_single_motor_rejected():
    system, body = setup()
    with pytest.raises(ValueError, match="two identical"):
        run_rolling_bench(constant_supply(), BatteryState(), body, 1, 0.01)
    unequal = system.model_copy(
        update={"channels": (system.channels[0], constant_supply(speed=1).channels[0])}
    )
    with pytest.raises(ValueError, match="two identical"):
        run_rolling_bench(unequal, BatteryState(), body, 1, 0.01)


def test_no_weight_on_drive_wheels_rejected():
    with pytest.raises(ValueError):
        RollingBody(
            total_mass_kg=2,
            wheel_radius_m=0.1,
            rotating_inertia_per_wheel_kg_m2=0,
            friction_coefficient=1,
            driven_weight_fraction=0,
        )
