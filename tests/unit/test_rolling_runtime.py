import pytest

from roboforge.hardware.battery import BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.rolling import run_rolling_bench
from roboforge.hardware.rolling_runtime import RollingRuntime
from tests.unit.test_rolling import setup


def test_incremental_rolling_matches_fixed_command_bench():
    system, body = setup()
    expected = run_rolling_bench(system, BatteryState(), body, 0.1, 0.01)
    runtime = RollingRuntime(system, BatteryState(), body)
    for index in range(10):
        step = runtime.step(0.01)
        sample = expected.samples[index + 1]
        assert step.end.distance_m == pytest.approx(sample.distance_m)
        assert step.end.shaft.speed_rad_s == pytest.approx(sample.shaft.speed_rad_s)
        assert step.end.shaft.state.soc == pytest.approx(sample.shaft.state.soc)
    assert runtime.time_s == pytest.approx(0.1)


def test_command_change_brakes_both_wheels_and_preserves_distance():
    system, body = setup()
    runtime = RollingRuntime(system, BatteryState(), body)
    for _ in range(10):
        runtime.step(0.01)
    before = runtime.sample
    brake = DriverInputs(standby=True, in1=True, in2=True)
    result = runtime.step(0.01, (brake, brake))
    assert result.start.distance_m == before.distance_m
    assert result.start.shaft.displacement_rad == before.shaft.displacement_rad
    assert result.end.distance_m > before.distance_m
    assert result.end.speed_m_s < before.speed_m_s
    assert all(c.mode == "brake" for c in result.end.shaft.point.channels)
    assert runtime.step(0.01).inputs == (brake, brake)


def test_invalid_asymmetric_command_does_not_change_runtime():
    system, body = setup()
    runtime = RollingRuntime(system, BatteryState(), body)
    before = runtime.sample, runtime.system
    with pytest.raises(ValueError, match="two identical"):
        runtime.step(0.01, (DriverInputs(standby=True, in1=True), DriverInputs()))
    assert (runtime.sample, runtime.system) == before


def test_traction_failure_on_command_change_is_atomic():
    system, body = setup(DriverInputs(standby=False))
    slippery = body.model_copy(update={"friction_coefficient": 0.001})
    runtime = RollingRuntime(system, BatteryState(), slippery)
    before = runtime.sample, runtime.system
    drive = DriverInputs(standby=True, in1=True)
    with pytest.raises(ValueError, match="traction exceeded"):
        runtime.step(0.01, (drive, drive))
    assert (runtime.sample, runtime.system) == before
