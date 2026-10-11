import json
import runpy
from pathlib import Path

import pytest

from roboforge.hardware.battery import BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.shaft_dynamics import ShaftLoad, run_shaft_bench
from roboforge.hardware.shaft_runtime import ShaftRuntime
from tests.unit.test_shaft_dynamics import constant_supply


def runtime():
    return ShaftRuntime(constant_supply(), BatteryState(), (ShaftLoad(inertia_kg_m2=0.01),))


def test_incremental_steps_agree_with_fixed_command_batch():
    live = runtime()
    for _ in range(10):
        result = live.step(0.01)
    batch = run_shaft_bench(
        constant_supply(), BatteryState(), (ShaftLoad(inertia_kg_m2=0.01),), 0.1, 0.01
    )
    assert result.end.speed_rad_s == pytest.approx(batch.samples[-1].speed_rad_s)
    assert result.end.displacement_rad == pytest.approx(batch.samples[-1].displacement_rad)
    assert live.battery_state.soc == pytest.approx(batch.samples[-1].state.soc)


def test_command_change_brakes_existing_motion_without_resetting_position():
    live = runtime()
    driven = live.step(0.1)
    braking = live.step(0.1, (DriverInputs(standby=True, in1=True, in2=True),))
    assert braking.start.displacement_rad == driven.end.displacement_rad
    assert 0 < braking.end.speed_rad_s[0] < driven.end.speed_rad_s[0]
    assert braking.start.point.battery.current_a == 0
    assert braking.end.displacement_rad[0] > driven.end.displacement_rad[0]
    assert live.step(0.01).start.point.channels[0].mode == "brake"


def test_failed_step_does_not_consume_time_charge_or_commands():
    live = runtime()
    original = live.system
    with pytest.raises(ValueError, match="regenerative"):
        live.step(100)
    assert live.time_s == 0
    assert live.battery_state == BatteryState()
    assert live.system == original
    assert live.step(0.01) == runtime().step(0.01)


def test_invalid_step_and_command_count():
    live = runtime()
    for duration in (0, -1, True, float("nan")):
        with pytest.raises(ValueError):
            live.step(duration)
    with pytest.raises(ValueError, match="one driver command"):
        live.step(0.01, ())


def test_encoder_feedback_example_brakes_after_observed_count(capsys):
    path = Path(__file__).parents[2] / "examples/esp32_diff_drive/encoder_brake.py"
    runpy.run_path(str(path), run_name="__main__")
    report = json.loads(capsys.readouterr().out)
    samples = report["samples"]
    index = next(i for i, s in enumerate(samples) if s["driver_mode"] == "brake")
    assert index > 1
    assert samples[index - 1]["encoder_count"] >= report["target_count"]
    assert samples[index - 2]["encoder_count"] < report["target_count"]
    assert report["brake_command_time_s"] == samples[index - 1]["time_s"]
    assert all(s["driver_mode"] == "brake" for s in samples[index:])
    assert samples[-1]["speed_rad_s"] < samples[index]["speed_rad_s"]
