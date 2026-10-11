import json
import runpy
from pathlib import Path

import pytest


def test_scripted_target_brakes_after_measured_echo(capsys):
    path = Path(__file__).parents[2] / "examples/esp32_diff_drive/ultrasonic_brake.py"
    runpy.run_path(str(path), run_name="__main__")
    report = json.loads(capsys.readouterr().out)
    assert report["readings"][0]["distance_m"] == pytest.approx(0.5)
    near = next(r for r in report["readings"] if r["distance_m"] <= 0.2)
    assert near["distance_m"] == pytest.approx(0.1)
    assert near["time_s"] <= report["brake_command_time_s"] < near["time_s"] + 0.001
    samples = report["samples"]
    first = next(i for i, s in enumerate(samples) if s["driver_mode"] == "brake")
    assert samples[first]["time_s"] > report["brake_command_time_s"]
    assert all(s["driver_mode"] == "brake" for s in samples[first:])
    assert samples[-1]["speed_rad_s"] < samples[first]["speed_rad_s"]
