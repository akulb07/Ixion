import json
import runpy
from pathlib import Path

import pytest

from roboforge.hardware.ultrasonic import UltrasonicConfig
from roboforge.hardware.ultrasonic_feedback import (
    TargetDistance,
    UltrasonicFeedbackConfig,
    run_ultrasonic_feedback,
)
from tests.unit.test_board_inputs import project


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


def scenario(**options):
    return UltrasonicFeedbackConfig(
        inertia_kg_m2=0.01,
        sensor=UltrasonicConfig(response_delay_s=0.0002),
        targets=(TargetDistance(time_s=0, distance_m=None),),
        **options,
    )


def test_missing_echo_brakes_after_deadline_without_inventing_distance():
    report = run_ultrasonic_feedback(project(), scenario(duration_s=0.1))
    assert report["status"] == "brake_triggered"
    assert report["brake_reason"] == "echo_timeout"
    assert report["detection_time_s"] == pytest.approx(0.03001)
    assert report["brake_command_time_s"] == pytest.approx(0.031)
    assert all(r["distance_m"] is None and r["status"] == "timeout" for r in report["readings"])


def test_continue_policy_records_timeouts_without_braking():
    report = run_ultrasonic_feedback(project(), scenario(duration_s=0.1, timeout_action="continue"))
    assert report["status"] == "completed_without_brake"
    assert report["brake_reason"] is None
    assert len(report["readings"]) == 2
    assert all(s["driver_mode"] == "forward" for s in report["samples"])


def test_end_before_next_control_interval_reports_pending_brake():
    report = run_ultrasonic_feedback(project(), scenario(duration_s=0.0305))
    assert report["status"] == "brake_pending"
    assert report["brake_command_time_s"] is None
    assert report["samples"][-1]["time_s"] == 0.0305


def test_invalid_poll_period_and_fixture_order_rejected():
    with pytest.raises(ValueError, match="poll period"):
        scenario(poll_every_steps=10)
    with pytest.raises(ValueError, match="start at zero"):
        UltrasonicFeedbackConfig(
            inertia_kg_m2=0.01,
            sensor=UltrasonicConfig(response_delay_s=0),
            targets=(TargetDistance(time_s=1, distance_m=1),),
        )


def test_reusable_runner_supports_right_motor_and_is_deterministic():
    settings = scenario(motor="right_motor", duration_s=0.1)
    report = run_ultrasonic_feedback(project(), settings)
    assert report == run_ultrasonic_feedback(project(), settings)
    assert report["motor_connection"]["channel"] == "B"
    assert report["status"] == "brake_triggered"
