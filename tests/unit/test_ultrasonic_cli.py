import json

import pytest

from roboforge.cli import main
from roboforge.hardware.ultrasonic_feedback import load_ultrasonic_scenario
from tests.unit.test_hardware_project import REFERENCE


@pytest.mark.parametrize(
    "scenario,reason", [("obstacle.yaml", "obstacle"), ("missing_echo.yaml", "echo_timeout")]
)
def test_cli_runs_reference_scenarios_and_preserves_existing_report(tmp_path, scenario, reason):
    output = tmp_path / "report.json"
    args = [
        "ultrasonic-feedback",
        str(REFERENCE),
        str(REFERENCE.with_name(scenario)),
        "--output",
        str(output),
    ]
    assert main(args) == 0
    payload = output.read_bytes()
    report = json.loads(payload)
    assert report["brake_reason"] == reason
    assert report["config"]["targets"][0]["distance_m"] == 0.5
    assert "project" in report and "diagnostics" in report
    assert main(args) == 2
    assert output.read_bytes() == payload


@pytest.mark.parametrize(
    "content",
    [
        "inertia_kg_m2: 1\ninertia_kg_m2: 2",
        "unknown: 1",
        "x" * 65537,
        "!!python/object/apply:os.system ['echo nope']",
    ],
    ids=["duplicate-key", "unknown-field", "oversized", "unsafe-tag"],
)
def test_invalid_scenario_is_rejected_without_creating_output(tmp_path, content):
    source = tmp_path / "scenario.yaml"
    source.write_text(content)
    output = tmp_path / "report.json"
    assert main(["ultrasonic-feedback", str(REFERENCE), str(source), "--output", str(output)]) == 2
    assert not output.exists()


def test_missing_input_is_reported_as_validation_error(tmp_path):
    with pytest.raises(ValueError, match="Invalid ultrasonic scenario"):
        load_ultrasonic_scenario(tmp_path / "absent.yaml")


def test_continue_policy_saves_report_with_non_trigger_exit_code(tmp_path):
    source = tmp_path / "scenario.json"
    source.write_text(
        json.dumps(
            {
                "inertia_kg_m2": 0.01,
                "sensor": {"response_delay_s": 0.0002},
                "targets": [{"time_s": 0, "distance_m": None}],
                "timeout_action": "continue",
                "duration_s": 0.1,
            }
        )
    )
    output = tmp_path / "report.json"
    assert main(["ultrasonic-feedback", str(REFERENCE), str(source), "--output", str(output)]) == 3
    assert json.loads(output.read_text())["status"] == "completed_without_brake"
