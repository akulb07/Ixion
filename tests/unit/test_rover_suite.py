import json

import pytest

from roboforge.cli import main
from roboforge.hardware.rover_brake import load_rover_scenario
from roboforge.hardware.rover_suite import (
    RoverAcceptance,
    RoverSuiteConfig,
    check_rover_acceptance,
    load_rover_suite,
)
from tests.unit.test_hardware_project import REFERENCE


def suite_data():
    scenario = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml")).model_dump(mode="json")
    scenario["duration_s"] = 0.1
    return {"name": "test_suite", "cases": [{"name": "first", "scenario": scenario}]}


def test_checked_in_suite_loads_with_full_reproducible_scenario_data():
    config = load_rover_suite(REFERENCE.with_name("rover_feedback_suite.yaml"))
    assert len(config.cases) == 4
    assert config.cases[0].scenario.encoder_faults == ()
    assert len(config.cases[2].scenario.encoder_faults) == 2
    assert config == RoverSuiteConfig.model_validate_json(config.model_dump_json())


@pytest.mark.parametrize(
    "change", ["duplicate_names", "empty", "too_many", "steps", "missing_fault"]
)
def test_invalid_suite_bounds_and_evidence_requirements(change):
    data = suite_data()
    case = data["cases"][0]
    if change == "duplicate_names":
        data["cases"] *= 2
    elif change == "empty":
        data["cases"] = []
    elif change == "too_many":
        data["cases"] = [{**case, "name": f"case_{i}"} for i in range(17)]
    elif change == "steps":
        case["scenario"]["duration_s"] = 10
        data["cases"] = [{**case, "name": f"case_{i}"} for i in range(5)]
    else:
        case["acceptance"] = {"maximum_fault_detection_delay_s": 0.2}
    with pytest.raises(ValueError):
        RoverSuiteConfig.model_validate(data)


@pytest.mark.parametrize("value", [-1, float("nan"), True, "0.1"])
def test_acceptance_thresholds_require_finite_nonnegative_numbers(value):
    with pytest.raises(ValueError):
        RoverAcceptance(minimum_front_clearance_m=value)


@pytest.mark.parametrize(
    "content",
    [
        "name: one\nname: two\ncases: []",
        "x" * 131_073,
        "!!python/object/apply:os.system ['nope']",
        "{name: invalid, cases: []}",
    ],
    ids=["duplicate", "oversize", "unsafe_tag", "empty_cases"],
)
def test_bad_suite_file_is_rejected_without_output(tmp_path, content):
    source, output = tmp_path / "suite.yaml", tmp_path / "result.json"
    source.write_text(content)
    assert main(["rover-suite", str(REFERENCE), str(source), "--output", str(output)]) == 2
    assert not output.exists()


def test_missing_requested_measurements_cannot_pass_acceptance():
    report = {
        "status": "stopped_before_wall",
        "brake_reason": "obstacle",
        "metrics": {},
        "encoder_feedback": {"activations": []},
    }
    policy = RoverAcceptance(
        minimum_front_clearance_m=0,
        maximum_peak_pack_current_a=1,
        maximum_fault_detection_delay_s=0.2,
    )
    checks = check_rover_acceptance(report, policy)
    assert checks[0]["passed"]
    assert all(not check["passed"] and check["actual"] is None for check in checks[1:])


def test_acceptance_reports_each_actual_value_and_allows_threshold_equality():
    report = {
        "status": "stopped_before_wall",
        "brake_reason": "encoder_no_progress",
        "metrics": {"final_front_clearance_m": 0.5, "peak_sampled_pack_current_a": 1},
        "encoder_feedback": {"activations": [{"time_s": 0.5}]},
        "speed_control": {"encoder_health": {"fault": {"time_s": 0.75}}},
    }
    policy = RoverAcceptance(
        expected_brake_reason="encoder_no_progress",
        minimum_front_clearance_m=0.5,
        maximum_peak_pack_current_a=1,
        maximum_fault_detection_delay_s=0.25,
    )
    checks = check_rover_acceptance(report, policy)
    assert len(checks) == 5 and all(c["passed"] for c in checks)
    assert checks[-1]["actual"] == 0.25


def test_sonar_stop_is_not_treated_as_encoder_fault_detection():
    report = {
        "status": "stopped_before_wall",
        "brake_reason": "obstacle",
        "metrics": {"detection_time_s": 1},
        "encoder_feedback": {"activations": [{"time_s": 0.5}]},
        "speed_control": {"encoder_health": {"fault": None}},
    }
    assert not check_rover_acceptance(report, RoverAcceptance(maximum_fault_detection_delay_s=1))[
        -1
    ]["passed"]


def test_cli_saves_failed_checks_and_does_not_replace_existing_output(tmp_path, capsys):
    source, output = tmp_path / "suite.json", tmp_path / "result.json"
    source.write_text(json.dumps(suite_data()))
    args = ["rover-suite", str(REFERENCE), str(source), "--output", str(output)]
    assert main(args) == 3
    assert "0/1 cases passed" in capsys.readouterr().out
    payload = output.read_bytes()
    result = json.loads(payload)
    assert result["summary"] == {"total": 1, "passed": 0, "failed": 1, "error": 0}
    assert result["cases"][0]["report"]["status"] == "completed_without_brake"
    assert main(args) == 2
    assert output.read_bytes() == payload
