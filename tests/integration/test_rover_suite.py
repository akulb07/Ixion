import pytest

from roboforge.hardware import load_project
from roboforge.hardware.rover_suite import RoverSuiteConfig, load_rover_suite, run_rover_suite
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def config():
    return load_rover_suite(REFERENCE.with_name("rover_feedback_suite.yaml"))


def test_reference_suite_passes_all_checks_and_keeps_full_case_evidence(model, config):
    result = run_rover_suite(model, config)
    assert result["status"] == "passed"
    assert result["summary"] == {"total": 4, "passed": 4, "failed": 0, "error": 0}
    assert result["config"] == config.model_dump(mode="json")
    assert result["project"] == model.model_dump(mode="json")
    assert [row["name"] for row in result["cases"]] == [case.name for case in config.cases]
    for row in result["cases"]:
        assert all(c["passed"] for c in row["checks"])
        assert row["error"] is None
        assert row["report"]["samples"][0]["distance_m"] == 0
        assert row["report"]["samples"][0]["soc"] == 1
        assert row["report"]["speed_control"]["samples"][0]["time_s"] == 0


def test_a_model_error_is_recorded_and_does_not_stop_later_cases(model, config):
    first = config.cases[0].model_dump()
    first["scenario"].update(friction_coefficient=0.001)
    second = config.cases[1].model_dump()
    data = {**config.model_dump(), "cases": [first, second]}
    result = run_rover_suite(model, RoverSuiteConfig.model_validate(data))
    assert result["status"] == "failed"
    assert result["summary"] == {"total": 2, "passed": 1, "failed": 0, "error": 1}
    assert result["cases"][0]["report"] is None and result["cases"][0]["error"]
    assert result["cases"][1]["status"] == "passed"


def test_case_order_does_not_leak_controller_or_physics_state(model, config):
    cases = config.cases[:2]
    forward = run_rover_suite(model, config.model_copy(update={"cases": cases}))
    backward = run_rover_suite(model, config.model_copy(update={"cases": tuple(reversed(cases))}))
    assert forward["cases"] == list(reversed(backward["cases"]))


def test_sonar_only_cases_and_expected_contact_are_explicit(model, config):
    data = config.cases[0].model_dump()
    data.update(controller="sonar", acceptance={"expected_status": "wall_contact"})
    data["scenario"]["brake_threshold_m"] = 0.01
    suite = RoverSuiteConfig(name="contact", cases=(data,))
    result = run_rover_suite(model, suite)
    assert result["status"] == "passed"
    assert "speed_control" not in result["cases"][0]["report"]
    assert result["cases"][0]["report"]["metrics"]["wall_contact_time_s"] is not None
