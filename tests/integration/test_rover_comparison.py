from copy import deepcopy

import pytest

from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.rover_comparison import compare_rover_suites
from roboforge.hardware.rover_report import project_rover_evidence, render_rover_evidence
from roboforge.hardware.rover_suite import RoverSuiteConfig, load_rover_suite, run_rover_suite
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def setup():
    model = load_project(REFERENCE)
    source = load_rover_suite(REFERENCE.with_name("rover_feedback_suite.yaml"))
    config = source.model_copy(update={"cases": source.cases[:2]})
    data = model.model_dump()
    data["assembly"]["total_mass_kg"] += 1
    heavier = RobotProject.model_validate(data)
    return model, heavier, config


@pytest.fixture(scope="module")
def reports(setup):
    baseline, candidate, config = setup
    return run_rover_suite(baseline, config), run_rover_suite(candidate, config)


def test_added_mass_changes_measured_response_under_identical_conditions(reports):
    baseline, candidate = reports
    result = compare_rover_suites(baseline, candidate)
    assert result["status"] == "passed"
    assert result["design_changes"]["total"] == 1
    [change] = result["design_changes"]["changes"]
    assert change["path"] == "/assembly/total_mass_kg"
    assert change["candidate"] - change["baseline"] == 1
    assert (
        result["sources"]["baseline"]["project_sha256"]
        != result["sources"]["candidate"]["project_sha256"]
    )
    metrics = result["cases"][0]["metrics"]
    assert metrics["peak_sampled_pack_current_a"]["delta"] != 0
    for entry in metrics.values():
        if entry["availability"] == "available":
            assert entry["delta"] == entry["candidate"] - entry["baseline"]


def test_saved_case_order_can_change_without_changing_comparison(reports):
    baseline, candidate = reports
    reordered = deepcopy(candidate)
    reordered["cases"].reverse()
    reordered["config"]["cases"].reverse()
    result = compare_rover_suites(baseline, reordered)
    assert [case["name"] for case in result["cases"]] == [
        case["name"] for case in baseline["cases"]
    ]


def test_model_errors_keep_missing_deltas_instead_of_inventing_measurements(setup, reports):
    model, _, config = setup
    data = model.model_dump()
    for component in data["components"]:
        if component["id"] in ("left_motor", "right_motor"):
            component["parameters"] = tuple(
                p for p in component["parameters"] if p["name"] != "stall_current"
            )
    # Missing a required motor rating is a valid graph but cannot run this model.
    candidate = run_rover_suite(RobotProject.model_validate(data), config)
    if candidate["summary"]["error"] == 0:
        pytest.fail("fixture must remove a required motor rating")
    comparison = compare_rover_suites(reports[0], candidate)
    assert comparison["status"] == "failed"
    assert all(
        metric["candidate"] is None and metric["delta"] is None
        for metric in comparison["cases"][0]["metrics"].values()
    )


def test_changed_acceptance_policy_is_rejected_even_when_both_runs_are_valid(setup, reports):
    model, _, config = setup
    data = config.model_dump()
    data["cases"][0]["acceptance"]["minimum_front_clearance_m"] = 0.1
    candidate = run_rover_suite(model, RoverSuiteConfig.model_validate(data))
    with pytest.raises(ValueError, match="different case conditions"):
        compare_rover_suites(reports[0], candidate)


def test_comparison_viewer_pairs_runs_and_recomputes_edited_deltas(reports):
    comparison = compare_rover_suites(*reports)
    comparison["cases"][0]["metrics"]["terminal_energy_j"]["delta"] = 123456
    data = project_rover_evidence(comparison, "comparison_file_digest")
    assert data["source_sha256"] == "comparison_file_digest"
    assert len(data["cases"]) == 4
    assert data["cases"][0]["design"] == "baseline" and data["cases"][0]["peer_index"] == 1
    assert data["cases"][1]["design"] == "candidate" and data["cases"][1]["peer_index"] == 0
    metric = data["comparison"]["cases"][0]["metrics"]["terminal_energy_j"]
    assert metric["delta"] != 123456 and metric["delta"] == metric["candidate"] - metric["baseline"]
    html = render_rover_evidence(comparison)
    assert 'id="comparison-metrics"' in html and '"peer_index": 1' in html
