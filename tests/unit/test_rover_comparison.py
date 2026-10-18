import json
from copy import deepcopy

import pytest

from roboforge.cli import main
from roboforge.hardware import load_project
from roboforge.hardware.rover_comparison import compare_rover_suites
from roboforge.hardware.rover_suite import RoverSuiteConfig, run_rover_suite
from tests.unit.test_hardware_project import REFERENCE
from tests.unit.test_rover_suite import suite_data


@pytest.fixture(scope="module")
def report():
    return run_rover_suite(load_project(REFERENCE), RoverSuiteConfig.model_validate(suite_data()))


def test_identical_suites_have_zero_available_deltas_and_no_design_changes(report):
    result = compare_rover_suites(report, report)
    assert result["design_changes"]["total"] == 0
    assert result["status"] == "failed" and result["gate"] == "candidate_suite_acceptance"
    assert result["sources"]["baseline"]["sha256"] == result["sources"]["candidate"]["sha256"]
    assert result["sources"]["baseline"]["hash_scope"] == "canonical_report_json"
    metrics = result["cases"][0]["metrics"]
    assert (
        metrics["terminal_energy_j"]["unit"] == "J" and metrics["terminal_energy_j"]["delta"] == 0
    )
    assert metrics["stop_time_s"]["delta"] is None
    assert metrics["stop_time_s"]["availability"] == "missing_evidence"


@pytest.mark.parametrize(
    "change",
    [
        "conditions",
        "policy",
        "summary",
        "checks",
        "final_metric",
        "energy_metric",
        "understated_peak",
        "response_metric",
        "interval_energy",
        "project",
        "firmware",
        "missing_case",
        "duplicate_case",
    ],
)
def test_inconsistent_or_mismatched_evidence_cannot_be_compared(report, change):
    candidate = deepcopy(report)
    row = candidate["cases"][0]
    if change == "conditions":
        candidate["config"]["cases"][0]["scenario"]["wall_distance_m"] += 0.1
    elif change == "policy":
        candidate["config"]["cases"][0]["acceptance"]["minimum_front_clearance_m"] = 0.1
    elif change == "summary":
        candidate["summary"]["passed"] += 1
    elif change == "checks":
        row["checks"][0]["passed"] = True
    elif change == "final_metric":
        row["report"]["metrics"]["final_distance_m"] += 1
    elif change == "energy_metric":
        row["report"]["metrics"]["terminal_energy_j"] += 1
    elif change == "understated_peak":
        row["report"]["metrics"]["peak_sampled_pack_current_a"] = 0
    elif change == "response_metric":
        row["report"]["metrics"]["controller_response_delay_s"] = 99
    elif change == "interval_energy":
        row["report"]["samples"][-1]["interval_terminal_energy_j"] = -1
    elif change == "project":
        row["report"]["project"]["assembly"]["total_mass_kg"] += 1
    elif change == "firmware":
        row["report"]["firmware"]["parameters"]["target_speed_m_s"] += 1
    elif change == "missing_case":
        candidate["cases"] = []
    else:
        candidate["cases"] *= 2
    with pytest.raises(ValueError, match="Invalid rover design comparison"):
        compare_rover_suites(report, candidate)


def test_cli_records_input_byte_fingerprints_and_preserves_sources(tmp_path, report):
    baseline, candidate, output = (
        tmp_path / "base.json",
        tmp_path / "next.json",
        tmp_path / "comparison.json",
    )
    baseline.write_text(json.dumps(report))
    candidate.write_text(json.dumps(report, indent=2))
    base_before, next_before = baseline.read_bytes(), candidate.read_bytes()
    args = ["rover-compare", str(baseline), str(candidate), "--output", str(output)]
    assert main(args) == 3
    result = json.loads(output.read_text())
    assert result["sources"]["baseline"]["hash_scope"] == "source_file_bytes"
    assert result["sources"]["baseline"]["sha256"] != result["sources"]["candidate"]["sha256"]
    assert (
        result["sources"]["baseline"]["project_sha256"]
        == result["sources"]["candidate"]["project_sha256"]
    )
    assert baseline.read_bytes() == base_before and candidate.read_bytes() == next_before
    payload = output.read_bytes()
    assert main(args) == 2 and output.read_bytes() == payload


def test_older_level_suites_without_grade_fields_remain_semantically_compatible(report):
    old = deepcopy(report)
    explicit = deepcopy(report)
    for case in old["config"]["cases"]:
        case["scenario"].pop("grade_rad", None)
    for case in old["cases"]:
        if case["report"] is not None:
            case["report"]["config"].pop("grade_rad", None)
            case["report"]["body"].pop("grade_rad", None)
    for case in explicit["config"]["cases"]:
        case["scenario"]["grade_rad"] = 0
    for case in explicit["cases"]:
        if case["report"] is not None:
            case["report"]["config"]["grade_rad"] = 0
            case["report"]["body"]["grade_rad"] = 0
    comparison = compare_rover_suites(old, explicit)
    assert comparison["design_changes"]["total"] == 0
    assert all(
        metric["delta"] in (0, None)
        for case in comparison["cases"]
        for metric in case["metrics"].values()
    )
