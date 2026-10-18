import json
from copy import deepcopy
from xml.etree import ElementTree as ET

import pytest

from roboforge.ci_reports import bench_acceptance_junit
from roboforge.cli import main
from roboforge.hardware.bench_acceptance import (
    BenchAcceptancePolicy,
    check_bench_comparison,
    load_bench_policy,
)
from roboforge.hardware.bench_trace import compare_bench_trace
from roboforge.hardware.rover_report import project_rover_evidence
from tests.unit.test_bench_trace import recording, trace


def policy_data():
    return {
        "name": "synthetic fixture only",
        "reference": "declared test errors, not hardware tolerances",
        "source_kind": "synthetic",
        "speed_reference": "wheel",
        "maximum_alignment_age_s": 0,
        "signals": {
            "speed_m_s": {
                "minimum_pairs": 3,
                "minimum_paired_fraction": 1,
                "minimum_paired_span_s": 0.2,
                "maximum_paired_gap_s": 0.11,
                "maximum_rmse": 0.11,
                "maximum_absolute_error": 0.11,
                "maximum_absolute_mean_error": 0.11,
            }
        },
    }


def comparison(rows=None, age=0):
    if rows is None:
        rows = [{"time_s": i * 0.1, "speed_m_s": i + 0.1} for i in range(3)]
    return compare_bench_trace(recording(), trace(rows), time_offset_s=0, maximum_sample_age_s=age)


def check(source=None, policy=None):
    return check_bench_comparison(
        comparison() if source is None else source,
        BenchAcceptancePolicy.model_validate(policy_data() if policy is None else policy),
    )


def test_bias_limits_pass_with_exact_coverage_without_mutating_input():
    source = comparison()
    before = deepcopy(source)
    result = check(source)
    assert result["status"] == "pass"
    assert all(c["status"] == "pass" for c in result["checks"])
    assert result["coverage"]["speed_m_s"] == {
        "supplied_values": 3,
        "paired_values": 3,
        "paired_fraction": 1,
        "paired_span_s": 0.2,
        "maximum_paired_gap_s": 0.1,
    }
    assert source == before


def test_measured_policy_rejects_synthetic_despite_forged_derived_fields():
    source = comparison()
    source["source_kind"] = "measured"
    source["metrics"]["speed_m_s"]["rmse"] = 0
    policy = policy_data()
    policy["source_kind"] = "measured"
    policy["signals"]["speed_m_s"]["maximum_rmse"] = 0.01
    result = check(source, policy)
    failures = {c["name"] for c in result["checks"] if c["status"] == "fail"}
    assert result["source_kind"] == "synthetic"
    assert {"declared_source_kind", "speed_m_s.maximum_rmse"} <= failures


def test_coverage_denominator_counts_supplied_values_and_rejected_rows():
    source = comparison(
        [
            {"time_s": 0, "speed_m_s": 0},
            {"time_s": 0.1, "distance_m": 0.1},
            {"time_s": 0.2, "speed_m_s": 2},
            {"time_s": 0.3, "speed_m_s": 3},
        ]
    )
    result = check(source)
    assert result["coverage"]["speed_m_s"]["paired_fraction"] == pytest.approx(2 / 3)
    assert result["coverage"]["speed_m_s"]["supplied_values"] == 3
    assert result["status"] == "fail"


def test_missing_signal_cannot_become_zero_error_or_success():
    source = comparison([{"time_s": 0, "pack_voltage_v": 7.4}])
    result = check(source)
    assert result["status"] == "fail"
    assert result["coverage"]["speed_m_s"]["paired_fraction"] is None
    rmse = next(c for c in result["checks"] if c["name"] == "speed_m_s.maximum_rmse")
    assert rmse["actual"] is None and rmse["status"] == "inconclusive"
    root = ET.fromstring(bench_acceptance_junit(result))
    assert int(root.get("errors")) > 0 and int(root.get("failures")) > 0


def test_gap_requires_two_samples_even_if_other_declared_limits_accept_one():
    policy = policy_data()
    policy["signals"]["speed_m_s"].update(minimum_pairs=1, minimum_paired_span_s=0)
    result = check(comparison([{"time_s": 0, "speed_m_s": 0}]), policy)
    assert result["status"] == "inconclusive"
    gap = next(c for c in result["checks"] if c["name"] == "speed_m_s.maximum_paired_gap")
    assert gap["status"] == "inconclusive"


def test_alignment_age_speed_reference_and_time_gap_are_independent_gates():
    source = comparison(age=0.1)
    source["metadata"]["speed_reference"] = "ground"
    policy = policy_data()
    policy["signals"]["speed_m_s"]["maximum_paired_gap_s"] = 0.05
    result = check(source, policy)
    failures = {c["name"] for c in result["checks"] if c["status"] == "fail"}
    assert {
        "alignment_sample_age_limit",
        "speed_reference",
        "speed_m_s.maximum_paired_gap",
    } <= failures


@pytest.mark.parametrize(
    "change",
    [
        {"signals": {}},
        {"name": " "},
        {"reference": " "},
        {"speed_reference": None},
        {"source_kind": "unknown"},
        {"maximum_alignment_age_s": True},
        {"maximum_alignment_age_s": float("inf")},
    ],
)
def test_policy_requires_meaningful_explicit_finite_context(change):
    with pytest.raises(ValueError):
        BenchAcceptancePolicy.model_validate({**policy_data(), **change})


@pytest.mark.parametrize(
    "change",
    [
        {"minimum_pairs": True},
        {"minimum_pairs": 0},
        {"minimum_paired_fraction": 1.1},
        {"minimum_paired_span_s": -1},
        {"maximum_rmse": -1},
        {"unknown": 1},
        {"maximum_rmse": None, "maximum_absolute_error": None, "maximum_absolute_mean_error": None},
    ],
)
def test_bad_or_missing_limits_are_rejected(change):
    policy = policy_data()
    policy["signals"]["speed_m_s"].update(change)
    with pytest.raises(ValueError):
        BenchAcceptancePolicy.model_validate(policy)


def test_policy_loader_rejects_duplicate_json_keys_and_size_limit(tmp_path):
    path = tmp_path / "policy.json"
    path.write_text('{"name":"a","name":"b"}')
    with pytest.raises(ValueError, match="duplicate"):
        load_bench_policy(path)
    path.write_text(" " * 65_537)
    with pytest.raises(ValueError, match="limit"):
        load_bench_policy(path)


def test_junit_and_viewer_recompute_acceptance_instead_of_trusting_edited_checks():
    policy = policy_data()
    policy["signals"]["speed_m_s"]["maximum_rmse"] = 0.01
    result = check(policy=policy)
    assert result["status"] == "fail"
    result["status"] = "pass"
    result["checks"] = []
    result["coverage"] = {}
    result["policy_sha256"] = "forged"
    root = ET.fromstring(bench_acceptance_junit(result))
    assert root.get("failures") == "1"
    view = project_rover_evidence(result)
    assert view["bench_acceptance"]["status"] == "fail"
    assert view["bench_acceptance"]["coverage"]["speed_m_s"]["paired_values"] == 3
    assert view["bench_acceptance"]["policy_sha256"] != "forged"


def test_cli_saves_failed_evidence_junit_and_source_hashes_without_overwrite(tmp_path):
    source, policy, output, junit = [
        tmp_path / name for name in ("source.json", "policy.json", "check.json", "check.xml")
    ]
    source.write_text(json.dumps(comparison()))
    data = policy_data()
    data["source_kind"] = "measured"
    policy.write_text(json.dumps(data))
    before = [p.read_bytes() for p in (source, policy)]
    args = [
        "rover-bench-check",
        str(source),
        str(policy),
        "--output",
        str(output),
        "--junit",
        str(junit),
    ]
    assert main(args) == 3
    result = json.loads(output.read_text())
    assert result["status"] == "fail" and len(result["sources"]["comparison_sha256"]) == 64
    assert ET.parse(junit).getroot().get("failures") == "1"
    saved = output.read_bytes(), junit.read_bytes()
    assert main(args) == 2 and saved == (output.read_bytes(), junit.read_bytes())
    assert before == [p.read_bytes() for p in (source, policy)]


def test_cli_rejects_colliding_or_protected_outputs_before_writing(tmp_path):
    source = tmp_path / "source.json"
    source.write_text(json.dumps(comparison()))
    original = source.read_bytes()
    output = tmp_path / "new.json"
    assert (
        main(
            [
                "rover-bench-check",
                str(source),
                "missing.json",
                "--output",
                str(output),
                "--junit",
                str(output),
            ]
        )
        == 2
    )
    assert not output.exists()
    assert main(["rover-bench-check", str(source), "missing.json", "--output", str(source)]) == 2
    assert source.read_bytes() == original
