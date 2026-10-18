import json
import math
from copy import deepcopy

import pytest

from roboforge.cli import main
from roboforge.hardware.bench_trace import (
    BenchMetadata,
    BenchSample,
    BenchTrace,
    compare_bench_trace,
    load_bench_trace,
    select_bench_run,
)
from roboforge.hardware.rover_report import project_rover_evidence
from tests.unit.test_rover_report import recorded


def recording():
    report = recorded()
    initial = report["samples"][0]
    report["samples"] = [deepcopy(initial) for _ in range(3)]
    for index, sample in enumerate(report["samples"]):
        sample.update(
            time_s=index * 0.1, speed_m_s=index, pack_current_a=0.1 * index, distance_m=0.1 * index
        )
    return report


def metadata():
    return {
        "source_kind": "synthetic",
        "reference": "unit fixture",
        "instrument": "none",
        "conditions": "synthetic values",
        "speed_reference": "wheel",
    }


def trace(rows):
    return BenchTrace(
        metadata=BenchMetadata(**metadata()),
        samples=tuple(BenchSample(**row) for row in rows),
        csv_sha256="1" * 64,
        metadata_sha256="2" * 64,
    )


def test_explicit_biases_produce_expected_error_metrics_without_changing_sources():
    run = recording()
    before = deepcopy(run)
    source = trace(
        [
            {
                "time_s": s["time_s"],
                "speed_m_s": s["speed_m_s"] + 0.1,
                "pack_current_a": s["pack_current_a"] - 0.02,
            }
            for s in run["samples"]
        ]
    )
    result = compare_bench_trace(run, source, time_offset_s=0, maximum_sample_age_s=0)
    assert result["status"] == "compared" and result["source_kind"] == "synthetic"
    assert result["counts"]["paired_rows"] == 3
    assert result["alignment"]["error_sign"] == "prediction_minus_measurement"
    assert result["metrics"]["speed_m_s"]["mean_error"] == pytest.approx(-0.1)
    assert result["metrics"]["speed_m_s"]["rmse"] == pytest.approx(0.1)
    assert result["metrics"]["pack_current_a"]["mean_error"] == pytest.approx(0.02)
    assert result["metrics"]["distance_m"]["rmse"] is None
    assert run == before


def test_time_offset_is_required_and_never_automatically_inferred():
    source = trace([{"time_s": 1, "speed_m_s": 0}, {"time_s": 1.1, "speed_m_s": 1}])
    unaligned = compare_bench_trace(recording(), source, time_offset_s=0, maximum_sample_age_s=0)
    assert unaligned["status"] == "no_paired_values" and unaligned["counts"]["after_run"] == 2
    result = compare_bench_trace(recording(), source, time_offset_s=-1, maximum_sample_age_s=0)
    assert result["counts"]["paired_rows"] == 2
    assert result["metrics"]["speed_m_s"]["rmse"] == 0


def test_previous_sample_selection_has_explicit_age_and_excludes_stale_values():
    source = trace(
        [
            {"time_s": -0.1, "speed_m_s": 99},
            {"time_s": 0.05, "speed_m_s": 0},
            {"time_s": 0.16, "speed_m_s": 99},
            {"time_s": 0.3, "speed_m_s": 99},
        ]
    )
    result = compare_bench_trace(recording(), source, time_offset_s=0, maximum_sample_age_s=0.05)
    assert [row["status"] for row in result["rows"]] == [
        "before_run",
        "aligned",
        "stale",
        "after_run",
    ]
    assert result["rows"][1]["sample_time_s"] == 0
    assert result["rows"][1]["sample_age_s"] == 0.05
    assert result["rows"][0]["measurement"]["speed_m_s"] == 99
    assert result["metrics"]["speed_m_s"]["paired_values"] == 1


def test_inadequate_timestamp_precision_cannot_claim_a_match():
    source = trace([{"time_s": 1e308, "speed_m_s": 0}])
    result = compare_bench_trace(recording(), source, time_offset_s=-1e308, maximum_sample_age_s=0)
    assert result["status"] == "no_paired_values"
    assert result["counts"]["insufficient_time_precision"] == 1


def test_boundary_rounding_and_large_finite_rmse_are_handled_without_overflow():
    source = trace(
        [
            {"time_s": math.nextafter(0.1, 0), "speed_m_s": -1e308},
            {"time_s": 0.2, "speed_m_s": -1e308},
        ]
    )
    result = compare_bench_trace(recording(), source, time_offset_s=0, maximum_sample_age_s=0)
    assert result["counts"]["paired_rows"] == 2
    assert result["metrics"]["speed_m_s"]["rmse"] == pytest.approx(1e308)
    assert math.isfinite(result["metrics"]["speed_m_s"]["mean_error"])


@pytest.mark.parametrize(
    "csv_data",
    [
        "time_s,speed_m_s,speed_m_s\n0,1,2\n",
        "time_s,speed_mph\n0,1\n",
        "time_s\n0\n",
        "time_s,speed_m_s\n0,nan\n",
        "time_s,speed_m_s\n0,1\n0,2\n",
        "time_s,speed_m_s\n0,1,2\n",
        "time_s,speed_m_s\n0,\n",
        "time_s,speed_m_s\n,1\n",
    ],
    ids=[
        "duplicate_header",
        "wrong_unit",
        "no_signal",
        "nonfinite",
        "duplicate_time",
        "ragged",
        "all_missing",
        "missing_time",
    ],
)
def test_invalid_csv_data_is_rejected(tmp_path, csv_data):
    source, meta = tmp_path / "trace.csv", tmp_path / "metadata.json"
    source.write_text(csv_data)
    meta.write_text(json.dumps(metadata()))
    with pytest.raises(ValueError, match="Invalid bench trace"):
        load_bench_trace(source, meta)


def test_missing_values_and_input_fingerprints_are_retained(tmp_path):
    source, meta = tmp_path / "trace.csv", tmp_path / "metadata.json"
    source.write_text("time_s,speed_m_s,pack_voltage_v\n0,0,\n0.1,,7.3\n")
    meta.write_text(json.dumps(metadata()))
    loaded = load_bench_trace(source, meta)
    assert loaded.samples[0].speed_m_s == 0 and loaded.samples[0].pack_voltage_v is None
    assert loaded.samples[1].speed_m_s is None
    assert len(loaded.csv_sha256) == len(loaded.metadata_sha256) == 64


@pytest.mark.parametrize(
    "change", [{"source_kind": "unknown"}, {"instrument": " "}, {"reference": ""}]
)
def test_source_metadata_cannot_be_omitted_or_whitespace(change):
    with pytest.raises(ValueError):
        BenchMetadata(**{**metadata(), **change})


def test_cli_saves_no_overlap_result_and_preserves_all_inputs(tmp_path):
    run, source, meta, output = [
        tmp_path / name for name in ("run.json", "trace.csv", "metadata.json", "comparison.json")
    ]
    run.write_text(json.dumps(recording()))
    source.write_text("time_s,speed_m_s\n2,1\n")
    meta.write_text(json.dumps(metadata()))
    before = [path.read_bytes() for path in (run, source, meta)]
    args = [
        "rover-bench",
        str(run),
        str(source),
        str(meta),
        "--time-offset-s",
        "0",
        "--max-sample-age-s",
        "0",
        "--output",
        str(output),
    ]
    assert main(args) == 3
    assert json.loads(output.read_text())["status"] == "no_paired_values"
    payload = output.read_bytes()
    assert main(args) == 2 and output.read_bytes() == payload
    assert [path.read_bytes() for path in (run, source, meta)] == before


def test_ambiguous_comparison_and_malformed_suite_sources_are_validation_errors():
    for source in (
        {"experiment": "rover_design_comparison"},
        {"experiment": "rover_acceptance_suite"},
    ):
        with pytest.raises(ValueError, match="Invalid bench simulation evidence"):
            select_bench_run(source)


@pytest.mark.parametrize("offset,age", [(float("nan"), 0), (0, -1), (True, 0), (0, float("inf"))])
def test_invalid_alignment_parameters_are_rejected(offset, age):
    with pytest.raises(ValueError):
        compare_bench_trace(
            recording(),
            trace([{"time_s": 0, "speed_m_s": 0}]),
            time_offset_s=offset,
            maximum_sample_age_s=age,
        )


def test_bench_viewer_recomputes_errors_from_embedded_rows_and_keeps_origin_visible():
    source = trace([{"time_s": 0, "speed_m_s": 0.1}, {"time_s": 0.1, "speed_m_s": 1.1}])
    comparison = compare_bench_trace(recording(), source, time_offset_s=0, maximum_sample_age_s=0)
    comparison["metrics"]["speed_m_s"]["rmse"] = 999
    comparison["source_kind"] = "measured"
    data = project_rover_evidence(comparison)
    assert data["bench"]["metrics"]["speed_m_s"]["rmse"] == pytest.approx(0.1)
    assert data["bench"]["source_kind"] == "synthetic"
    assert "synthetic" in data["name"]


def test_bench_viewer_rejects_unknown_measurement_fields():
    comparison = compare_bench_trace(
        recording(), trace([{"time_s": 0, "speed_m_s": 0}]), time_offset_s=0, maximum_sample_age_s=0
    )
    comparison["rows"][0]["measurement"]["time_s"] = 100
    with pytest.raises(ValueError, match="unknown signal"):
        project_rover_evidence(comparison)
