import pytest

from roboforge.hardware import load_project
from roboforge.hardware.bench_trace import (
    BenchMetadata,
    BenchSample,
    BenchTrace,
    compare_bench_trace,
)
from roboforge.hardware.rover_suite import load_rover_suite, run_rover_suite
from tests.unit.test_hardware_project import REFERENCE


def test_saved_suite_case_can_be_compared_to_explicit_synthetic_trace_without_rerun():
    config = load_rover_suite(REFERENCE.with_name("rover_feedback_suite.yaml"))
    report = run_rover_suite(
        load_project(REFERENCE), config.model_copy(update={"cases": config.cases[:1]})
    )
    run = report["cases"][0]["report"]
    samples = tuple(
        BenchSample(
            time_s=s["time_s"],
            speed_m_s=s["speed_m_s"] + 0.01,
            pack_voltage_v=s["pack_voltage_v"] - 0.05,
        )
        for s in run["samples"][::50]
    )
    trace = BenchTrace(
        metadata=BenchMetadata(
            source_kind="synthetic",
            reference="generated test",
            instrument="none",
            conditions="fixed biases",
            speed_reference="wheel",
        ),
        samples=samples,
        csv_sha256="1" * 64,
        metadata_sha256="2" * 64,
    )
    result = compare_bench_trace(
        report, trace, time_offset_s=0, maximum_sample_age_s=0, case_name="normal"
    )
    assert result["counts"]["paired_rows"] == len(samples)
    assert result["metrics"]["speed_m_s"]["rmse"] == pytest.approx(0.01)
    assert result["metrics"]["pack_voltage_v"]["mean_error"] == pytest.approx(0.05)
    assert result["source_kind"] == "synthetic"
    with pytest.raises(ValueError, match="missing"):
        compare_bench_trace(
            report, trace, time_offset_s=0, maximum_sample_age_s=0, case_name="missing"
        )
