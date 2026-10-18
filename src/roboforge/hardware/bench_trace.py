"""Explicit time alignment of measured or synthetic traces with saved rover data."""

import bisect
import csv
import hashlib
import io
import math
from pathlib import Path
from typing import Literal

from pydantic import Field, model_validator

from roboforge.config import Real, Schema
from roboforge.core import finite, nonnegative

from .artifact_io import decode_json_object
from .rover_report import project_rover_evidence

SIGNALS = {"speed_m_s": "m/s", "distance_m": "m", "pack_voltage_v": "V", "pack_current_a": "A"}


class BenchMetadata(Schema):
    schema_version: Literal[1] = 1
    source_kind: Literal["measured", "synthetic"]
    reference: str = Field(min_length=1, max_length=2000)
    instrument: str = Field(min_length=1, max_length=2000)
    conditions: str = Field(min_length=1, max_length=4000)
    captured_at: str | None = Field(default=None, max_length=200)
    speed_reference: Literal["wheel", "ground", "unspecified"] = "unspecified"

    @model_validator(mode="after")
    def meaningful_labels(self):
        if any(not getattr(self, key).strip() for key in ("reference", "instrument", "conditions")):
            raise ValueError("bench source labels must contain non-whitespace text")
        return self


class BenchSample(Schema):
    time_s: Real
    speed_m_s: Real | None = None
    distance_m: Real | None = None
    pack_voltage_v: Real | None = None
    pack_current_a: Real | None = None


class BenchTrace(Schema):
    metadata: BenchMetadata
    samples: tuple[BenchSample, ...] = Field(min_length=1, max_length=50_000)
    csv_sha256: str = Field(pattern=r"^[a-f0-9]{64}$")
    metadata_sha256: str = Field(pattern=r"^[a-f0-9]{64}$")

    @model_validator(mode="after")
    def sample_order(self):
        if any(a.time_s >= b.time_s for a, b in zip(self.samples, self.samples[1:])):
            raise ValueError("bench timestamps must be strictly increasing")
        if not any(
            getattr(sample, signal) is not None for sample in self.samples for signal in SIGNALS
        ):
            raise ValueError("bench trace must contain at least one measured value")
        return self


def load_bench_trace(csv_path: str | Path, metadata_path: str | Path) -> BenchTrace:
    try:
        with Path(csv_path).open("rb") as stream:
            raw = stream.read(4 * 1024 * 1024 + 1)
        with Path(metadata_path).open("rb") as stream:
            metadata_raw = stream.read(65_537)
    except OSError as exc:
        raise ValueError(f"Invalid bench trace: {exc}") from exc
    return decode_bench_trace(raw, metadata_raw)


def decode_bench_trace(raw: bytes, metadata_raw: bytes) -> BenchTrace:
    """Read the same bounded format from archive members without extracting files."""
    try:
        if len(raw) > 4 * 1024 * 1024:
            raise ValueError("bench CSV exceeds 4 MiB")
        metadata = BenchMetadata.model_validate(
            decode_json_object(metadata_raw, maximum_bytes=65_536, label="bench metadata")
        )
        reader = csv.reader(io.StringIO(raw.decode("utf-8-sig"), newline=""), strict=True)
        headers = next(reader, [])
        if (
            len(headers) != len(set(headers))
            or "time_s" not in headers
            or not set(headers) <= {"time_s", *SIGNALS}
            or len(headers) < 2
        ):
            raise ValueError(
                "CSV requires unique time_s and at least one supported signal column with explicit units"
            )
        samples = []
        for row in reader:
            if not row:
                continue
            if len(row) != len(headers) or len(samples) >= 50_000:
                raise ValueError("bench CSV has a ragged row or exceeds 50,000 samples")
            values = {
                name: None if value.strip() == "" else float(value)
                for name, value in zip(headers, row)
            }
            samples.append(BenchSample.model_validate(values))
        return BenchTrace(
            metadata=metadata,
            samples=tuple(samples),
            csv_sha256=hashlib.sha256(raw).hexdigest(),
            metadata_sha256=hashlib.sha256(metadata_raw).hexdigest(),
        )
    except (OSError, ValueError, csv.Error, RecursionError) as exc:
        raise ValueError(f"Invalid bench trace: {exc}") from exc


def _select_bench_run(report: dict, case_name: str | None = None) -> dict:
    if report.get("experiment") == "rover_acceptance_suite":
        from .rover_comparison import validate_rover_suite_evidence

        _, _, cases = validate_rover_suite_evidence(report)
        if case_name is None and len(cases) != 1:
            raise ValueError("select a case from the multi-case rover suite")
        row = next(
            (row for name, row in cases.items() if case_name is None or name == case_name), None
        )
        if row is None or row["report"] is None:
            raise ValueError("selected case is missing or has no completed simulation evidence")
        return row["report"]
    if case_name is not None:
        raise ValueError("case selection applies only to a suite report")
    if report.get("experiment") != "straight_line_wall_brake":
        raise ValueError(
            "bench comparison requires one rover run or an explicitly selected suite case"
        )
    project_rover_evidence(report)
    return report


def select_bench_run(report: dict, case_name: str | None = None) -> dict:
    try:
        return _select_bench_run(report, case_name)
    except (KeyError, TypeError, AttributeError, ValueError, RecursionError) as exc:
        raise ValueError(f"Invalid bench simulation evidence: {exc}") from exc


def recompute_bench_comparison(report: dict) -> dict:
    """Rebuild alignment and errors from saved measurements, never edited totals."""
    try:
        if (
            report.get("experiment") != "rover_bench_comparison"
            or type(report.get("schema_version")) is not int
            or report["schema_version"] != 1
        ):
            raise ValueError("requires bench comparison schema 1")
        if any(not set(row["measurement"]) <= set(SIGNALS) for row in report["rows"]):
            raise ValueError("bench measurement rows contain unknown signal fields")
        trace = BenchTrace.model_validate(
            {
                "metadata": report["metadata"],
                "samples": [
                    {"time_s": row["measurement_time_s"], **row["measurement"]}
                    for row in report["rows"]
                ],
                "csv_sha256": report["sources"]["csv_sha256"],
                "metadata_sha256": report["sources"]["metadata_sha256"],
            }
        )
        alignment = report["alignment"]
        result = compare_bench_trace(
            report["simulation"],
            trace,
            time_offset_s=alignment["time_offset_s"],
            maximum_sample_age_s=alignment["maximum_sample_age_s"],
            report_sha256=report["sources"]["run_sha256"],
        )
        selected = alignment.get("selected_case")
        if selected is not None and (not isinstance(selected, str) or not selected.strip()):
            raise ValueError("selected case must contain a name or be null")
        result["alignment"]["selected_case"] = selected
        return result
    except (KeyError, TypeError, AttributeError, ValueError, RecursionError) as exc:
        raise ValueError(f"Invalid saved bench comparison: {exc}") from exc


def compare_bench_trace(
    report: dict,
    trace: BenchTrace,
    *,
    time_offset_s: float,
    maximum_sample_age_s: float,
    case_name: str | None = None,
    report_sha256: str | None = None,
) -> dict:
    offset = finite(time_offset_s, "bench time offset")
    maximum_age = nonnegative(maximum_sample_age_s, "maximum sample age")
    run = select_bench_run(report, case_name)
    samples = project_rover_evidence(run)["cases"][0]["run"]["samples"]
    times = [sample["time_s"] for sample in samples]
    minimum_spacing = min((b - a for a, b in zip(times, times[1:])), default=None)
    errors = {name: [] for name in SIGNALS}
    rows, counts = (
        [],
        {
            "aligned": 0,
            "before_run": 0,
            "after_run": 0,
            "stale": 0,
            "insufficient_time_precision": 0,
        },
    )
    for measurement in trace.samples:
        time = finite(measurement.time_s + offset, "aligned bench time")
        source_rounding = 4 * max(math.ulp(measurement.time_s), math.ulp(offset))
        before_rounding = max(source_rounding, 4 * max(math.ulp(time), math.ulp(times[0])))
        after_rounding = max(source_rounding, 4 * max(math.ulp(time), math.ulp(times[-1])))
        index = bisect.bisect_right(times, time) - 1
        if index + 1 < len(times):
            next_time = times[index + 1]
            if next_time - time <= max(
                source_rounding, 4 * max(math.ulp(time), math.ulp(next_time))
            ):
                index += 1
        reason = (
            "before_run"
            if time < times[0] - before_rounding
            else "after_run"
            if time > times[-1] + after_rounding
            else "aligned"
        )
        if minimum_spacing is not None and source_rounding >= minimum_spacing / 2:
            reason = "insufficient_time_precision"
        sample = None if reason != "aligned" else samples[max(0, index)]
        age = None if sample is None else max(0.0, time - sample["time_s"])
        rounding = (
            0
            if sample is None
            else max(source_rounding, 4 * max(math.ulp(time), math.ulp(sample["time_s"])))
        )
        if age is not None and age > maximum_age + rounding:
            reason = "stale"
        values = {}
        if reason == "aligned":
            for name in SIGNALS:
                measured = getattr(measurement, name)
                if measured is not None:
                    predicted = sample[name]
                    error = finite(predicted - measured, f"{name} comparison error")
                    errors[name].append(error)
                    values[name] = {"measured": measured, "predicted": predicted, "error": error}
        counts[reason] += 1
        rows.append(
            {
                "measurement_time_s": measurement.time_s,
                "aligned_time_s": time,
                "measurement": measurement.model_dump(mode="json", exclude={"time_s"}),
                "sample_time_s": None if sample is None else sample["time_s"],
                "sample_age_s": age,
                "status": reason,
                "values": values,
            }
        )
    metrics = {}
    for name, values in errors.items():
        count = len(values)
        metrics[name] = {
            "unit": SIGNALS[name],
            "paired_values": count,
            "mean_error": None if not count else math.fsum(value / count for value in values),
            "rmse": None
            if not count
            else math.hypot(*(value / math.sqrt(count) for value in values)),
            "maximum_absolute_error": None if not count else max(abs(value) for value in values),
        }
    return {
        "schema_version": 1,
        "experiment": "rover_bench_comparison",
        "result_type": "recorded_data_comparison",
        "status": "compared" if any(errors.values()) else "no_paired_values",
        "source_kind": trace.metadata.source_kind,
        "metadata": trace.metadata.model_dump(mode="json"),
        "sources": {
            "run_sha256": report_sha256,
            "csv_sha256": trace.csv_sha256,
            "metadata_sha256": trace.metadata_sha256,
        },
        "alignment": {
            "time_offset_s": offset,
            "maximum_sample_age_s": maximum_age,
            "method": "latest_recorded_sample",
            "error_sign": "prediction_minus_measurement",
            "selected_case": case_name,
        },
        "counts": {
            "total": len(rows),
            "paired_rows": sum(bool(row["values"]) for row in rows),
            **counts,
        },
        "metrics": metrics,
        "rows": rows,
        "simulation": run,
        "limitations": [
            "Measured/synthetic origin and instrument metadata are user declarations, not authenticated observations",
            "Positive offset adds to CSV time; alignment never automatically shifts the first sample or extrapolates outside the run",
            "Timestamp comparisons allow four floating-point ULPs of source, offset and matched times; inadequate precision for the recorded cadence is excluded",
            "No interpolation, filtering, clock-drift correction or automatic calibration",
            "Errors describe paired saved values, not validated hardware accuracy, confidence intervals or an acceptance certificate",
            "Current and PWM boundary samples inherit the fixture's interval semantics; sensor and instrument dynamics are not compensated",
            "Wheel versus ground speed and operating conditions must match the intended comparison; the fixture assumes no slip",
        ],
    }
