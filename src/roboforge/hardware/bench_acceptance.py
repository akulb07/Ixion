"""Declared error and evidence limits for a saved bench comparison."""

import hashlib
import math
from pathlib import Path
from typing import Literal

from pydantic import Field, StrictInt, model_validator

from roboforge.config import Nonnegative, Probability, Schema

from .artifact_io import decode_json_object
from .bench_trace import SIGNALS, recompute_bench_comparison
from .design_identity import canonical_sha256

Signal = Literal["speed_m_s", "distance_m", "pack_voltage_v", "pack_current_a"]


class BenchSignalLimits(Schema):
    minimum_pairs: StrictInt = Field(ge=1, le=50_000)
    minimum_paired_fraction: Probability
    minimum_paired_span_s: Nonnegative
    maximum_paired_gap_s: Nonnegative | None = None
    maximum_rmse: Nonnegative | None = None
    maximum_absolute_error: Nonnegative | None = None
    maximum_absolute_mean_error: Nonnegative | None = None

    @model_validator(mode="after")
    def has_error_limit(self):
        if all(
            getattr(self, field) is None
            for field in ("maximum_rmse", "maximum_absolute_error", "maximum_absolute_mean_error")
        ):
            raise ValueError("each signal requires at least one error limit")
        return self


class BenchAcceptancePolicy(Schema):
    schema_version: Literal[1] = 1
    name: str = Field(min_length=1, max_length=200)
    reference: str = Field(min_length=1, max_length=2000)
    source_kind: Literal["measured", "synthetic"]
    speed_reference: Literal["wheel", "ground"] | None = None
    maximum_alignment_age_s: Nonnegative
    signals: dict[Signal, BenchSignalLimits] = Field(min_length=1, max_length=4)

    @model_validator(mode="after")
    def explicit_context(self):
        if not self.name.strip() or not self.reference.strip():
            raise ValueError("policy name and reference must contain text")
        if "speed_m_s" in self.signals and self.speed_reference is None:
            raise ValueError("speed limits require an explicit wheel or ground reference")
        return self


def load_bench_policy(path: str | Path) -> tuple[BenchAcceptancePolicy, str]:
    with Path(path).open("rb") as stream:
        raw = stream.read(65_537)
    policy = BenchAcceptancePolicy.model_validate(
        decode_json_object(raw, maximum_bytes=65_536, label="bench acceptance policy")
    )
    return policy, hashlib.sha256(raw).hexdigest()


def check_bench_comparison(
    report: dict,
    policy: BenchAcceptancePolicy,
    *,
    comparison_sha256: str | None = None,
    policy_source_sha256: str | None = None,
) -> dict:
    comparison = recompute_bench_comparison(report)
    checks = []

    def check(name, actual, expected, *, unit="", minimum=False, equal=False):
        if actual is None:
            status = "inconclusive"
            reason = "No paired evidence is available for this statistic"
        else:
            passed = (
                actual == expected
                if equal
                else actual >= expected
                if minimum
                else actual <= expected
            )
            status = "pass" if passed else "fail"
            operator = "=" if equal else ">=" if minimum else "<="
            reason = f"{actual} {operator} {expected} {unit}".rstrip()
        checks.append(
            {
                "name": name,
                "status": status,
                "actual": actual,
                "expected": expected,
                "unit": unit,
                "reason": reason,
            }
        )

    check("declared_source_kind", comparison["source_kind"], policy.source_kind, equal=True)
    check(
        "alignment_sample_age_limit",
        comparison["alignment"]["maximum_sample_age_s"],
        policy.maximum_alignment_age_s,
        unit="s",
    )
    if "speed_m_s" in policy.signals:
        check(
            "speed_reference",
            comparison["metadata"]["speed_reference"],
            policy.speed_reference,
            equal=True,
        )
    coverage = {}
    for name, limits in sorted(policy.signals.items()):
        metric = comparison["metrics"][name]
        supplied = sum(row["measurement"][name] is not None for row in comparison["rows"])
        times = [row["aligned_time_s"] for row in comparison["rows"] if name in row["values"]]
        count = len(times)
        span = None if not count else times[-1] - times[0]
        gap = None if count < 2 else max(b - a for a, b in zip(times, times[1:]))
        if any(value is not None and not math.isfinite(value) for value in (span, gap)):
            raise ValueError("bench coverage times exceed the finite numeric range")
        fraction = None if not supplied else count / supplied
        coverage[name] = {
            "supplied_values": supplied,
            "paired_values": count,
            "paired_fraction": fraction,
            "paired_span_s": span,
            "maximum_paired_gap_s": gap,
        }
        check(f"{name}.minimum_pairs", count, limits.minimum_pairs, minimum=True, unit="values")
        check(
            f"{name}.minimum_paired_fraction",
            fraction,
            limits.minimum_paired_fraction,
            minimum=True,
        )
        check(
            f"{name}.minimum_paired_span",
            span,
            limits.minimum_paired_span_s,
            minimum=True,
            unit="s",
        )
        if limits.maximum_paired_gap_s is not None:
            check(f"{name}.maximum_paired_gap", gap, limits.maximum_paired_gap_s, unit="s")
        for field, actual in (
            ("maximum_rmse", metric["rmse"]),
            ("maximum_absolute_error", metric["maximum_absolute_error"]),
            (
                "maximum_absolute_mean_error",
                None if metric["mean_error"] is None else abs(metric["mean_error"]),
            ),
        ):
            expected = getattr(limits, field)
            if expected is not None:
                check(f"{name}.{field}", actual, expected, unit=SIGNALS[name])
    status = (
        "fail"
        if any(row["status"] == "fail" for row in checks)
        else "inconclusive"
        if any(row["status"] == "inconclusive" for row in checks)
        else "pass"
    )
    return {
        "schema_version": 1,
        "experiment": "rover_bench_acceptance",
        "result_type": "declared_bench_acceptance",
        "status": status,
        "source_kind": comparison["source_kind"],
        "policy": policy.model_dump(mode="json"),
        "policy_sha256": canonical_sha256(policy.model_dump(mode="json")),
        "sources": {
            "comparison_sha256": comparison_sha256,
            "policy_source_sha256": policy_source_sha256,
            "fingerprint_scope": "source bytes when supplied; policy_sha256 is normalized policy JSON",
        },
        "checks": checks,
        "coverage": coverage,
        "comparison": comparison,
        "limitations": [
            "Pass means the supplied evidence satisfies the declared limits; it is not an engineering readiness certificate",
            "Measured origin, instrument, operating conditions and clock alignment remain user declarations",
            "No proof that limits were set before data collection; save and review the policy before the experiment",
            "Coverage fraction counts nonmissing supplied values of each signal; it does not infer missing instrument samples",
            "Paired span and gaps describe recorded sample times, not continuous observation or statistical independence",
            "No instrument uncertainty propagation, confidence interval, model calibration or physical safety certification",
        ],
    }


def recompute_bench_acceptance(report: dict) -> dict:
    try:
        if (
            report.get("experiment") != "rover_bench_acceptance"
            or type(report.get("schema_version")) is not int
            or report["schema_version"] != 1
        ):
            raise ValueError("requires bench acceptance schema 1")
        return check_bench_comparison(
            report["comparison"],
            BenchAcceptancePolicy.model_validate(report["policy"]),
            comparison_sha256=report["sources"].get("comparison_sha256"),
            policy_source_sha256=report["sources"].get("policy_source_sha256"),
        )
    except (KeyError, TypeError, ValueError, AttributeError, RecursionError) as exc:
        raise ValueError(f"Invalid saved bench acceptance: {exc}") from exc
