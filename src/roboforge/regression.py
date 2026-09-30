"""Acceptance checks over saved runs. No reruns and no implicit baseline updates."""

import hashlib
import json
import math
from pathlib import Path
from typing import Literal

from pydantic import Field, model_validator

from roboforge.comparison import ComparisonRequest, compare_runs
from roboforge.config import Real, Schema
from roboforge.service import RUN_ID, ServiceError


class MetricRule(Schema):
    name: str = Field(min_length=1, max_length=100)
    metric: str = Field(min_length=1, max_length=100)
    mode: Literal["absolute", "delta"] = "absolute"
    minimum: Real | None = None
    maximum: Real | None = None

    @model_validator(mode="after")
    def bounds(self):
        if self.minimum is None and self.maximum is None:
            raise ValueError("a rule needs at least one bound")
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError("minimum must not exceed maximum")
        return self


class RegressionPolicy(Schema):
    name: str = Field(min_length=1, max_length=100)
    rules: tuple[MetricRule, ...] = Field(min_length=1, max_length=64)
    allowed_config_changes: tuple[str, ...] = Field(default=(), max_length=64)
    allow_software_change: bool = False
    require_goal_reached: bool = False
    max_goal_error_m: Real | None = Field(default=None, ge=0)

    @model_validator(mode="after")
    def unique_names(self):
        if len({rule.name for rule in self.rules}) != len(self.rules):
            raise ValueError("rule names must be unique")
        return self


class RegressionRequest(Schema):
    baseline_id: str
    candidate_id: str
    policy: RegressionPolicy


class SavedRuns:
    """Read artifacts without opening the worker service or modifying its job store."""

    def __init__(self, root):
        self.root = Path(root).resolve()

    def artifact(self, run_id, filename):
        if not RUN_ID.fullmatch(run_id):
            raise ValueError("invalid run ID")
        directory = (self.root / run_id).resolve()
        path = (directory / filename).resolve()
        if directory.parent != self.root or path.parent != directory:
            raise ValueError("artifact path leaves run store")
        return path

    def get(self, run_id):
        record = json.loads(self.artifact(run_id, "job.json").read_text(encoding="utf-8"))
        if record.get("id") != run_id:
            raise ValueError("recorded run ID does not match its directory")
        return record


def check_regression(service, specification: RegressionRequest):
    comparison = compare_runs(
        service, ComparisonRequest(run_ids=(specification.baseline_id, specification.candidate_id))
    )
    baseline, candidate = comparison["runs"]
    policy = specification.policy
    checks = []

    def add(name, status, reason, **values):
        checks.append({"name": name, "status": status, "reason": reason, **values})

    add(
        "Candidate completed",
        "pass" if candidate["status"] == "completed" else "fail",
        f"Recorded execution status: {candidate['status']}",
    )
    if policy.require_goal_reached:
        outcome = candidate.get("navigation_outcome")
        known = outcome in {"reached", "budget_exceeded", "collision"}
        add(
            "Navigation goal reached",
            "pass" if outcome == "reached" else "fail" if known else "inconclusive",
            f"Recorded navigation outcome: {outcome}"
            if known
            else "Navigation outcome unavailable or unsupported; completion does not prove goal arrival",
            candidate=outcome,
        )
    if policy.max_goal_error_m is not None:
        navigation = candidate["config"].get("navigation")
        metrics = candidate["metrics"] or {}
        path = navigation.get("path", []) if navigation else []
        x, y = metrics.get("final_x_m"), metrics.get("final_y_m")
        distance = None
        if path and x is not None and y is not None:
            distance = math.hypot(x - path[-1]["x"], y - path[-1]["y"])
            if not math.isfinite(distance):
                distance = None
        add(
            "Simulated final goal error",
            "inconclusive"
            if distance is None
            else "pass"
            if distance <= policy.max_goal_error_m
            else "fail",
            "Simulated ground-truth endpoint distance to configured goal"
            if distance is not None
            else "Configured navigation goal or finite final-position measurements unavailable",
            measured=distance,
            maximum=policy.max_goal_error_m,
        )
    unexpected = [
        row["path"]
        for row in comparison["config_differences"]
        if row["path"] != "name" and row["path"] not in policy.allowed_config_changes
    ]
    compatible = not unexpected and (comparison["same_software"] or policy.allow_software_change)
    add(
        "Comparison compatibility",
        "pass" if compatible else "inconclusive",
        "Setup differences acknowledged by policy"
        if compatible
        else "Unacknowledged setup or software differences",
        unexpected_config_changes=unexpected,
        same_software=comparison["same_software"],
    )
    for rule in policy.rules:
        base = (baseline["metrics"] or {}).get(rule.metric)
        value = (candidate["metrics"] or {}).get(rule.metric)
        measured = value
        reason = ""
        if value is None:
            reason = "Candidate measurement unavailable"
        elif rule.mode == "delta":
            if baseline["status"] != "completed" or base is None:
                reason = "Completed baseline measurement unavailable"
            elif not compatible:
                reason = "Relative check requires compatible or explicitly acknowledged settings"
            else:
                measured = value - base
                if not math.isfinite(measured):
                    reason = "Difference exceeds finite numeric range"
        if reason:
            add(
                rule.name,
                "inconclusive",
                reason,
                metric=rule.metric,
                mode=rule.mode,
                baseline=base,
                candidate=value,
                measured=None,
            )
            continue
        passed = (rule.minimum is None or measured >= rule.minimum) and (
            rule.maximum is None or measured <= rule.maximum
        )
        add(
            rule.name,
            "pass" if passed else "fail",
            "Within bounds" if passed else "Outside bounds",
            metric=rule.metric,
            mode=rule.mode,
            baseline=base,
            candidate=value,
            measured=measured,
            minimum=rule.minimum,
            maximum=rule.maximum,
        )
    status = (
        "fail"
        if any(c["status"] == "fail" for c in checks)
        else "inconclusive"
        if any(c["status"] == "inconclusive" for c in checks)
        else "pass"
    )
    design = policy.model_dump(mode="json")
    return {
        "format_version": 1,
        "status": status,
        "created_utc": comparison["created_utc"],
        "software_version": comparison["software_version"],
        "policy": design,
        "policy_sha256": hashlib.sha256(
            json.dumps(design, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest(),
        "checks": checks,
        "comparison": comparison,
    }


def check_saved_runs(root, specification):
    try:
        return check_regression(SavedRuns(root), specification)
    except (ServiceError, KeyError, TypeError, AttributeError) as exc:
        raise ValueError(str(exc)) from exc
