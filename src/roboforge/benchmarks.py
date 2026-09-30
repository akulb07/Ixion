"""Versioned static-world planner benchmarks with explicit budgets and failures."""

import csv
import hashlib
import json
import time
import uuid
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Annotated, Literal

import numpy as np
from pydantic import Field, model_validator

from roboforge import __version__
from roboforge.config import Circle, Environment, Positive, Rectangle, Schema, Steps
from roboforge.experiments import write_manifest
from roboforge.geometry import Vector2
from roboforge.planning import PlanningWorld, grid_plan, sampling_plan
from roboforge.reports import report_html, trial_csv


@dataclass(frozen=True, slots=True)
class BenchmarkCase:
    name: str
    environment: Environment
    start: Vector2
    goal: Vector2


def standard_cases() -> dict[str, BenchmarkCase]:
    """Suite v1: exact geometry is also saved alongside every benchmark report."""
    cases = [
        BenchmarkCase("empty_room", Environment(width=8, height=8), Vector2(1, 1), Vector2(7, 7)),
        BenchmarkCase(
            "corridor",
            Environment(
                width=8,
                height=4,
                obstacles=(
                    Rectangle(x=0, y=0, width=8, height=1),
                    Rectangle(x=0, y=3, width=8, height=1),
                ),
            ),
            Vector2(0.75, 2),
            Vector2(7.25, 2),
        ),
        BenchmarkCase(
            "maze",
            Environment(
                width=8,
                height=8,
                obstacles=(
                    Rectangle(x=2, y=0, width=0.25, height=5.5),
                    Rectangle(x=4, y=2.5, width=0.25, height=5.5),
                    Rectangle(x=6, y=0, width=0.25, height=5.5),
                ),
            ),
            Vector2(1, 1),
            Vector2(7, 7),
        ),
        BenchmarkCase(
            "clutter",
            Environment(
                width=8,
                height=8,
                obstacles=tuple(
                    Circle(x=x, y=y, radius=0.45)
                    for x, y in ((2, 2), (4, 2), (6, 2), (3, 4), (5, 4), (2, 6), (4, 6), (6, 6))
                ),
            ),
            Vector2(0.75, 0.75),
            Vector2(7.25, 7.25),
        ),
    ]
    return {case.name: case for case in cases}


class BenchmarkConfig(Schema):
    cases: tuple[Literal["empty_room", "corridor", "maze", "clutter"], ...] = (
        "empty_room",
        "corridor",
        "maze",
        "clutter",
    )
    algorithms: tuple[Literal["astar", "dijkstra", "rrt", "rrt_star"], ...] = (
        "astar",
        "dijkstra",
        "rrt",
        "rrt_star",
    )
    seeds: tuple[Annotated[int, Field(strict=True, ge=0)], ...] = (42,)
    radius: Positive = 0.2
    resolution: Positive = 0.25
    iterations: Steps = Field(default=500, le=10000)
    max_expansions: Steps = Field(default=10000, le=1000000)
    step_size: Positive = 0.5
    rewire_radius: Positive = 1.0

    @model_validator(mode="after")
    def design(self):
        if any(
            not values or len(set(values)) != len(values)
            for values in (self.cases, self.algorithms, self.seeds)
        ):
            raise ValueError("benchmark cases, algorithms and seeds must be nonempty and unique")
        if len(self.cases) * len(self.algorithms) * len(self.seeds) > 1000:
            raise ValueError("benchmark exceeds 1000-trial budget")
        return self


def run_benchmarks(config: BenchmarkConfig, output: str | Path) -> Path:
    directory = Path(output) / f"benchmark-{uuid.uuid4().hex}"
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "config.json").write_text(config.model_dump_json(indent=2), encoding="utf-8")
    cases = standard_cases()
    worlds = {
        name: {
            "environment": cases[name].environment.model_dump(mode="json"),
            "start": asdict(cases[name].start),
            "goal": asdict(cases[name].goal),
        }
        for name in config.cases
    }
    (directory / "cases.json").write_text(json.dumps(worlds, indent=2), encoding="utf-8")
    records = []
    created = datetime.now(UTC).isoformat()
    report = {
        "id": directory.name,
        "suite_version": 1,
        "software_version": __version__,
        "created_utc": created,
        "status": "running",
        "expected_trials": len(config.cases) * len(config.algorithms) * len(config.seeds),
        "finished_trials": 0,
        "specification_sha256": hashlib.sha256(
            json.dumps(
                config.model_dump(mode="json"), sort_keys=True, separators=(",", ":")
            ).encode()
        ).hexdigest(),
        "trials": records,
    }
    for name in config.cases:
        case = cases[name]
        for algorithm in config.algorithms:
            for seed in config.seeds:
                record = {
                    "case": name,
                    "algorithm": algorithm,
                    "seed": seed,
                    "status": "error",
                    "path_length_m": None,
                    "expanded": None,
                    "collision_checks": None,
                    "verified_collision_free": None,
                    "error": None,
                }
                start = time.perf_counter()
                try:
                    if algorithm in ("astar", "dijkstra"):
                        plan = grid_plan(
                            case.environment,
                            case.start,
                            case.goal,
                            config.radius,
                            config.resolution,
                            algorithm,
                            config.max_expansions,
                        )
                    else:
                        plan = sampling_plan(
                            case.environment,
                            case.start,
                            case.goal,
                            config.radius,
                            algorithm,
                            seed,
                            config.iterations,
                            config.step_size,
                            rewire_radius=config.rewire_radius,
                        )
                    record["planner_runtime_s"] = time.perf_counter() - start
                    record.update(
                        status=plan.status,
                        expanded=plan.expanded,
                        collision_checks=plan.collision_checks,
                    )
                    if plan.status == "success":
                        oracle = PlanningWorld(case.environment, config.radius)
                        valid = (
                            plan.path[0] == case.start
                            and plan.path[-1] == case.goal
                            and all(
                                oracle.segment_free(a, b) for a, b in zip(plan.path, plan.path[1:])
                            )
                        )
                        record["verified_collision_free"] = valid
                        if not valid:
                            raise ValueError(
                                "planner returned a path that failed independent replay of edge checks"
                            )
                        record["path_length_m"] = plan.length
                    filename = f"{name}-{algorithm}-{seed}.json"
                    (directory / filename).write_text(
                        json.dumps(asdict(plan), indent=2), encoding="utf-8"
                    )
                    record["artifact"] = filename
                except Exception as exc:
                    record["planner_runtime_s"] = time.perf_counter() - start
                    record["status"] = "error"
                    record["error"] = {"type": type(exc).__name__, "message": str(exc)}
                records.append(record)
                report["finished_trials"] = len(records)
                if len(records) == report["expected_trials"]:
                    report.update(status="completed", finished_utc=datetime.now(UTC).isoformat())
                pending = directory / "report.pending.json"
                pending.write_text(
                    json.dumps(
                        report,
                        indent=2,
                    ),
                    encoding="utf-8",
                )
                pending.replace(directory / "report.json")
    summaries = []
    for name in config.cases:
        for algorithm in config.algorithms:
            group = [r for r in records if r["case"] == name and r["algorithm"] == algorithm]
            successes = [r for r in group if r["status"] == "success"]
            summaries.append(
                {
                    "case": name,
                    "algorithm": algorithm,
                    "trials": len(group),
                    "successful_paths": len(successes),
                    "failure_rate": 1 - len(successes) / len(group),
                    "mean_successful_path_length_m": float(
                        np.mean([r["path_length_m"] for r in successes])
                    )
                    if successes
                    else None,
                    "median_planner_runtime_s": float(
                        np.median([r["planner_runtime_s"] for r in group])
                    ),
                }
            )
    with (directory / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    (directory / "summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    (directory / "trials.csv").write_text(
        trial_csv(report, "benchmark"), encoding="utf-8", newline=""
    )
    (directory / "report.html").write_text(
        report_html(
            report,
            kind="benchmark",
            design=config.model_dump(mode="json"),
            cases=worlds,
            summaries=summaries,
        ),
        encoding="utf-8",
    )
    write_manifest(directory)
    return directory
