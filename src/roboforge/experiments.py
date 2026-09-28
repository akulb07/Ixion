"""Bounded parameter/seed experiments with immutable trial identity and failure retention."""

import copy
import csv
import hashlib
import itertools
import json
import math
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path
from typing import Annotated, Any, Callable

import numpy as np
from pydantic import Field, model_validator

from roboforge import __version__
from roboforge.config import RunConfig, Schema, Steps, _UniqueKeyLoader
from roboforge.simulation import SimulationResult, Simulator


class SweepAxis(Schema):
    path: str = Field(pattern=r"^[a-z][a-z0-9_]*(?:\.(?:[a-z][a-z0-9_]*|[0-9]+))*$")
    values: tuple[Any, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def finite_json_values(self):
        try:
            json.dumps(self.values, allow_nan=False)
        except (ValueError, TypeError) as exc:
            raise ValueError("sweep values must be finite JSON values") from exc
        return self


class ExperimentConfig(Schema):
    name: str = Field(default="experiment", min_length=1)
    base: RunConfig
    seeds: tuple[Annotated[int, Field(strict=True, ge=0)], ...] = (42,)
    axes: tuple[SweepAxis, ...] = ()
    max_runs: Steps = Field(default=100, le=1000)
    max_total_steps: Steps = Field(default=2_000_000, le=10_000_000)

    @model_validator(mode="after")
    def check_design(self):
        if not self.seeds or len(set(self.seeds)) != len(self.seeds):
            raise ValueError("experiment seeds must be nonempty and unique")
        if math.prod(len(axis.values) for axis in self.axes) * len(self.seeds) > self.max_runs:
            raise ValueError("experiment exceeds run budget")
        paths = [axis.path for axis in self.axes]
        for i, path in enumerate(paths):
            if path == "seed":
                raise ValueError("vary seeds through the seeds field")
            if any(
                path == other or path.startswith(other + ".") or other.startswith(path + ".")
                for other in paths[i + 1 :]
            ):
                raise ValueError("sweep paths must not overlap")
        return self


def load_experiment(path: str | Path) -> ExperimentConfig:
    import yaml

    try:
        return ExperimentConfig.model_validate(
            yaml.load(Path(path).read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
        )
    except (ValueError, yaml.YAMLError, RecursionError) as exc:
        raise ValueError(f"Invalid experiment configuration {path}: {exc}") from exc


def _assign(document, path, value):
    pieces = path.split(".")
    current = document
    for piece in pieces[:-1]:
        if isinstance(current, list) and piece.isdecimal() and int(piece) < len(current):
            current = current[int(piece)]
        elif isinstance(current, dict) and piece in current:
            current = current[piece]
        else:
            raise ValueError(f"unknown sweep path: {path}")
    last = pieces[-1]
    if isinstance(current, list) and last.isdecimal() and int(last) < len(current):
        current[int(last)] = copy.deepcopy(value)
    elif isinstance(current, dict) and last in current:
        current[last] = copy.deepcopy(value)
    else:
        raise ValueError(f"unknown sweep path: {path}")


def simulation_metrics(result: SimulationResult) -> dict[str, float]:
    """No energy claim: effort is the wheel-rate-squared time integral."""
    path = sum(motion.travel for motion in result.motions)
    effort = sum((m.wheels.left**2 + m.wheels.right**2) * m.dt for m in result.motions)
    final = result.states[-1]
    metrics = {
        "duration_s": final.time,
        "path_length_m": path,
        "collision_count": float(len(result.collisions)),
        "wheel_rate_squared_integral_rad2_per_s": effort,
        "final_x_m": final.pose.x,
        "final_y_m": final.pose.y,
        "final_heading_rad": final.pose.theta,
    }
    if result.control_samples:
        metrics["wheel_tracking_rmse_rad_s"] = math.sqrt(
            np.mean(
                [
                    sample.error**2
                    for update in result.control_samples
                    for sample in (update.left, update.right)
                ]
            )
        )
    if result.navigation_samples:
        sample = result.navigation_samples[-1]
        goal = result.config.navigation.path[-1]
        metrics["estimated_goal_error_m"] = sample.tracking.goal_distance
        metrics["truth_goal_error_m"] = math.hypot(final.pose.x - goal.x, final.pose.y - goal.y)
        metrics["navigation_reached"] = float(result.navigation_outcome == "reached")
    return metrics


def _summarize(records):
    groups = {}
    for record in records:
        groups.setdefault(record["group"], []).append(record)
    summary = {}
    for key, values in groups.items():
        failed = sum(v["status"] != "completed" for v in values)
        n, z = len(values), 1.959963984540054
        proportion = failed / n
        center = (proportion + z * z / (2 * n)) / (1 + z * z / n)
        half = (
            z * math.sqrt(proportion * (1 - proportion) / n + z * z / (4 * n * n)) / (1 + z * z / n)
        )
        metrics = {}
        for name in sorted({name for v in values for name in v["metrics"]}):
            samples = np.array([v["metrics"][name] for v in values if name in v["metrics"]])
            metrics[name] = {
                "measured_trials": len(samples),
                "mean": float(samples.mean()),
                "sample_stddev": float(samples.std(ddof=1)) if len(samples) > 1 else None,
                "median": float(np.median(samples)),
                "p05": float(np.quantile(samples, 0.05)),
                "p95": float(np.quantile(samples, 0.95)),
            }
        summary[key] = {
            "trials": n,
            "failed_trials": failed,
            "failure_rate": proportion,
            "failure_rate_wilson95": [max(0, center - half), min(1, center + half)],
            "metrics": metrics,
        }
    return summary


def write_manifest(directory: Path):
    names = sorted(p.name for p in directory.iterdir() if p.is_file() and p.name != "manifest.json")
    _write_json(
        directory / "manifest.json",
        {
            "format_version": 1,
            "files": {
                name: hashlib.sha256((directory / name).read_bytes()).hexdigest() for name in names
            },
        },
    )


def _write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def expand_experiment(config: ExperimentConfig):
    """Resolve every trial before execution; retain invalid values but reject unknown paths."""
    variants, total_steps = [], 0
    for group_index, combination in enumerate(itertools.product(*(a.values for a in config.axes))):
        parameters = dict(zip((a.path for a in config.axes), combination))
        for seed in config.seeds:
            document = config.base.model_dump(mode="json")
            for path, value in parameters.items():
                _assign(document, path, value)
            document["seed"] = seed
            # Invalid individual configurations are retained, while path typos abort before any run.
            try:
                resolved = RunConfig.model_validate(document)
                total_steps += resolved.step_budget
                validation_error = None
            except ValueError as exc:
                resolved, validation_error = None, str(exc)
            variants.append(
                (f"group-{group_index:04d}", parameters, seed, document, resolved, validation_error)
            )
    if total_steps > config.max_total_steps:
        raise ValueError("experiment exceeds total simulation-step budget")
    return variants


def run_experiment(
    config: ExperimentConfig,
    output: str | Path,
    *,
    runner: Callable[[RunConfig], SimulationResult] | None = None,
    metric_functions: dict[str, Callable[[SimulationResult], float]] | None = None,
    plot: bool = False,
) -> Path:
    """Run sequentially; each invocation gets a new UUID directory and retains failures."""
    from roboforge.io import save_result

    runner = (lambda c: Simulator(c).run()) if runner is None else runner
    variants = expand_experiment(config)
    directory = Path(output) / f"experiment-{uuid.uuid4().hex}"
    directory.mkdir(parents=True, exist_ok=False)
    _write_json(directory / "experiment.json", config.model_dump(mode="json"))
    records = []
    for index, (group, parameters, seed, document, resolved, validation_error) in enumerate(
        variants
    ):
        canonical = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)
        digest = hashlib.sha256(canonical.encode()).hexdigest()
        trial_id = f"trial-{index:04d}-{digest[:12]}"
        trial_dir = directory / trial_id
        trial_dir.mkdir()
        _write_json(trial_dir / "input.json", document)
        record = {
            "run_id": f"{directory.name}/{trial_id}",
            "trial_id": trial_id,
            "group": group,
            "parameters": parameters,
            "seed": seed,
            "config_sha256": digest,
            "software_version": __version__,
            "started_utc": datetime.now(UTC).isoformat(),
            "status": "error",
            "metrics": {},
            "error": None,
        }
        start = time.perf_counter()
        try:
            if validation_error is not None:
                raise ValueError(validation_error)
            result = runner(resolved)
            if result.config != resolved:
                raise ValueError("runner returned a different configuration")
            save_result(result, trial_dir)
            measured = simulation_metrics(result)
            for name, function in (metric_functions or {}).items():
                if name in measured:
                    raise ValueError(f"metric already exists: {name}")
                measured[name] = float(function(result))
            if not all(math.isfinite(value) for value in measured.values()):
                raise ValueError("metrics must all be finite")
            record["metrics"] = measured
            record["status"] = result.status
            if plot:
                from roboforge.visualization import plot_trajectory

                plot_trajectory(result, trial_dir / "trajectory.png")
        except Exception as exc:
            record["status"] = "error"
            record["error"] = {"type": type(exc).__name__, "message": str(exc)}
        record["wall_runtime_s"] = time.perf_counter() - start
        _write_json(trial_dir / "trial.json", record)
        write_manifest(trial_dir)
        records.append(record)
        # Atomic report replacement preserves completed trials if a later run is interrupted.
        staging = directory / "report.pending.json"
        _write_json(
            staging,
            {
                "format_version": 1,
                "expected_trials": len(variants),
                "finished_trials": len(records),
                "trials": records,
                "groups": _summarize(records),
            },
        )
        staging.replace(directory / "report.json")
    columns = ["trial_id", "group", "seed", "status", "wall_runtime_s"]
    metric_names = sorted({key for r in records for key in r["metrics"]})
    with (directory / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns + metric_names)
        writer.writeheader()
        for record in records:
            writer.writerow({**{key: record[key] for key in columns}, **record["metrics"]})
    return directory


def paired_differences(report: dict, baseline: str, challenger: str, metric: str) -> dict:
    """Challenger-minus-baseline by shared seed; exclusions remain explicit."""
    groups = [
        {r["seed"]: r for r in report["trials"] if r["group"] == group}
        for group in (baseline, challenger)
    ]
    if not all(groups):
        raise ValueError("paired comparison requires two existing groups")
    differences, excluded = [], []
    for seed in sorted(set(groups[0]) | set(groups[1])):
        pair = [g.get(seed) for g in groups]
        if any(r is None or r["status"] != "completed" or metric not in r["metrics"] for r in pair):
            excluded.append(seed)
        else:
            differences.append(
                {
                    "seed": seed,
                    "difference": pair[1]["metrics"][metric] - pair[0]["metrics"][metric],
                }
            )
    return {
        "baseline": baseline,
        "challenger": challenger,
        "metric": metric,
        "pairs": differences,
        "excluded_seeds": excluded,
        "mean_difference": float(np.mean([d["difference"] for d in differences]))
        if differences
        else None,
    }
