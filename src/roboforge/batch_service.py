"""Small, durable experiment batches using the existing simulation worker."""

import hashlib
import json
import re
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from typing import Annotated

from pydantic import Field

from roboforge import __version__
from roboforge.experiments import ExperimentConfig, SweepAxis, _summarize, expand_experiment
from roboforge.service import (
    ACTIVE,
    READY,
    RunService,
    ServiceError,
    _stamp,
    _write,
    resource_estimate,
)

BATCH_ID = re.compile(r"^batch-[0-9a-f]{32}$")
TERMINAL = {
    "completed",
    "collision",
    "budget_exceeded",
    "failed",
    "cancelled",
    "interrupted",
    "invalid",
}


class BatchRequest(ExperimentConfig):
    seeds: tuple[Annotated[int, Field(strict=True, ge=0, le=9007199254740991)], ...] = Field(
        default=(42,), min_length=1, max_length=8
    )
    axes: tuple[SweepAxis, ...] = Field(default=(), max_length=2)
    max_runs: Annotated[int, Field(strict=True, ge=1, le=32)] = 32
    max_total_steps: Annotated[int, Field(strict=True, ge=1, le=100000)] = 100000


def _digest(value):
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def prepare_batch(specification: BatchRequest):
    try:
        variants = expand_experiment(specification)
    except ValueError as exc:
        raise ServiceError(str(exc), 422) from exc
    totals = {"steps": 0, "duration_s": 0, "sensor_readings": 0, "lidar_rays": 0}
    trials, inputs, configs = [], [], []
    for index, (group, parameters, seed, document, resolved, error) in enumerate(variants):
        budget = resource_estimate(resolved) if resolved is not None else None
        if budget:
            for key in totals:
                totals[key] += budget[key]
        # Store the exact resolved input, including defaults, for each valid run.
        document = resolved.model_dump(mode="json") if resolved is not None else document
        inputs.append(document)
        configs.append(resolved)
        trials.append(
            {
                "index": index,
                "group": group,
                "parameters": parameters,
                "seed": seed,
                "config_sha256": _digest(document),
                "budget": budget,
                "status": "invalid" if error else "pending",
                "error": error,
                "run_id": None,
                "metrics": {},
                "navigation_outcome": None,
            }
        )
    if (
        totals["sensor_readings"] > 500000
        or totals["lidar_rays"] > 2000000
        or totals["duration_s"] > 1800
    ):
        raise ServiceError(
            "batch exceeds 500,000 readings, 2,000,000 rays or 1,800 simulated seconds", 422
        )
    return {"expected_trials": len(trials), "resources": totals, "trials": trials}, inputs, configs


class BatchService:
    """One active batch; its trials share the normal bounded run queue and history."""

    def __init__(self, runs: RunService):
        self.runs = runs
        self.root = runs.root / "batches"
        self.root.mkdir(exist_ok=True)
        self._lock = threading.RLock()
        self._records = {}
        self._events = {}
        self._closed = False
        self.recovery_warnings = []
        for path in sorted(self.root.glob("batch-*/batch.json")):
            if (
                not BATCH_ID.fullmatch(path.parent.name)
                or path.parent.resolve().parent != self.root
            ):
                continue
            try:
                record = json.loads(path.read_text(encoding="utf-8"))
                if record["id"] != path.parent.name or record["status"] not in ACTIVE | {
                    "completed",
                    "failed",
                    "cancelled",
                    "interrupted",
                }:
                    raise ValueError("invalid batch identity or status")
                if (
                    record["expected_trials"] != len(record["trials"])
                    or not 1 <= len(record["trials"]) <= 32
                ):
                    raise ValueError("invalid trial count")
                if record["status"] in ACTIVE:
                    for trial in record["trials"]:
                        if trial["status"] not in TERMINAL:
                            child = runs.get(trial["run_id"]) if trial["run_id"] else None
                            if child:
                                self._copy_result(trial, child)
                            else:
                                trial.update(
                                    status="interrupted",
                                    error="server stopped before trial submission",
                                )
                    record.update(
                        status="interrupted",
                        finished_utc=_stamp(),
                        error="server stopped before batch finished",
                    )
                    _write(path, record)
                self._records[record["id"]] = record
            except (OSError, ValueError, KeyError, TypeError, ServiceError):
                self.recovery_warnings.append(path.parent.name)
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="roboforge-batch")

    @staticmethod
    def _copy_result(trial, child):
        trial.update(
            status=child["status"],
            error=child["error"],
            metrics=child["metrics"] or {} if child["status"] in READY else {},
            navigation_outcome=child.get("navigation_outcome"),
        )

    def _path(self, batch_id):
        if not BATCH_ID.fullmatch(batch_id):
            raise ServiceError("unknown batch", 404)
        path = (self.root / batch_id).resolve()
        if path.parent != self.root:
            raise ServiceError("batch path leaves storage root", 404)
        return path

    def _save(self, record):
        _write(self._path(record["id"]) / "batch.json", record)

    def get(self, batch_id):
        self._path(batch_id)
        with self._lock:
            if batch_id not in self._records:
                raise ServiceError("unknown batch", 404)
            document = json.loads(json.dumps(self._records[batch_id]))
        terminal = [trial for trial in document["trials"] if trial["status"] in TERMINAL]
        document["finished_trials"] = len(terminal)
        document["groups"] = _summarize(terminal)
        return document

    def list(self, offset=0, limit=20):
        with self._lock:
            records = sorted(
                self._records.values(), key=lambda value: value["created_utc"], reverse=True
            )
            return {
                "total": len(records),
                "items": [
                    {
                        key: value[key]
                        for key in ("id", "name", "status", "created_utc", "expected_trials")
                    }
                    for value in records[offset : offset + limit]
                ],
            }

    def submit(self, specification: BatchRequest):
        preview, inputs, configs = prepare_batch(specification)
        with self._lock:
            if self._closed:
                raise ServiceError("batch service is shutting down", 503)
            if any(record["status"] in ACTIVE for record in self._records.values()):
                raise ServiceError("one experiment batch may run at a time", 429)
            batch_id = "batch-" + uuid.uuid4().hex
            path = self._path(batch_id)
            path.mkdir(exist_ok=False)
            spec = specification.model_dump(mode="json")
            record = {
                **preview,
                "id": batch_id,
                "name": specification.name,
                "status": "queued",
                "format_version": 1,
                "software_version": __version__,
                "created_utc": _stamp(),
                "finished_utc": None,
                "error": None,
                "cancel_requested": False,
                "specification_sha256": _digest(spec),
            }
            _write(path / "experiment.json", spec)
            _write(path / "inputs.json", inputs)
            self._save(record)
            self._records[batch_id] = record
            self._events[batch_id] = threading.Event()
            self._executor.submit(self._run, batch_id, configs)
            return self.get(batch_id)

    def _run(self, batch_id, configs):
        record, event = self._records[batch_id], self._events[batch_id]
        child_id = None
        try:
            with self._lock:
                record["status"] = "running"
                self._save(record)
            for trial, config in zip(record["trials"], configs):
                if config is None:
                    continue
                child_id = None
                while not event.is_set():
                    try:
                        with self._lock:
                            if event.is_set():
                                break
                            child = self.runs.submit(config)
                            child_id = child["id"]
                            trial.update(run_id=child_id, status=child["status"])
                            self._save(record)
                        break
                    except ServiceError as exc:
                        if exc.status != 429:
                            raise
                        event.wait(0.05)
                if child_id is None:
                    break
                while True:
                    if event.is_set():
                        self.runs.cancel(child_id)
                    child = self.runs.get(child_id)
                    if child["status"] not in ACTIVE:
                        break
                    # Use a separate wait after cancellation to avoid a busy loop.
                    threading.Event().wait(0.05)
                with self._lock:
                    self._copy_result(trial, child)
                    self._save(record)
                if event.is_set():
                    break
            with self._lock:
                for trial in record["trials"]:
                    if trial["status"] not in TERMINAL:
                        trial.update(
                            status="cancelled", error="batch cancelled before trial submission"
                        )
                record.update(
                    status="cancelled" if event.is_set() else "completed", finished_utc=_stamp()
                )
                self._save(record)
        except Exception as exc:
            if child_id:
                self.runs.cancel(child_id)
            with self._lock:
                for trial in record["trials"]:
                    if trial["status"] not in TERMINAL:
                        trial.update(status="interrupted", error="batch stopped after an error")
                record.update(
                    status="failed", error=f"{type(exc).__name__}: {exc}", finished_utc=_stamp()
                )
                self._save(record)
        finally:
            with self._lock:
                self._events.pop(batch_id, None)

    def cancel(self, batch_id):
        with self._lock:
            record = self.get(batch_id)
            if record["status"] in ACTIVE:
                self._events[batch_id].set()
                self._records[batch_id]["cancel_requested"] = True
                self._save(self._records[batch_id])
            return self.get(batch_id)

    def artifact(self, batch_id, filename):
        self.get(batch_id)
        if filename not in {"experiment.json", "inputs.json", "batch.json"}:
            raise ServiceError("unknown batch artifact", 404)
        path = (self._path(batch_id) / filename).resolve()
        if path.parent != self._path(batch_id) or not path.is_file():
            raise ServiceError("unknown batch artifact", 404)
        return path

    def close(self):
        with self._lock:
            self._closed = True
            for event in self._events.values():
                event.set()
        self._executor.shutdown(wait=True)
