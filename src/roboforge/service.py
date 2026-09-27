"""Bounded local run service; transport-independent and durable on disk."""

import hashlib
import json
import math
import os
import re
import threading
import time
import uuid
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from datetime import UTC, datetime
from pathlib import Path

from roboforge import __version__
from roboforge.config import LidarConfig, RunConfig
from roboforge.experiments import simulation_metrics, write_manifest
from roboforge.io import save_result
from roboforge.replay import ReplayLog
from roboforge.simulation import SimulationCancelled, Simulator

READY = {"completed", "collision", "budget_exceeded"}
ACTIVE = {"queued", "running"}
RUN_ID = re.compile(r"^run-[0-9a-f]{32}$")


class ServiceError(Exception):
    def __init__(self, message: str, status: int = 409):
        super().__init__(message)
        self.status = status


def resource_estimate(config: RunConfig) -> dict:
    steps = config.step_budget
    duration = steps * config.simulation.dt
    if config.navigation and steps * len(config.navigation.path) > 2_000_000:
        raise ServiceError("navigation tracking workload exceeds 2,000,000 waypoint steps", 422)
    if steps > 50000 or duration > 300:
        raise ServiceError(
            "local API runs are limited to 50,000 steps and 300 simulated seconds", 422
        )
    if len(config.sensors) > 32 or len(config.faults) > 64:
        raise ServiceError("local API runs are limited to 32 sensors and 64 faults", 422)
    if any(duration * s.rate_hz > 100000 for s in config.sensors):
        raise ServiceError("sensor rate exceeds local API workload budget", 422)
    reads = sum(math.floor(duration * s.rate_hz) + 1 for s in config.sensors)
    rays = sum(
        (math.floor(duration * s.rate_hz) + 1) * s.rays
        for s in config.sensors
        if isinstance(s, LidarConfig)
    )
    if reads > 100000 or rays > 1000000:
        raise ServiceError("sensor workload exceeds 100,000 readings or 1,000,000 ray casts", 422)
    if (
        len(config.environment.obstacles) > 500
        or steps * max(1, len(config.environment.obstacles)) > 2000000
    ):
        raise ServiceError("world workload exceeds the local API budget", 422)
    if config.simulation.collision.max_queries > 100000:
        raise ServiceError("collision query budget exceeds 100,000 per step", 422)
    return {"steps": steps, "duration_s": duration, "sensor_readings": reads, "lidar_rays": rays}


def _stamp():
    return datetime.now(UTC).isoformat()


def _write(path: Path, document):
    pending = path.with_suffix(".pending.json")
    pending.write_text(json.dumps(document, indent=2, allow_nan=False), encoding="utf-8")
    pending.replace(path)


class _StorageLock:
    """An OS-held lock releases even when the owning process crashes."""

    def __init__(self, root):
        self.stream = (root / ".service.lock").open("a+b")
        self.stream.seek(0, 2)
        if self.stream.tell() == 0:
            self.stream.write(b"0")
            self.stream.flush()
        self.stream.seek(0)
        try:
            if os.name == "nt":
                import msvcrt

                msvcrt.locking(self.stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl

                fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self.stream.close()
            raise ServiceError("another service owns this storage directory") from exc

    def close(self):
        self.stream.close()


class RunService:
    """Single worker, at most eight pending runs, one process per storage directory.

    Results are published only after exports and integrity manifests are complete.
    Cancellation retains configuration/status, not a partial physical trajectory.
    """

    def __init__(self, directory: str | Path, capacity: int = 8):
        if isinstance(capacity, bool) or not isinstance(capacity, int) or not 1 <= capacity <= 8:
            raise ValueError("queue capacity must be between one and eight")
        self.root = Path(directory).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._storage_lock = _StorageLock(self.root)
        self.capacity = capacity
        self._lock = threading.RLock()
        self._events: dict[str, threading.Event] = {}
        self._records: dict[str, dict] = {}
        self._cache: OrderedDict[str, ReplayLog] = OrderedDict()
        self._closed = False
        self.recovery_warnings: list[str] = []
        for path in sorted(self.root.glob("run-*/job.json")):
            if not RUN_ID.fullmatch(path.parent.name) or path.parent.resolve().parent != self.root:
                continue
            try:
                record = json.loads(path.read_text(encoding="utf-8"))
                if record["id"] != path.parent.name or record["status"] not in ACTIVE | READY | {
                    "failed",
                    "cancelled",
                    "interrupted",
                }:
                    raise ValueError("invalid job identity or status")
                if not isinstance(record["created_utc"], str):
                    raise ValueError("invalid creation timestamp")
                datetime.fromisoformat(record["created_utc"])
                if record["status"] in ACTIVE:
                    record.update(
                        status="interrupted",
                        finished_utc=_stamp(),
                        error="server stopped before this run finished",
                    )
                    _write(path, record)
                self._records[record["id"]] = record
            except (OSError, ValueError, KeyError, TypeError):
                self.recovery_warnings.append(path.parent.name)
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="roboforge-run")

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for event in self._events.values():
                event.set()
        try:
            self._executor.shutdown(wait=True)
        finally:
            self._storage_lock.close()

    def _path(self, run_id: str) -> Path:
        if not RUN_ID.fullmatch(run_id):
            raise ServiceError("unknown run", 404)
        path = (self.root / run_id).resolve()
        if path.parent != self.root:
            raise ServiceError("run path leaves storage root", 404)
        return path

    def get(self, run_id):
        self._path(run_id)
        with self._lock:
            if run_id not in self._records:
                raise ServiceError("unknown run", 404)
            return json.loads(json.dumps(self._records[run_id]))

    def list(self, offset=0, limit=50):
        with self._lock:
            values = sorted(self._records.values(), key=lambda r: r["created_utc"], reverse=True)
            return {
                "total": len(values),
                "items": json.loads(json.dumps(values[offset : offset + limit])),
            }

    def submit(self, config: RunConfig):
        budget = resource_estimate(config)
        with self._lock:
            if self._closed:
                raise ServiceError("service is shutting down", 503)
            if sum(r["status"] in ACTIVE for r in self._records.values()) >= self.capacity:
                raise ServiceError("run queue is full", 429)
            run_id = "run-" + uuid.uuid4().hex
            path = self._path(run_id)
            path.mkdir(exist_ok=False)
            document = config.model_dump(mode="json")
            digest = hashlib.sha256(
                json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            record = {
                "id": run_id,
                "name": config.name,
                "status": "queued",
                "created_utc": _stamp(),
                "started_utc": None,
                "finished_utc": None,
                "software_version": __version__,
                "config_sha256": digest,
                "seed": config.seed,
                "budget": budget,
                "metrics": None,
                "error": None,
                "wall_runtime_s": None,
                "cancel_requested": False,
            }
            _write(path / "config.json", document)
            _write(path / "job.json", record)
            self._records[run_id] = record
            self._events[run_id] = threading.Event()
            self._executor.submit(self._run, run_id, config)
            return self.get(run_id)

    def _run(self, run_id, config):
        started = time.perf_counter()
        path = self._path(run_id)
        with self._lock:
            record = self._records[run_id]
            event = self._events[run_id]
        status, error, metrics = "failed", None, None
        try:
            with self._lock:
                record.update(status="running", started_utc=_stamp())
                _write(path / "job.json", record)
            result = Simulator(config).run(should_cancel=event.is_set)
            if event.is_set():
                raise SimulationCancelled("run cancelled before export")
            save_result(result, path)
            metrics = simulation_metrics(result)
            _write(path / "metrics.json", metrics)
            status = result.status
            with self._lock:
                record["navigation_outcome"] = result.navigation_outcome
        except SimulationCancelled:
            status = "cancelled"
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
        with self._lock:
            if event.is_set() and status in READY:
                status, metrics = "cancelled", None
            record.update(
                status=status,
                error=error,
                metrics=metrics,
                finished_utc=_stamp(),
                wall_runtime_s=time.perf_counter() - started,
            )
            try:
                _write(path / "job.json", record)
                if status in READY:
                    write_manifest(path)
            except OSError as exc:
                record.update(status="failed", error=f"export failed: {exc}")
                try:
                    _write(path / "job.json", record)
                except OSError:
                    pass
            self._events.pop(run_id, None)

    def cancel(self, run_id):
        with self._lock:
            record = self.get(run_id)
            if record["status"] in ACTIVE:
                self._events[run_id].set()
                self._records[run_id]["cancel_requested"] = True
                _write(self._path(run_id) / "job.json", self._records[run_id])
            return self.get(run_id)

    def config(self, run_id):
        self.get(run_id)
        return RunConfig.model_validate_json(
            (self._path(run_id) / "config.json").read_text(encoding="utf-8")
        )

    def replay(self, run_id):
        with self._lock:
            if self.get(run_id)["status"] not in READY:
                raise ServiceError("run has no completed replay")
            if run_id not in self._cache:
                try:
                    replay = ReplayLog(self._path(run_id))
                except (OSError, ValueError) as exc:
                    raise ServiceError(
                        "recorded replay is missing or failed integrity validation"
                    ) from exc
                self._cache[run_id] = replay
                if len(self._cache) > 2:
                    self._cache.popitem(last=False)
            self._cache.move_to_end(run_id)
            return self._cache[run_id]

    def artifact(self, run_id, filename):
        record = self.get(run_id)
        allowed = {"config.json", "job.json"}
        if record["status"] in READY:
            allowed |= {
                "metadata.json",
                "metrics.json",
                "trajectory.csv",
                "sensors.jsonl",
                "actuators.json",
                "control.json",
                "faults.json",
                "collisions.json",
                "motion_segments.json",
                "navigation.json",
                "manifest.json",
            }
        if filename not in allowed:
            raise ServiceError("unknown artifact", 404)
        path = (self._path(run_id) / filename).resolve()
        if path.parent != self._path(run_id) or not path.is_file():
            raise ServiceError("unknown artifact", 404)
        return path
