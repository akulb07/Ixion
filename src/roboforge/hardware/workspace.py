"""Durable, bounded execution of built-in rover suites for the local workspace."""

import hashlib
import json
import re
import threading
import uuid
from concurrent.futures import ThreadPoolExecutor
from datetime import UTC, datetime
from importlib.resources import files
from pathlib import Path

import yaml

from roboforge.config import Schema
from roboforge.service import ServiceError, _StorageLock

from .artifact_io import decode_json_object
from .project import RobotProject
from .rover_suite import RoverSuiteConfig, run_rover_suite

JOB_ID = re.compile(r"^hardware-[0-9a-f]{32}$")
MAX_REPORT_BYTES = 64 * 1024 * 1024
TEMPLATES = {
    "feedback": ("Encoder feedback", "rover_feedback_suite.yaml"),
    "power": ("Battery charge and load", "rover_power_suite.yaml"),
    "voltage": ("Voltage reading delays", "rover_voltage_feedback_suite.yaml"),
    "sensor-errors": ("Voltage sensor errors", "rover_voltage_error_suite.yaml"),
    "power-response": ("Voltage-limit response", "rover_power_response_suite.yaml"),
}


class HardwareRunRequest(Schema):
    project: RobotProject
    suite: RoverSuiteConfig


def hardware_template(template_id: str) -> dict:
    if template_id not in TEMPLATES:
        raise ServiceError("hardware template not found", 404)
    resources = files("roboforge").joinpath("hardware/templates/esp32_rover")
    project = RobotProject.model_validate(
        yaml.safe_load(resources.joinpath("robot.yaml").read_text(encoding="utf-8"))
    )
    suite = RoverSuiteConfig.model_validate(
        yaml.safe_load(resources.joinpath(TEMPLATES[template_id][1]).read_text(encoding="utf-8"))
    )
    return HardwareRunRequest(project=project, suite=suite).model_dump(mode="json")


def _bytes(value):
    return (json.dumps(value, indent=2, allow_nan=False) + "\n").encode("utf-8")


def _write(path, content):
    pending = path.with_suffix(".pending")
    pending.write_bytes(content)
    pending.replace(path)


def _stamp():
    return datetime.now(UTC).isoformat()


class HardwareService:
    def __init__(self, directory):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        self._storage_lock = _StorageLock(self.root)
        self._mutex = threading.RLock()
        self._jobs = {}
        self._closed = False
        self.recovery_warnings = []
        try:
            self._recover()
            self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="ixion-hardware")
        except BaseException:
            self._storage_lock.close()
            raise

    def _recover(self):
        for folder in sorted(self.root.glob("hardware-*"))[:200]:
            if not JOB_ID.fullmatch(folder.name) or not folder.is_dir():
                continue
            try:
                job = self._read(folder / "job.json", 65_536)
                if job["id"] != folder.name or job["status"] not in {
                    "queued",
                    "running",
                    "completed",
                    "failed",
                }:
                    raise ValueError("invalid hardware job identity or status")
                if not all(
                    isinstance(job.get(key), str) for key in ("name", "project_name", "created_at")
                ):
                    raise ValueError("invalid hardware job metadata")
                request = self._read(folder / "request.json", 1_000_000)
                HardwareRunRequest.model_validate(request)
                if job["status"] in {"queued", "running"}:
                    job.update(
                        status="failed",
                        error="Service stopped before this run completed.",
                        finished_at=_stamp(),
                    )
                    _write(folder / "job.json", _bytes(job))
                elif job["status"] == "completed":
                    self._report_bytes(job)
                self._jobs[job["id"]] = job
            except (OSError, ValueError, KeyError, TypeError) as exc:
                self.recovery_warnings.append(f"Could not recover {folder.name}: {exc}")

    @staticmethod
    def _read(path, limit):
        with path.open("rb") as stream:
            raw = stream.read(limit + 1)
        return decode_json_object(raw, maximum_bytes=limit, label="hardware document")

    def _save(self, job):
        _write(self.root / job["id"] / "job.json", _bytes(job))

    def submit(self, request: HardwareRunRequest):
        specification = _bytes(request.model_dump(mode="json"))
        if len(specification) > 1_000_000:
            raise ServiceError("hardware setup exceeds 1 MB", 422)
        with self._mutex:
            if self._closed:
                raise ServiceError("hardware service is closed")
            if len(self._jobs) >= 200:
                raise ServiceError("hardware history limit reached (200 runs)")
            if sum(j["status"] in {"queued", "running"} for j in self._jobs.values()) >= 4:
                raise ServiceError("hardware run queue is full")
            job = {
                "id": "hardware-" + uuid.uuid4().hex,
                "name": request.suite.name,
                "project_name": request.project.name,
                "status": "queued",
                "created_at": _stamp(),
                "finished_at": None,
                "summary": None,
                "outcome": None,
                "error": None,
            }
            folder = self.root / job["id"]
            folder.mkdir()
            _write(folder / "request.json", specification)
            self._save(job)
            self._jobs[job["id"]] = job
            self._executor.submit(self._run, job["id"], request)
            return dict(job)

    def _run(self, job_id, request):
        with self._mutex:
            job = self._jobs[job_id]
        try:
            with self._mutex:
                job.update(status="running")
                self._save(job)
            report = run_rover_suite(request.project, request.suite)
            raw = _bytes(report)
            if len(raw) > MAX_REPORT_BYTES:
                raise ValueError("hardware report exceeds 64 MiB")
            _write(self.root / job_id / "report.json", raw)
            with self._mutex:
                job.update(
                    status="completed",
                    outcome=report["status"],
                    summary=report["summary"],
                    report_sha256=hashlib.sha256(raw).hexdigest(),
                    finished_at=_stamp(),
                )
                self._save(job)
        except Exception as exc:
            with self._mutex:
                job.update(status="failed", error=str(exc), finished_at=_stamp())
                self._save(job)

    def get(self, job_id):
        with self._mutex:
            if not JOB_ID.fullmatch(job_id) or job_id not in self._jobs:
                raise ServiceError("hardware run not found", 404)
            return json.loads(json.dumps(self._jobs[job_id]))

    def list(self, offset=0, limit=20):
        with self._mutex:
            jobs = sorted(self._jobs.values(), key=lambda j: j["created_at"], reverse=True)
            return {
                "total": len(jobs),
                "items": json.loads(json.dumps(jobs[offset : offset + limit])),
            }

    def _report_bytes(self, job):
        path = self.root / job["id"] / "report.json"
        with path.open("rb") as stream:
            raw = stream.read(MAX_REPORT_BYTES + 1)
        if len(raw) > MAX_REPORT_BYTES or hashlib.sha256(raw).hexdigest() != job.get(
            "report_sha256"
        ):
            raise ValueError("hardware report integrity check failed")
        return raw

    def report(self, job_id):
        job = self.get(job_id)
        if job["status"] != "completed":
            raise ServiceError("hardware run has no completed report")
        try:
            raw = self._report_bytes(job)
            return decode_json_object(
                raw, maximum_bytes=MAX_REPORT_BYTES, label="hardware report"
            ), job["report_sha256"]
        except (OSError, ValueError) as exc:
            raise ServiceError(str(exc)) from exc

    def specification(self, job_id):
        self.get(job_id)
        try:
            return HardwareRunRequest.model_validate(
                self._read(self.root / job_id / "request.json", 1_000_000)
            ).model_dump(mode="json")
        except (OSError, ValueError) as exc:
            raise ServiceError(str(exc)) from exc

    def close(self):
        with self._mutex:
            self._closed = True
        try:
            self._executor.shutdown(wait=True)
        finally:
            self._storage_lock.close()
