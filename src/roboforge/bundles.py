"""Portable recorded-run checks. Bundles contain data, never executable hooks."""

import hashlib
import importlib.metadata
import json
import platform
import tempfile
import zipfile
from pathlib import Path

from roboforge import __version__
from roboforge.regression import RegressionRequest, SavedRuns, check_saved_runs
from roboforge.service import READY

MAX_BYTES = 128 * 1024 * 1024
ARTIFACTS = {
    "config.json",
    "job.json",
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
}


def _json(value):
    return json.dumps(value, indent=2, allow_nan=False).encode("utf-8")


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def export_bundle(store, specification: RegressionRequest, output):
    """Publish a new ZIP only after validating every selected run artifact."""
    output = Path(output).resolve()
    if output.is_relative_to(Path(store).resolve()):
        raise ValueError("bundle output must be outside the run store")
    report = check_saved_runs(store, specification)
    reader = SavedRuns(store)
    files = {
        "request.json": _json(specification.model_dump(mode="json")),
        "report.json": _json(report),
    }
    for run in report["comparison"]["runs"]:
        run_id = run["id"]
        expected = {}
        names = {"config.json", "job.json"}
        if run["status"] in READY:
            manifest_bytes = reader.artifact(run_id, "manifest.json").read_bytes()
            manifest = json.loads(manifest_bytes)
            if manifest.get("format_version") != 1 or not isinstance(manifest.get("files"), dict):
                raise ValueError("unsupported run manifest")
            expected = manifest["files"]
            if not {"config.json", "job.json", "metrics.json"}.issubset(expected):
                raise ValueError("incomplete run manifest")
            if not set(expected).issubset(ARTIFACTS):
                raise ValueError("run manifest contains unsupported artifacts")
            names |= set(expected)
            files[f"runs/{run_id}/manifest.json"] = manifest_bytes
        for name in sorted(names):
            path = reader.artifact(run_id, name)
            if path.stat().st_size + sum(map(len, files.values())) > MAX_BYTES:
                raise ValueError("bundle exceeds 128 MiB uncompressed limit")
            data = path.read_bytes()
            if name in expected and _digest(data) != expected[name]:
                raise ValueError(f"artifact integrity error: {run_id}/{name}")
            files[f"runs/{run_id}/{name}"] = data
        job = json.loads(files[f"runs/{run_id}/job.json"])
        if run["metrics"] is None:
            job["metrics"] = None
        if job != {key: value for key, value in run.items() if key != "config"}:
            raise ValueError("run changed during bundle export")
    environment = {
        "scope": "export environment, not necessarily the original execution environment",
        "roboforge": __version__,
        "python": platform.python_version(),
        "platform": platform.system(),
        "dependencies": {
            name: importlib.metadata.version(name) for name in ("numpy", "pydantic", "PyYAML")
        },
        "original_source_revision": None,
    }
    files["export-environment.json"] = _json(environment)
    files["README.txt"] = (
        "Ixion recorded-check bundle\n\n"
        "Run: roboforge bundle-check this-file.zip\n"
        "This verifies and repeats the acceptance check; it does not rerun the robot.\n"
        "Run artifacts are under runs/. New service jobs carry execution provenance in job.json; older jobs may not.\n"
        "Hashes detect corruption, not malicious replacement of files and their manifest.\n"
    ).encode()
    files["bundle-manifest.json"] = _json(
        {"format_version": 1, "files": {name: _digest(data) for name, data in files.items()}}
    )
    if sum(map(len, files.values())) > MAX_BYTES:
        raise ValueError("bundle exceeds 128 MiB uncompressed limit")
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation preserves earlier evidence. Remove only this newly created
    # output if writing fails, never an existing package.
    with output.open("xb") as stream:
        try:
            with zipfile.ZipFile(stream, "w", zipfile.ZIP_DEFLATED) as archive:
                for name, data in files.items():
                    archive.writestr(name, data)
        except BaseException:
            stream.close()
            output.unlink()
            raise
    return report


def check_bundle(path):
    """Validate a bounded archive before writing only allowlisted paths to a temp store."""
    try:
        with zipfile.ZipFile(path) as archive:
            entries = archive.infolist()
            if len(entries) > 64 or sum(e.file_size for e in entries) > MAX_BYTES:
                raise ValueError("bundle exceeds archive limits")
            if len({e.filename for e in entries}) != len(entries):
                raise ValueError("duplicate bundle member")
            files = {entry.filename: archive.read(entry) for entry in entries}
        manifest = json.loads(files.pop("bundle-manifest.json"))
        if manifest.get("format_version") != 1 or set(manifest["files"]) != set(files):
            raise ValueError("bundle manifest does not match archive")
        if any(_digest(data) != manifest["files"][name] for name, data in files.items()):
            raise ValueError("bundle integrity check failed")
        spec = RegressionRequest.model_validate_json(files["request.json"])
        from roboforge.service import RUN_ID

        ids = (spec.baseline_id, spec.candidate_id)
        if any(not RUN_ID.fullmatch(run_id) for run_id in ids) or ids[0] == ids[1]:
            raise ValueError("invalid bundle run IDs")
        allowed = {"request.json", "report.json", "export-environment.json", "README.txt"}
        allowed |= {
            f"runs/{run_id}/{name}" for run_id in ids for name in ARTIFACTS | {"manifest.json"}
        }
        if not set(files).issubset(allowed):
            raise ValueError("unsupported bundle member")
        for run_id in ids:
            prefix = f"runs/{run_id}/"
            if prefix + "manifest.json" in files:
                inner = json.loads(files[prefix + "manifest.json"])
                if inner.get("format_version") != 1 or not set(inner["files"]).issubset(ARTIFACTS):
                    raise ValueError("invalid run manifest in bundle")
                for name, digest in inner["files"].items():
                    if _digest(files[prefix + name]) != digest:
                        raise ValueError("run artifact integrity check failed")
        saved = json.loads(files["report.json"])
        with tempfile.TemporaryDirectory(prefix="roboforge-check-") as temporary:
            root = Path(temporary)
            for name, data in files.items():
                if name.startswith("runs/"):
                    destination = root / name
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    destination.write_bytes(data)
            report = check_saved_runs(root / "runs", spec)
        if any(report[key] != saved[key] for key in ("status", "checks", "policy_sha256")):
            raise ValueError("recorded check differs from this version's evaluation")
        return report
    except (KeyError, TypeError, AttributeError, zipfile.BadZipFile, RuntimeError) as exc:
        raise ValueError(f"invalid check bundle: {exc}") from exc
