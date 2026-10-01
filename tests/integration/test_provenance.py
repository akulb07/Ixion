import json
import subprocess
import zipfile

from roboforge.bundles import check_bundle, export_bundle
from roboforge.provenance import capture_provenance, source_identity
from roboforge.regression import RegressionRequest, check_saved_runs
from roboforge.service import RunService
from tests.integration.test_regression import policy
from tests.integration.test_service import config, finished


def test_source_fingerprint_detects_edits_and_ignores_cache_files(tmp_path):
    package = tmp_path / "package"
    package.mkdir()
    source = package / "model.py"
    source.write_text("value = 1")
    first = source_identity(package)
    (package / "cache.pyc").write_bytes(b"cache")
    assert source_identity(package)["sha256"] == first["sha256"]
    source.write_text("value = 2")
    assert source_identity(package)["sha256"] != first["sha256"]
    assert first["git_revision"] is None


def test_git_unavailable_is_explicit_and_does_not_block_fingerprint(tmp_path, monkeypatch):
    (tmp_path / ".git").mkdir()
    package = tmp_path / "src" / "roboforge"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("")

    def unavailable(*args, **kwargs):
        raise subprocess.TimeoutExpired("git", 2)

    monkeypatch.setattr(subprocess, "run", unavailable)
    source = source_identity(package)
    assert source["sha256"] and source["git_revision"] is None
    assert source["notes"]


def test_execution_provenance_survives_bundling_and_gates_changed_code(tmp_path, monkeypatch):
    import roboforge.provenance as provenance

    snapshot = capture_provenance()
    assert snapshot["dependencies"]["numpy"]
    assert "C:" not in json.dumps(snapshot) and "environment" not in snapshot
    monkeypatch.setattr(provenance, "capture_provenance", lambda: json.loads(json.dumps(snapshot)))
    service = RunService(tmp_path / "store")
    try:
        first = finished(service, service.submit(config())["id"])
        snapshot["source"]["sha256"] = "a" * 64
        second = finished(service, service.submit(config())["id"])
        assert first["provenance"]["source"]["sha256"] != second["provenance"]["source"]["sha256"]
        request = RegressionRequest(
            baseline_id=first["id"], candidate_id=second["id"], policy=policy()
        )
        assert check_saved_runs(service.root, request)["status"] == "inconclusive"
        request = request.model_copy(update={"policy": policy(allow_software_change=True)})
        archive = tmp_path / "check.zip"
        report = export_bundle(service.root, request, archive)
        assert report["status"] == "pass"
        with zipfile.ZipFile(archive) as z:
            job = json.loads(z.read(f"runs/{first['id']}/job.json"))
            assert job["provenance"] == first["provenance"]
        assert check_bundle(archive)["checks"] == report["checks"]
    finally:
        service.close()


def test_failed_execution_still_records_provenance(tmp_path, monkeypatch):
    from roboforge.simulation import Simulator

    def broken(*args, **kwargs):
        raise RuntimeError("test failure")

    monkeypatch.setattr(Simulator, "run", broken)
    service = RunService(tmp_path)
    try:
        run = finished(service, service.submit(config())["id"])
        assert run["status"] == "failed"
        assert run["provenance"]["capture_stage"] == "service worker before simulation"
        job = json.loads(service.artifact(run["id"], "job.json").read_text())
        assert job["provenance"] == run["provenance"]
    finally:
        service.close()


def test_legacy_runs_remain_checkable_without_inventing_provenance(tmp_path):
    from roboforge.experiments import write_manifest

    service = RunService(tmp_path / "store")
    try:
        runs = [finished(service, service.submit(config())["id"]) for _ in range(2)]
    finally:
        service.close()
    for run in runs:
        directory = tmp_path / "store" / run["id"]
        job = json.loads((directory / "job.json").read_text())
        del job["provenance"]
        (directory / "job.json").write_text(json.dumps(job))
        write_manifest(directory)
    spec = RegressionRequest(baseline_id=runs[0]["id"], candidate_id=runs[1]["id"], policy=policy())
    report = check_saved_runs(tmp_path / "store", spec)
    assert report["status"] == "pass"
    assert report["comparison"]["same_source"] is None
    assert report["comparison"]["same_dependencies"] is None
    assert "unavailable" in report["checks"][1]["provenance_note"]
    archive = tmp_path / "legacy.zip"
    export_bundle(tmp_path / "store", spec, archive)
    checked = check_bundle(archive)
    assert all("provenance" not in run for run in checked["comparison"]["runs"])
