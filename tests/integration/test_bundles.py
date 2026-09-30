import hashlib
import json
import zipfile

import pytest

from roboforge.bundles import check_bundle, export_bundle
from roboforge.cli import main
from roboforge.regression import RegressionRequest
from roboforge.service import RunService
from tests.integration.test_regression import policy
from tests.integration.test_service import config, finished


def make_bundle(tmp_path, design=None):
    store = tmp_path / "store"
    with_service = RunService(store)
    try:
        a = finished(with_service, with_service.submit(config())["id"])
        b = finished(with_service, with_service.submit(config())["id"])
        spec = RegressionRequest(
            baseline_id=a["id"], candidate_id=b["id"], policy=design or policy()
        )
        archive = tmp_path / "check.zip"
        report = export_bundle(store, spec, archive)
    finally:
        with_service.close()
    return store, spec, archive, report


def rewrite(source, target, transform, rehash=False):
    with zipfile.ZipFile(source) as z:
        files = {name: z.read(name) for name in z.namelist()}
    transform(files)
    if rehash:
        files["bundle-manifest.json"] = json.dumps(
            {
                "format_version": 1,
                "files": {
                    key: hashlib.sha256(value).hexdigest()
                    for key, value in files.items()
                    if key != "bundle-manifest.json"
                },
            }
        ).encode()
    with zipfile.ZipFile(target, "w") as z:
        for name, data in files.items():
            z.writestr(name, data)


def test_portable_check_survives_moving_store_and_preserves_evidence(tmp_path):
    store, spec, archive, report = make_bundle(tmp_path)
    original = archive.read_bytes()
    with pytest.raises(FileExistsError):
        export_bundle(store, spec, archive)
    assert archive.read_bytes() == original
    store.rename(tmp_path / "moved-store")
    checked = check_bundle(archive)
    assert checked["checks"] == report["checks"] and checked["status"] == "pass"
    assert main(["bundle-check", str(archive)]) == 0
    with zipfile.ZipFile(archive) as z:
        environment = json.loads(z.read("export-environment.json"))
        assert environment["original_source_revision"] is None
        assert environment["dependencies"]["numpy"]
        assert any(name.endswith("trajectory.csv") for name in z.namelist())


def test_bundle_failure_and_inconclusive_exit_codes(tmp_path):
    for status, metric, maximum, expected in [
        ("failure", "duration_s", -1, 3),
        ("missing", "missing_metric", 0, 4),
    ]:
        folder = tmp_path / status
        folder.mkdir()
        design = policy(rules=[{"name": "requirement", "metric": metric, "maximum": maximum}])
        store, spec, archive, _ = make_bundle(folder, design)
        assert main(["bundle-check", str(archive)]) == expected
        policy_path = folder / "policy.json"
        policy_path.write_text(design.model_dump_json())
        assert (
            main(
                [
                    "bundle",
                    str(policy_path),
                    "--store",
                    str(store),
                    "--baseline",
                    spec.baseline_id,
                    "--candidate",
                    spec.candidate_id,
                    "--output",
                    str(folder / "cli.zip"),
                ]
            )
            == 0
        )
        assert main(["bundle-check", str(folder / "cli.zip")]) == expected


@pytest.mark.parametrize(
    "change,rehash",
    [
        ("corrupt", False),
        ("traversal", True),
        ("missing_telemetry", True),
        ("false_result", True),
    ],
)
def test_invalid_archives_rejected_before_rechecking(tmp_path, change, rehash):
    _, _, archive, _ = make_bundle(tmp_path)

    def mutate(files):
        if change == "corrupt":
            files["README.txt"] += b"altered"
        elif change == "traversal":
            files["../escaped.txt"] = b"bad"
        elif change == "missing_telemetry":
            del files[next(name for name in files if name.endswith("sensors.jsonl"))]
        else:
            report = json.loads(files["report.json"])
            report["status"] = "fail"
            files["report.json"] = json.dumps(report).encode()

    bad = tmp_path / "bad.zip"
    rewrite(archive, bad, mutate, rehash)
    with pytest.raises(ValueError):
        check_bundle(bad)
    assert not (tmp_path / "escaped.txt").exists()
    assert main(["bundle-check", str(bad)]) == 2


def test_failed_run_bundles_retain_errors_without_fabricating_telemetry(tmp_path, monkeypatch):
    from roboforge.simulation import Simulator

    store = tmp_path / "store"
    service = RunService(store)
    try:
        a = finished(service, service.submit(config())["id"])

        def fail(*args, **kwargs):
            raise RuntimeError("controller unavailable")

        monkeypatch.setattr(Simulator, "run", fail)
        b = finished(service, service.submit(config())["id"])
        spec = RegressionRequest(baseline_id=a["id"], candidate_id=b["id"], policy=policy())
        archive = tmp_path / "failed.zip"
        export_bundle(store, spec, archive)
        result = check_bundle(archive)
        assert result["status"] == "fail"
        assert "controller unavailable" in result["comparison"]["runs"][1]["error"]
        with zipfile.ZipFile(archive) as z:
            assert f"runs/{b['id']}/trajectory.csv" not in z.namelist()
    finally:
        service.close()


def test_duplicate_members_and_size_budget_rejected(tmp_path, monkeypatch):
    import roboforge.bundles as bundles

    _, _, archive, _ = make_bundle(tmp_path)
    duplicate = tmp_path / "duplicate.zip"
    duplicate.write_bytes(archive.read_bytes())
    with zipfile.ZipFile(duplicate, "a") as z:
        with pytest.warns(UserWarning, match="Duplicate name"):
            z.writestr("README.txt", b"duplicate")
    with pytest.raises(ValueError, match="duplicate"):
        check_bundle(duplicate)
    monkeypatch.setattr(bundles, "MAX_BYTES", 10)
    with pytest.raises(ValueError, match="limits"):
        check_bundle(archive)
