import json
import threading

import pytest

from roboforge.comparison import ComparisonRequest, _differences, compare_runs
from roboforge.config import RunConfig
from roboforge.service import RunService, ServiceError
from roboforge.simulation import SimulationCancelled, Simulator
from tests.integration.test_service import config, finished


def test_comparison_retains_identity_deltas_and_reproducing_inputs(tmp_path, monkeypatch):
    service = RunService(tmp_path)
    try:
        first = finished(service, service.submit(config())["id"])
        changed = config().model_dump(mode="json")
        changed["commands"][0]["steps"] *= 2
        changed["seed"] += 1
        second = finished(service, service.submit(RunConfig.model_validate(changed))["id"])
        monkeypatch.setattr(
            Simulator, "run", lambda *a, **kw: pytest.fail("comparison reran a run")
        )
        document = compare_runs(service, ComparisonRequest(run_ids=[first["id"], second["id"]]))
        assert document["baseline_id"] == first["id"]
        assert document["runs"][1]["config"] == changed
        assert document["runs"][0]["config_sha256"] == first["config_sha256"]
        duration = next(row for row in document["metrics"] if row["name"] == "duration_s")
        assert duration["values"] == pytest.approx([0.1, 0.2])
        assert duration["deltas"] == pytest.approx([0, 0.1])
        assert {row["path"] for row in document["config_differences"]} == {"commands", "seed"}
        assert not document["same_setup"] and document["same_software"]
        reverse = compare_runs(service, ComparisonRequest(run_ids=[second["id"], first["id"]]))
        assert next(row for row in reverse["metrics"] if row["name"] == "duration_s")["deltas"][
            1
        ] == pytest.approx(-0.1)
        json.dumps(document, allow_nan=False)
    finally:
        service.close()


def test_failure_is_not_zero_and_unavailable_baseline_has_no_deltas(tmp_path, monkeypatch):
    service = RunService(tmp_path)
    try:
        success = finished(service, service.submit(config())["id"])

        def broken(*args, **kwargs):
            raise RuntimeError("deliberate failure")

        monkeypatch.setattr(Simulator, "run", broken)
        failed = finished(service, service.submit(config())["id"])
        document = compare_runs(service, ComparisonRequest(run_ids=[failed["id"], success["id"]]))
        assert document["runs"][0]["status"] == "failed"
        assert "deliberate failure" in document["runs"][0]["error"]
        assert document["runs"][0]["metrics"] is None
        assert document["same_setup"] and not document["config_differences"]
        assert all(
            row["values"][0] is None and row["deltas"] == [None, None]
            for row in document["metrics"]
        )
    finally:
        service.close()


@pytest.mark.parametrize("filename", ["config.json", "metrics.json", "job.json"])
def test_comparison_rejects_modified_artifacts_after_restart(tmp_path, filename):
    service = RunService(tmp_path)
    ids = [finished(service, service.submit(config())["id"])["id"] for _ in range(2)]
    service.close()
    path = tmp_path / ids[0] / filename
    document = json.loads(path.read_text())
    document["seed" if filename != "metrics.json" else "duration_s"] = 999
    path.write_text(json.dumps(document))
    restored = RunService(tmp_path)
    try:
        with pytest.raises(ServiceError, match="integrity"):
            compare_runs(restored, ComparisonRequest(run_ids=ids))
    finally:
        restored.close()


def test_comparison_endpoint_bounds_active_runs_and_local_origin(tmp_path, monkeypatch):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from roboforge.api import create_app

    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.service
        first = finished(service, service.submit(config())["id"])["id"]
        second = finished(service, service.submit(config())["id"])["id"]
        assert client.post("/api/comparisons", json={"run_ids": [first, second]}).status_code == 200
        for ids in ([first], [first, first], [first] * 5):
            assert client.post("/api/comparisons", json={"run_ids": ids}).status_code == 422
        assert (
            client.post("/api/comparisons", json={"run_ids": [first, "../bad"]}).status_code == 404
        )
        assert (
            client.post(
                "/api/comparisons",
                json={"run_ids": [first, second]},
                headers={"Origin": "https://other.example"},
            ).status_code
            == 403
        )
        started, release = threading.Event(), threading.Event()

        def blocked(self, should_cancel):
            started.set()
            release.wait(5)
            raise SimulationCancelled()

        monkeypatch.setattr(Simulator, "run", blocked)
        running = service.submit(config())["id"]
        try:
            assert started.wait(2)
            response = client.post("/api/comparisons", json={"run_ids": [first, running]})
            assert response.status_code == 409
        finally:
            release.set()
        assert finished(service, running)["status"] == "cancelled"
        assert (
            client.post("/api/comparisons", json={"run_ids": [first, running]}).json()["runs"][1][
                "metrics"
            ]
            is None
        )


def test_absent_config_values_are_distinct_from_null():
    rows = _differences([{"navigation": None}, {}])
    assert rows == [
        {
            "path": "navigation",
            "values": [{"present": True, "value": None}, {"present": False, "value": None}],
        }
    ]
