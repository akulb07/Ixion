import json
import threading
import time

import pytest

from roboforge.batch_service import BatchRequest, BatchService, prepare_batch
from roboforge.service import ACTIVE, RunService, ServiceError
from roboforge.simulation import SimulationCancelled, Simulator
from tests.integration.test_service import config, finished


def specification(**changes):
    return BatchRequest.model_validate(
        {
            "name": "wheel_radius",
            "base": config(),
            "seeds": [11, 12],
            "axes": [{"path": "robot.wheel_radius", "values": [0.04, 0.05]}],
            **changes,
        }
    )


def batch_finished(service, batch_id):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        result = service.get(batch_id)
        if result["status"] not in ACTIVE:
            return result
        time.sleep(0.01)
    pytest.fail("batch did not finish in time")


def test_preview_matches_execution_replay_and_restart(tmp_path):
    runs = RunService(tmp_path)
    batches = BatchService(runs)
    try:
        preview, inputs, _ = prepare_batch(specification())
        assert preview["expected_trials"] == 4
        assert preview["resources"]["steps"] == 40
        assert [value["seed"] for value in inputs] == [11, 12, 11, 12]
        batch = batch_finished(batches, batches.submit(specification())["id"])
        assert batch["finished_trials"] == 4 and batch["status"] == "completed"
        assert runs.list()["total"] == 4
        for expected, trial in zip(preview["trials"], batch["trials"]):
            record = runs.get(trial["run_id"])
            assert record["config_sha256"] == expected["config_sha256"]
            assert record["seed"] == trial["seed"]
            assert runs.replay(trial["run_id"]).at(0).state.time == 0
        assert all(
            group["trials"] == 2 and group["failure_rate"] == 0
            for group in batch["groups"].values()
        )
        assert json.loads(batches.artifact(batch["id"], "inputs.json").read_text()) == inputs
    finally:
        batches.close()
        runs.close()
    restored_runs = RunService(tmp_path)
    restored = BatchService(restored_runs)
    try:
        assert restored.get(batch["id"])["groups"] == batch["groups"]
        assert restored.list()["total"] == 1
    finally:
        restored.close()
        restored_runs.close()


def test_invalid_trials_are_retained_without_submitting_runs(tmp_path):
    runs = RunService(tmp_path)
    batches = BatchService(runs)
    try:
        spec = specification(axes=[{"path": "robot.wheel_radius", "values": [-1, 0.05]}])
        preview, _, _ = prepare_batch(spec)
        assert [t["status"] for t in preview["trials"]] == [
            "invalid",
            "invalid",
            "pending",
            "pending",
        ]
        batch = batch_finished(batches, batches.submit(spec)["id"])
        assert batch["status"] == "completed" and batch["finished_trials"] == 4
        assert runs.list()["total"] == 2
        assert batch["groups"]["group-0000"]["failure_rate"] == 1
        assert batch["groups"]["group-0000"]["metrics"] == {}
        assert batch["trials"][0]["run_id"] is None
        assert batch["trials"][0]["error"]
    finally:
        batches.close()
        runs.close()


def test_cancellation_does_not_submit_remaining_trials_or_cancel_other_runs(tmp_path, monkeypatch):
    started = threading.Event()

    def blocked(self, should_cancel):
        started.set()
        deadline = time.monotonic() + 5
        while not should_cancel() and time.monotonic() < deadline:
            time.sleep(0.005)
        raise SimulationCancelled()

    runs = RunService(tmp_path)
    batches = BatchService(runs)
    completed = finished(runs, runs.submit(config())["id"])
    monkeypatch.setattr(Simulator, "run", blocked)
    try:
        batch_id = batches.submit(specification())["id"]
        assert started.wait(2)
        with pytest.raises(ServiceError) as exc:
            batches.submit(specification())
        assert exc.value.status == 429
        assert batches.cancel(batch_id)["cancel_requested"]
        result = batch_finished(batches, batch_id)
        assert result["status"] == "cancelled"
        assert [t["status"] for t in result["trials"]] == ["cancelled"] * 4
        assert sum(t["run_id"] is not None for t in result["trials"]) == 1
        assert runs.get(completed["id"])["status"] == "completed"
        assert batches.cancel(batch_id)["status"] == "cancelled"
    finally:
        batches.close()
        runs.close()


def test_recovery_preserves_finished_child_and_marks_unstarted_trials_interrupted(tmp_path):
    runs = RunService(tmp_path)
    batches = BatchService(runs)
    batch = batch_finished(batches, batches.submit(specification())["id"])
    batches.close()
    runs.close()
    path = tmp_path / "batches" / batch["id"] / "batch.json"
    record = json.loads(path.read_text())
    record["status"] = "running"
    record["trials"][0]["status"] = "running"
    record["trials"][1].update(run_id=None, status="pending", metrics={})
    path.write_text(json.dumps(record))
    runs = RunService(tmp_path)
    batches = BatchService(runs)
    try:
        restored = batches.get(batch["id"])
        assert restored["status"] == "interrupted"
        assert restored["trials"][0]["status"] == "completed"
        assert restored["trials"][1]["status"] == "interrupted"
        assert restored["finished_trials"] == 4
        assert runs.list()["total"] == 4  # No automatic reruns.
    finally:
        batches.close()
        runs.close()


@pytest.mark.parametrize(
    "changes",
    [
        {"seeds": [1, 1]},
        {"seeds": list(range(9))},
        {"seeds": [9007199254740992]},
        {"max_runs": 33},
        {"max_total_steps": 100001},
        {"axes": [{"path": "robot.wheel_radius", "values": list(range(17))}]},
    ],
)
def test_batch_schema_bounds(changes):
    with pytest.raises(ValueError):
        specification(**changes)


def test_batch_workload_and_path_bounds():
    with pytest.raises(ServiceError, match="unknown sweep path"):
        prepare_batch(specification(axes=[{"path": "robot.typo", "values": [1]}]))
    with pytest.raises(ServiceError, match="budget"):
        prepare_batch(specification(max_total_steps=20))
    spec = specification(
        base={"commands": [{"left": 1, "right": 1, "steps": 50100}]}, axes=[], seeds=[1]
    )
    with pytest.raises(ServiceError, match="50,000"):
        prepare_batch(spec)


def test_batch_api_contract_and_exports(tmp_path):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from roboforge.api import create_app

    with TestClient(create_app(tmp_path)) as client:
        spec = specification().model_dump(mode="json")
        assert client.post("/api/experiments/preview", json=spec).json()["expected_trials"] == 4
        assert client.get("/api/runs").json()["total"] == 0
        response = client.post("/api/experiments", json=spec)
        assert response.status_code == 202
        batch_id = response.json()["id"]
        result = batch_finished(client.app.state.batches, batch_id)
        assert client.get(f"/api/experiments/{batch_id}").json() == result
        report = client.get(f"/api/experiments/{batch_id}/report")
        assert report.json()["finished_trials"] == 4
        assert "attachment" in report.headers["content-disposition"]
        assert client.get(f"/api/experiments/{batch_id}/artifacts/experiment.json").json() == spec
        assert client.get(f"/api/experiments/{batch_id}/artifacts/private.key").status_code == 404
        assert client.get("/api/experiments/missing").status_code == 404
        assert client.get("/api/experiments?offset=1").json()["items"] == []
        assert (
            client.post(
                "/api/experiments", json=spec, headers={"Origin": "https://elsewhere.example"}
            ).status_code
            == 403
        )


def test_cancel_while_shared_queue_is_full_leaves_unrelated_run_running(tmp_path, monkeypatch):
    started, release = threading.Event(), threading.Event()
    original = Simulator.run

    def held(self, should_cancel):
        started.set()
        assert release.wait(5)
        assert not should_cancel()
        return original(self, should_cancel)

    monkeypatch.setattr(Simulator, "run", held)
    runs = RunService(tmp_path, capacity=1)
    batches = BatchService(runs)
    try:
        separate_id = runs.submit(config())["id"]
        assert started.wait(2)
        batch_id = batches.submit(specification())["id"]
        batches.cancel(batch_id)
        record = batch_finished(batches, batch_id)
        assert record["status"] == "cancelled"
        assert all(trial["run_id"] is None for trial in record["trials"])
        assert runs.get(separate_id)["status"] == "running"
        release.set()
        assert finished(runs, separate_id)["status"] == "completed"
    finally:
        release.set()
        batches.close()
        runs.close()


def test_failed_run_does_not_prevent_remaining_trials(tmp_path, monkeypatch):
    original = Simulator.run

    def partly_broken(self, should_cancel):
        if self.config.seed == 11:
            raise RuntimeError("failed seed")
        return original(self, should_cancel)

    monkeypatch.setattr(Simulator, "run", partly_broken)
    runs = RunService(tmp_path)
    batches = BatchService(runs)
    try:
        batch = batch_finished(batches, batches.submit(specification())["id"])
        assert [trial["status"] for trial in batch["trials"]] == [
            "failed",
            "completed",
            "failed",
            "completed",
        ]
        for group in batch["groups"].values():
            assert group["failure_rate"] == 0.5
            assert group["metrics"]["duration_s"]["measured_trials"] == 1
            assert group["metrics"]["duration_s"]["sample_stddev"] is None
    finally:
        batches.close()
        runs.close()
