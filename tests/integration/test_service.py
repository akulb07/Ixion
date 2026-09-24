import json
import threading
import time

import pytest

from roboforge.config import RunConfig
from roboforge.service import ACTIVE, RunService, ServiceError, resource_estimate
from roboforge.simulation import SimulationCancelled, Simulator


def config():
    return RunConfig.model_validate(
        {
            "commands": [{"left": 2, "right": 4, "steps": 10}],
            "sensors": [{"type": "encoder", "rate_hz": 100, "latency": 0.02}],
        }
    )


def finished(service, run_id):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        record = service.get(run_id)
        if record["status"] not in ACTIVE:
            return record
        time.sleep(0.01)
    pytest.fail("run failed to finish within test timeout")


def test_cooperative_cancellation_preserves_default_simulation():
    assert Simulator(config()).run() == Simulator(config()).run(lambda: False)
    calls = []

    def stop():
        calls.append(True)
        return len(calls) == 4

    with pytest.raises(SimulationCancelled):
        Simulator(config()).run(stop)
    assert len(calls) == 4


def test_service_persists_integrity_and_excludes_second_writer(tmp_path, monkeypatch):
    service = RunService(tmp_path)
    try:
        with pytest.raises(ServiceError, match="another service"):
            RunService(tmp_path)
        run_id = service.submit(config())["id"]
        assert finished(service, run_id)["status"] == "completed"
    finally:
        service.close()
    restored = RunService(tmp_path)
    try:
        monkeypatch.setattr(
            Simulator, "run", lambda *a, **k: pytest.fail("replay reran simulation")
        )
        assert restored.list()["total"] == 1
        assert restored.replay(run_id).at(0.1).state.time == 0.1
        assert restored.cancel(run_id)["status"] == "completed"
        restored._cache.clear()
        with (tmp_path / run_id / "trajectory.csv").open("a") as stream:
            stream.write("tampered")
        with pytest.raises(ServiceError, match="integrity"):
            restored.replay(run_id)
        with pytest.raises(ServiceError):
            restored.artifact(run_id, "../config.json")
    finally:
        restored.close()


def test_queue_capacity_cancellation_and_failures(tmp_path, monkeypatch):
    entered = threading.Event()

    def blocked(self, should_cancel):
        entered.set()
        deadline = time.monotonic() + 5
        while not should_cancel() and time.monotonic() < deadline:
            time.sleep(0.005)
        raise SimulationCancelled()

    monkeypatch.setattr(Simulator, "run", blocked)
    service = RunService(tmp_path, capacity=1)
    try:
        run_id = service.submit(config())["id"]
        assert entered.wait(2)
        with pytest.raises(ServiceError) as error:
            service.submit(config())
        assert error.value.status == 429
        with pytest.raises(ServiceError, match="no completed replay"):
            service.replay(run_id)
        assert service.cancel(run_id)["cancel_requested"]
        assert finished(service, run_id)["status"] == "cancelled"

        def broken(*args, **kwargs):
            raise RuntimeError("deliberate failure")

        monkeypatch.setattr(Simulator, "run", broken)
        failure = finished(service, service.submit(config())["id"])
        assert failure["status"] == "failed"
        assert "deliberate failure" in failure["error"]
        assert service.list()["total"] == 2
    finally:
        service.close()


def test_restart_marks_unfinished_jobs_and_skips_bad_records(tmp_path):
    service = RunService(tmp_path)
    run_id = service.submit(config())["id"]
    record = finished(service, run_id)
    service.close()
    record["status"] = "running"
    (tmp_path / run_id / "job.json").write_text(json.dumps(record))
    malformed = tmp_path / ("run-" + "a" * 32)
    malformed.mkdir()
    (malformed / "job.json").write_text(json.dumps({"id": malformed.name, "status": "completed"}))
    recovered = RunService(tmp_path)
    try:
        assert recovered.get(run_id)["status"] == "interrupted"
        assert recovered.list()["total"] == 1
        assert recovered.recovery_warnings == [malformed.name]
    finally:
        recovered.close()


@pytest.mark.parametrize(
    "change",
    [
        {"commands": [{"left": 0, "right": 0, "steps": 50001}]},
        {"simulation": {"dt": 100}},
        {"sensors": [{"type": "encoder", "rate_hz": 1e308}]},
        {"sensors": [{"type": "lidar", "rate_hz": 1000, "rays": 20000}]},
    ],
)
def test_resource_limits(change):
    document = config().model_dump(mode="json")
    document.update(change)
    with pytest.raises(ServiceError) as error:
        resource_estimate(RunConfig.model_validate(document))
    assert error.value.status == 422
