import copy

from fastapi.testclient import TestClient

from roboforge.api import create_app, presets
from roboforge.config import RunConfig
from tests.integration.test_service import finished


def test_repeated_fault_setup_reproduces_saved_data_and_baseline_is_separate(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.service
        config = RunConfig.model_validate(presets()[3]["config"])
        first, second = [finished(service, service.submit(config)["id"]) for _ in range(2)]
        assert first["status"] == second["status"] == "completed"
        for artifact in ("trajectory.csv", "sensors.jsonl", "control.json", "faults.json"):
            left = client.get(f"/api/runs/{first['id']}/artifacts/{artifact}")
            right = client.get(f"/api/runs/{second['id']}/artifacts/{artifact}")
            assert left.status_code == right.status_code == 200
            assert left.content == right.content
        readings = service.replay(first["id"]).readings
        missing = [r for r in readings if r.sensor == "encoders" and r.left_ticks is None]
        assert missing and all(1 <= r.capture_time < 2 for r in missing)
        baseline = config.model_copy(update={"faults": ()})
        clean = finished(service, service.submit(baseline)["id"])
        assert clean["status"] == "completed"
        assert service.config(first["id"]).faults == config.faults
        assert not service.config(clean["id"]).faults
        assert all(
            r.left_ticks is not None
            for r in service.replay(clean["id"]).readings
            if r.sensor == "encoders"
        )


def test_fault_editor_values_use_authoritative_setup_validation(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        base = presets()[3]["config"]
        for update in (
            {"end": 0.5},
            {"target": "missing"},
            {"magnitude": 1.1},
            {"name": "bad name"},
            {"kind": "gyro_bias", "target": "encoders"},
            {"kind": "actuator_delay", "target": "both", "delay_steps": 2.5, "magnitude": 0},
        ):
            config = copy.deepcopy(base)
            config["faults"][0].update(update)
            assert client.post("/api/config/validate", json=config).status_code == 422
        config = copy.deepcopy(base)
        config["faults"][0].update(kind="actuator_delay", target="both", magnitude=0, delay_steps=5)
        assert client.post("/api/config/validate", json=config).status_code == 200
        config["faults"].append(copy.deepcopy(config["faults"][0]))
        assert client.post("/api/config/validate", json=config).status_code == 422
