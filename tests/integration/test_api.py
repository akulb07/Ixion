import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
from fastapi.testclient import TestClient

from roboforge.api import create_app
from tests.integration.test_service import config, finished


def test_api_end_to_end_replay_and_artifacts(tmp_path):
    app = create_app(tmp_path)
    with TestClient(app) as client:
        assert client.get("/api/health").json()["status"] == "ok"
        assert client.get("/api/config/schema").json()["title"] == "RunConfig"
        presets = client.get("/api/presets").json()
        assert len(presets) == 4
        for preset in presets:
            assert client.post("/api/config/validate", json=preset["config"]).status_code == 200
        response = client.post("/api/runs", json=config().model_dump(mode="json"))
        assert response.status_code == 202
        run_id = response.json()["id"]
        assert finished(app.state.service, run_id)["status"] == "completed"
        prefix = f"/api/runs/{run_id}"
        assert client.get(prefix).json()["metrics"]["duration_s"] == 0.1
        assert client.get(prefix + "/config").json() == config().model_dump(mode="json")
        assert client.get("/api/runs").json()["total"] == 1
        assert client.get(prefix + "/frame?time=0").json()["sensors"] == []
        frame = client.get(prefix + "/frame?time=.025").json()
        assert frame["state"]["time"] == 0.025
        assert all(r["delivery_time"] <= 0.025 for r in frame["sensors"])
        replay = app.state.service.replay(run_id)
        for reading in frame["sensors"]:
            capture_pose = replay.state_at(reading["capture_time"]).pose
            assert frame["sensor_capture_poses"][reading["sensor"]] == {
                "x": capture_pose.x,
                "y": capture_pose.y,
                "theta": capture_pose.theta,
            }
            assert capture_pose != replay.state_at(0.025).pose
        trajectory = client.get(prefix + "/trajectory?max_points=2").json()
        assert trajectory["sampled"] and trajectory["total_states"] == 11
        assert [s["time"] for s in trajectory["states"]] == [0, 0.1]
        assert client.get(prefix + "/artifacts/manifest.json").status_code == 200
        assert client.get(prefix + "/artifacts/secret.txt").status_code == 404
        assert client.get(prefix + "/frame?time=2").status_code == 422
        assert client.get(prefix + "/frame?time=nan").status_code == 422
        assert client.get("/api/runs/not-a-run").status_code == 404


def test_api_rejects_invalid_and_cross_origin_requests(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        assert client.post("/api/runs", json={"typo": True}).status_code == 422
        assert (
            client.post(
                "/api/runs", json={}, headers={"Origin": "https://evil.example"}
            ).status_code
            == 403
        )
        assert client.get("/api/health", headers={"Host": "evil.example"}).status_code == 400
        assert client.post("/api/runs", content=b"x" * 1000001).status_code == 413
        assert (
            client.post("/api/runs", content=iter([b"x" * 600000, b"y" * 600000])).status_code
            == 413
        )
        assert (
            client.post(
                "/api/config/validate",
                json=config().model_dump(mode="json"),
                headers={"Origin": "http://testserver"},
            ).status_code
            == 200
        )
        assert client.get("/api/runs?limit=101").status_code == 422
        assert client.get("/api/runs").json()["total"] == 0


def test_workspace_assets_are_local_and_packaged(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        home = client.get("/")
        assert home.status_code == 200
        assert "RoboForge" in home.text
        assert 'src="/assets/app.js?v=' in home.text
        for asset in ("app.js", "app.css"):
            response = client.get("/assets/" + asset)
            assert response.status_code == 200 and len(response.content) > 1000
        assert client.get("/assets/does-not-exist.js").status_code == 404
