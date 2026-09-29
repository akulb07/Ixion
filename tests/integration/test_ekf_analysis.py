import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from fastapi.testclient import TestClient

from roboforge.api import create_app, presets
from roboforge.config import RunConfig
from roboforge.ekf_analysis import analyze_ekf
from roboforge.geometry import Pose2
from roboforge.sensors.readings import ImuReading
from roboforge.service import ServiceError
from tests.integration.test_odometry_analysis import reading
from tests.integration.test_service import finished


def gyro(index, time, delay=0):
    return ImuReading(
        sensor="imu",
        frame="imu",
        sequence=index,
        capture_time=time,
        delivery_time=time + delay,
        gyro_z=0,
        acceleration_x=None,
        acceleration_y=None,
    )


def replay(truth=Pose2(), extra=()):
    encoders = [
        reading(i, i / 10, i * 100).model_copy(update={"right_ticks": i * 120}) for i in range(6)
    ]
    gyros = [gyro(i, i / 10) for i in range(6)]
    config = RunConfig.model_validate({"commands": [{"left": 0, "right": 0, "steps": 1}]})
    return SimpleNamespace(
        readings=[*encoders, *gyros, *extra],
        config=config,
        states=[SimpleNamespace(time=0.5)],
        state_at=lambda t: SimpleNamespace(pose=truth),
    )


def test_fusion_covariance_truth_separation_and_sampling():
    result = analyze_ekf(replay(), "encoders", "imu")
    assert result["gyro_accepted"] == 5 and result["gyro_rejected"] == 0
    assert abs(result["samples"][-1]["estimate"]["theta"]) < 0.001
    assert abs(result["samples"][-1]["odometry"]["theta"]) > 0.05
    for sample in result["samples"]:
        assert np.linalg.eigvalsh(sample["covariance"]).min() >= -1e-12
    changed = analyze_ekf(replay(Pose2(4, 5, 1)), "encoders", "imu")
    assert [s["estimate"] for s in changed["samples"]] == [s["estimate"] for s in result["samples"]]
    assert changed["metrics"] != result["metrics"]
    sampled = analyze_ekf(replay(), "encoders", "imu", max_points=2)
    assert sampled["metrics"] == result["metrics"] and sampled["sampled"]
    assert sampled["samples"] == [result["samples"][0], result["samples"][-1]]
    center = result["samples"][-1]["estimate"]
    p = np.array(result["samples"][-1]["covariance"])[:2, :2]
    for point in result["final_position_ellipse95"]:
        delta = np.array([point["x"] - center["x"], point["y"] - center["y"]])
        assert delta @ np.linalg.solve(p, delta) == pytest.approx(5.991464547107979)


def test_missing_delayed_and_dropout_gyro_handling():
    log = replay()
    log.readings = [r for r in log.readings if not isinstance(r, ImuReading)] + [
        gyro(0, 0),
        gyro(1, 0.1, 1),
    ]
    result = analyze_ekf(log, "encoders", "imu")
    assert result["encoder_only_intervals"] == 5 and result["gyro_accepted"] == 0
    assert result["metrics"]["final_heading_error_rad"] == pytest.approx(
        result["odometry_metrics"]["final_heading_error_rad"]
    )
    with pytest.raises(ServiceError, match="duplicate"):
        analyze_ekf(replay(extra=[gyro(0, 0)]), "encoders", "imu")
    with pytest.raises(ServiceError, match="no delivered"):
        analyze_ekf(replay(), "encoders", "absent")


def test_gyro_gate_rejection_is_reported():
    log = replay()
    log.readings = [
        r.model_copy(update={"gyro_z": 100}) if isinstance(r, ImuReading) else r
        for r in log.readings
    ]
    result = analyze_ekf(log, "encoders", "imu")
    assert result["gyro_rejected"] == 5 and result["gyro_accepted"] == 0


def test_ekf_api_exports_settings_and_does_not_resimulate(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.service
        document = presets()[1]["config"]
        document["commands"][0]["steps"] = 100
        run = finished(service, service.submit(RunConfig.model_validate(document))["id"])
        url = f"/api/runs/{run['id']}/ekf"
        result = client.get(url, params={"max_points": 2, "gyro_stddev": 0.03})
        assert result.status_code == 200 and "attachment" in result.headers["content-disposition"]
        assert result.json()["settings"]["gyro_stddev_rad_s"] == 0.03
        assert len(result.json()["samples"]) == 2
        for params in [
            {"gyro_stddev": 0},
            {"wheel_variance": -1},
            {"max_points": 2001},
            {"imu": "missing"},
        ]:
            assert client.get(url, params=params).status_code == 422
        assert service.list()["total"] == 1


def test_config_remains_readable_during_result_export(tmp_path, monkeypatch):
    from roboforge.service import RunService

    exporting, release = threading.Event(), threading.Event()
    original = Path.write_text
    original_replace = Path.replace

    def locked_config(path, target):
        if Path(target).name == "config.json" and Path(target).exists():
            raise PermissionError("Windows reader holds the published setup open")
        return original_replace(path, target)

    def blocked_write(path, text, *args, **kwargs):
        if path.name == "metadata.json":
            exporting.set()
            assert release.wait(5)
        return original(path, text, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", blocked_write)
    monkeypatch.setattr(Path, "replace", locked_config)
    service = RunService(tmp_path)
    config = RunConfig.model_validate({"commands": [{"left": 1, "right": 1, "steps": 2}]})
    try:
        run = service.submit(config)
        assert exporting.wait(5)
        assert service.config(run["id"]) == config
        release.set()
        assert finished(service, run["id"])["status"] == "completed"
        assert service.replay(run["id"]).config == config
    finally:
        release.set()
        service.close()
