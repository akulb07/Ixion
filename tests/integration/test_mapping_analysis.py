from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from roboforge.api import create_app, presets
from roboforge.config import RunConfig
from roboforge.geometry import Pose2
from roboforge.mapping_analysis import analyze_map
from roboforge.sensors.readings import LidarReading
from roboforge.service import ServiceError
from tests.integration.test_odometry_analysis import reading
from tests.integration.test_service import finished


def scan(time=0, delay=0, frame="base"):
    return LidarReading(
        sensor="lidar",
        frame=frame,
        sequence=int(time * 100),
        capture_time=time,
        delivery_time=time + delay,
        angles=(0,),
        ranges=(1,),
        hits=(True,),
        max_range=2,
    )


def log(scans, truth=Pose2(), config=None):
    return SimpleNamespace(
        config=config
        or RunConfig.model_validate({"commands": [{"left": 0, "right": 0, "steps": 1}]}),
        readings=[reading(0, 0, 0), *scans],
        states=[SimpleNamespace(time=1)],
        state_at=lambda time: SimpleNamespace(pose=truth),
    )


def test_map_cell_orientation_delayed_scan_and_explicit_pose_source():
    result = analyze_map(log([scan(), scan(1, 0.1)]), "lidar", "truth", "encoders", 0.5)
    assert result["used_scans"] == 1 and result["states"][0][2] == 100
    assert result["states"][0][0] == 0 and result["states"][1][0] == -1
    assert result["observed"][0][2] and result["probabilities"][0][2] == pytest.approx(0.7)
    estimated = analyze_map(log([scan(), scan(0.5)]), "lidar", "encoder", "encoders", 0.5)
    shifted = analyze_map(
        log([scan(), scan(0.5)], Pose2(2, 2)), "lidar", "encoder", "encoders", 0.5
    )
    assert estimated["states"] == shifted["states"]
    assert estimated["used_scans"] == 1 and estimated["skipped_scans"] == 1


def test_mount_translation_and_rotation_are_applied():
    config = RunConfig.model_validate(
        {
            "commands": [{"left": 0, "right": 0, "steps": 1}],
            "robot": {
                "mounts": [
                    {"name": "scanner", "pose": {"x": 1, "y": 1, "theta": 1.5707963267948966}}
                ]
            },
        }
    )
    result = analyze_map(
        log([scan(frame="scanner")], config=config), "lidar", "truth", "encoders", 0.5
    )
    assert result["states"][4][2] == 100


def test_mapping_workload_and_invalid_streams_are_rejected():
    for source, resolution, scans in [
        ("bad", 0.5, [scan()]),
        ("truth", 0.0001, [scan()]),
        ("truth", 0.5, []),
        ("truth", 0.5, [scan(frame="missing")]),
        ("truth", 0.5, [scan(), scan()]),
    ]:
        with pytest.raises(ServiceError):
            analyze_map(log(scans), "lidar", source, "encoders", resolution)
    huge = scan().model_copy(
        update={"angles": (0,) * 50001, "ranges": (1,) * 50001, "hits": (True,) * 50001}
    )
    with pytest.raises(ServiceError, match="budget"):
        analyze_map(log([huge]), "lidar", "truth", "encoders", 0.5)


def test_map_api_exports_saved_measurements_without_resimulation(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.service
        document = presets()[0]["config"]
        document["commands"][0]["steps"] = 20
        run = finished(service, service.submit(RunConfig.model_validate(document))["id"])
        url = f"/api/runs/{run['id']}/map"
        response = client.get(url, params={"sensor": "lidar", "pose_source": "truth"})
        assert response.status_code == 200
        assert response.json()["used_scans"] > 0
        assert sum(response.json()["counts"].values()) == 6400
        assert "attachment" in response.headers["content-disposition"]
        assert client.get(url, params={"sensor": "lidar", "pose_source": "bad"}).status_code == 422
        assert client.get(url, params={"sensor": "lidar", "resolution": 0.05}).status_code == 422
        assert service.list()["total"] == 1
