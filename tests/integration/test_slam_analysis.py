from dataclasses import replace
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from roboforge.api import create_app, presets
from roboforge.config import RunConfig
from roboforge.geometry import Pose2
from roboforge.mapping import GridConfig
from roboforge.service import ServiceError
from roboforge.slam import IncrementalSlam, MatchResult, SlamConfig
from roboforge.slam_analysis import analyze_slam
from tests.integration.test_odometry_analysis import reading
from tests.integration.test_service import finished
from tests.unit.test_slam import scan


def replay(truth=Pose2(), scans=None):
    config = RunConfig.model_validate({"commands": [{"left": 0, "right": 0, "steps": 1}]})
    return SimpleNamespace(
        config=config,
        readings=[reading(i, float(i), 0) for i in range(3)]
        + [
            s.model_copy(update={"frame": "base"})
            for s in (scans if scans is not None else [scan(0), scan(1), scan(2, invalid=True)])
        ],
        states=[SimpleNamespace(time=2)],
        state_at=lambda t: SimpleNamespace(pose=truth),
    )


def test_analysis_truth_is_scoring_only_and_rejected_scans_do_not_map():
    result = analyze_slam(replay(), "lidar", "encoders")
    assert result["counts"] == {"initialized": 1, "matched": 1, "rejected": 1}
    assert result["samples"][-1]["rejection_reasons"] == ("insufficient_correspondences",)
    assert not result["samples"][-1]["map_updated"]
    accepted_only = analyze_slam(replay(scans=[scan(0), scan(1)]), "lidar", "encoders")
    assert result["states"] == accepted_only["states"]
    assert result["map_points"] == accepted_only["map_points"]
    shifted = analyze_slam(replay(Pose2(9, 3, 1)), "lidar", "encoders")
    assert shifted["map_points"] == result["map_points"]
    assert [s["estimate"] for s in shifted["samples"]] == [s["estimate"] for s in result["samples"]]
    assert shifted["metrics"] != result["metrics"]


def test_missing_prior_late_delivery_and_initial_empty_scan():
    middle = scan(1).model_copy(update={"capture_time": 0.5, "delivery_time": 0.5})
    late = scan(2).model_copy(update={"delivery_time": 3})
    result = analyze_slam(replay(scans=[scan(0, invalid=True), middle, late]), "lidar", "encoders")
    assert result["counts"] == {"rejected": 1}
    assert result["rejection_counts"] == {"insufficient_hits": 1}
    assert result["skipped_scans"] == [{"capture_time": 0.5, "reason": "missing_encoder_prior"}]
    assert result["map_points"] == []
    assert all(cell == -1 for row in result["states"] for cell in row)


@pytest.mark.parametrize(
    "scans, message",
    [
        ([scan(0), scan(0)], "duplicate"),
        ([], "no delivered"),
        ([scan(1).model_copy(update={"capture_time": 0.5})], "no scans have matching"),
        ([scan(i) for i in range(201)], "work budget"),
    ],
)
def test_invalid_streams_and_work_budget(scans, message):
    log = replay(scans=scans)
    log.states[-1].time = 300
    with pytest.raises(ServiceError, match=message):
        analyze_slam(log, "lidar", "encoders")


def test_rejection_gates_report_all_reasons_without_map_insertion(monkeypatch):
    import roboforge.slam as module

    slam = IncrementalSlam(GridConfig(columns=60, rows=60, resolution=0.1), SlamConfig())
    slam.update(scan(0), Pose2(3, 3), prior_time=0)
    before = slam.grid.states().tolist()
    result = MatchResult(Pose2(4, 3, 0.5), "converged", 2, 40, 0.2, (0.3, 0.2))
    monkeypatch.setattr(module, "match_points", lambda *a: result)
    estimate = slam.update(scan(1), Pose2(3, 3), prior_time=1)
    assert estimate.rejection_reasons == ("residual_gate", "translation_gate", "rotation_gate")
    assert estimate.pose == Pose2(3, 3) and not estimate.map_updated
    assert slam.grid.states().tolist() == before
    result = replace(result, status="iteration_limit", pose=Pose2(3, 3), rmse=0.01)
    estimate = slam.update(scan(2), Pose2(3, 3), prior_time=2)
    assert estimate.rejection_reasons == ("iteration_limit",)


def test_slam_api_exports_settings_without_creating_another_run(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.service
        config = presets()[1]["config"]
        config["commands"][0]["steps"] = 50
        run = finished(service, service.submit(RunConfig.model_validate(config))["id"])
        url = f"/api/runs/{run['id']}/slam"
        response = client.get(url, params={"max_match_rmse": 0.03})
        assert response.status_code == 200
        assert "attachment" in response.headers["content-disposition"]
        data = response.json()
        assert data["settings"]["slam"]["max_match_rmse"] == 0.03
        assert data["samples"] and len(data["map_points"]) <= 300
        for params in [{"max_match_rmse": 0}, {"resolution": 0.01}, {"sensor": "missing"}]:
            assert client.get(url, params=params).status_code == 422
        assert service.list()["total"] == 1
