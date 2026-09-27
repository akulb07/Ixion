import math

import pytest
from pydantic import ValidationError

from roboforge.config import RunConfig
from roboforge.geometry import Pose2
from roboforge.io import save_result
from roboforge.replay import ReplayLog
from roboforge.service import ServiceError, resource_estimate
from roboforge.simulation import SimulationCancelled, Simulator
from tests.integration.test_service import finished


def config(**changes):
    document = {
        "name": "navigation_test",
        "robot": {"initial_pose": {"x": 1, "y": 1}, "footprint_radius": 0.1},
        "environment": {"width": 4, "height": 4},
        "simulation": {"dt": 0.02, "collision": {"mode": "stop"}},
        "sensors": [{"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 65536}],
        "navigation": {"path": [{"x": 1, "y": 1}, {"x": 2, "y": 1}], "max_steps": 500},
        **changes,
    }
    return RunConfig.model_validate(document)


def test_navigation_reaches_reproducibly_and_replay_retains_estimates(tmp_path):
    c = config()
    first = Simulator(c).run()
    assert first == Simulator(c).run()
    assert first.status == "completed" and first.navigation_outcome == "reached"
    assert abs(first.states[-1].pose.x - 2) <= c.navigation.goal_tolerance
    save_result(first, tmp_path)
    frame = ReplayLog(tmp_path).at(first.states[-1].time)
    assert frame.navigation["estimate"]["x"] == first.navigation_samples[-1].estimate.x
    assert frame.state == first.states[-1]


def test_navigation_and_pid_share_delivered_sensor_batch():
    c = config(
        wheel_controller={"left": {"kp": 0.2}, "right": {"kp": 0.2}},
        sensors=[
            {"type": "encoder", "rate_hz": 50, "latency": 0.04, "ticks_per_revolution": 65536}
        ],
    )
    result = Simulator(c).run()
    assert result.navigation_outcome == "reached"
    assert result.control_samples
    assert all(s.requested.left == 0 for s in result.navigation_samples if s.time < 0.04)
    for sample in result.navigation_samples:
        if sample.estimate_time is not None:
            assert sample.estimate_time + 0.04 <= sample.time + 1e-12


def test_slip_does_not_feed_ground_truth_into_follower():
    result = Simulator(
        config(faults=[{"name": "slip", "kind": "wheel_slip", "target": "both", "magnitude": 0.5}])
    ).run()
    assert result.navigation_outcome == "reached"
    assert abs(result.navigation_samples[-1].estimate.x - 2) < 0.051
    assert abs(result.states[-1].pose.x - 2) > 0.45


def test_missing_encoder_data_stops_requests_and_retains_budget_failure():
    c = config(
        faults=[{"name": "offline", "kind": "sensor_dropout", "target": "encoders", "magnitude": 1}]
    )
    result = Simulator(c).run()
    assert result.status == result.navigation_outcome == "budget_exceeded"
    assert result.states[-1].pose == Pose2(1, 1, 0)
    assert all(not s.measurement_fresh for s in result.navigation_samples)
    assert result.states[-1].time == 10


def test_path_validation_and_execution_collision_are_distinct():
    with pytest.raises(ValueError, match="intersects"):
        Simulator(
            config(
                environment={
                    "width": 4,
                    "height": 4,
                    "obstacles": [{"type": "circle", "x": 1.5, "y": 1, "radius": 0.2}],
                }
            )
        ).run()
    c = config(
        environment={
            "width": 4,
            "height": 4,
            "obstacles": [{"type": "rectangle", "x": 1.5, "y": 1.5, "width": 1, "height": 1}],
        },
        navigation={
            "path": [{"x": 1, "y": 1}, {"x": 1, "y": 3}, {"x": 3, "y": 3}, {"x": 3, "y": 1}],
            "lookahead": 3,
            "max_steps": 1000,
        },
    )
    result = Simulator(c).run()
    assert result.status == result.navigation_outcome == "collision"
    assert result.collisions


def test_navigation_bounds_and_cancellation():
    with pytest.raises(ValidationError):
        config(commands=[{"left": 1, "right": 1, "steps": 10}])
    with pytest.raises(ValidationError):
        config(sensors=[])
    c = config(navigation={"path": [{"x": 1, "y": 1}, {"x": 2, "y": 1}], "max_steps": 20000})
    with pytest.raises(ServiceError):
        resource_estimate(c)
    with pytest.raises(SimulationCancelled):
        Simulator(config()).run(lambda: True)


@pytest.mark.parametrize("steps,outcome", [(500, "reached"), (50, "budget_exceeded")])
def test_navigation_api_history_and_recorded_frame(tmp_path, steps, outcome):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from roboforge.api import create_app

    with TestClient(create_app(tmp_path)) as client:
        scenario = config(
            navigation={"path": [{"x": 1, "y": 1}, {"x": 2, "y": 1}], "max_steps": steps}
        )
        response = client.post("/api/runs", json=scenario.model_dump(mode="json"))
        assert response.status_code == 202
        job = finished(client.app.state.service, response.json()["id"])
        assert job["navigation_outcome"] == outcome
        prefix = f"/api/runs/{job['id']}"
        frame = client.get(prefix + "/frame?time=1").json()
        assert frame["navigation"]["time"] <= 1
        assert frame["navigation"]["estimate_time"] <= 1
        artifact = client.get(prefix + "/artifacts/navigation.json")
        assert artifact.status_code == 200 and artifact.json()["outcome"] == outcome
        assert math.isfinite(job["metrics"]["truth_goal_error_m"])


def test_stale_encoder_requests_zero_after_a_dropout():
    result = Simulator(
        config(
            faults=[
                {
                    "name": "offline",
                    "kind": "sensor_dropout",
                    "target": "encoders",
                    "start": 0.2,
                    "magnitude": 1,
                }
            ]
        )
    ).run()
    late = [s for s in result.navigation_samples if s.time >= 0.8]
    assert late and all(
        not s.measurement_fresh and s.requested.left == s.requested.right == 0 for s in late
    )
    assert result.status == "budget_exceeded"
    assert result.states[-1].pose.x < 1.3
