from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from roboforge.api import create_app
from roboforge.config import RunConfig
from roboforge.geometry import Pose2
from roboforge.odometry_analysis import analyze_odometry
from roboforge.sensors.readings import EncoderReading
from roboforge.service import ServiceError
from tests.integration.test_service import finished


def reading(sequence, time, ticks, delay=0):
    return EncoderReading(
        sensor="encoders",
        frame="base",
        sequence=sequence,
        capture_time=time,
        delivery_time=time + delay,
        ticks_per_revolution=1000,
        left_ticks=ticks,
        right_ticks=ticks,
    )


def replay(readings, truth=Pose2(), end=2):
    return SimpleNamespace(
        readings=readings,
        states=[SimpleNamespace(time=end)],
        config=RunConfig.model_validate({"commands": [{"left": 0, "right": 0, "steps": 1}]}),
        state_at=lambda time: SimpleNamespace(pose=truth),
    )


def test_delivered_capture_time_scoring_and_dropout_gap():
    log = replay(
        [
            reading(0, 0, 0),
            reading(1, 0.5, None),
            reading(2, 1, 1000),
            reading(3, 2, 2000, delay=0.1),
        ]
    )
    result = analyze_odometry(log, "encoders")
    assert result["total_estimates"] == 2 and result["dropped_readings"] == 1
    assert result["last_capture_time"] == 1
    expected = 2 * 3.141592653589793 * log.config.robot.wheel_radius
    assert result["metrics"]["final_position_error_m"] == pytest.approx(expected)
    assert result["metrics"]["position_rmse_m"] == pytest.approx(expected / 2**0.5)
    shifted = analyze_odometry(replay(log.readings, truth=Pose2(1, 1, 1)), "encoders")
    assert [s["estimate"] for s in result["samples"]] == [s["estimate"] for s in shifted["samples"]]
    assert result["metrics"] != shifted["metrics"]


def test_sampling_preserves_endpoints_and_full_metrics():
    log = replay([reading(i, i / 10, i * 10) for i in range(21)])
    full = analyze_odometry(log, "encoders")
    sampled = analyze_odometry(log, "encoders", 2)
    assert sampled["sampled"] and sampled["metrics"] == full["metrics"]
    assert sampled["samples"] == [full["samples"][0], full["samples"][-1]]


@pytest.mark.parametrize(
    "readings", [[], [reading(0, 0, None), reading(1, 1, 1)], [reading(0, 0, 0), reading(0, 1, 1)]]
)
def test_unusable_encoder_stream_is_rejected(readings):
    with pytest.raises(ServiceError):
        analyze_odometry(replay(readings), "encoders")


def test_api_reports_slip_drift_without_creating_another_run(tmp_path):
    config = RunConfig.model_validate(
        {
            "commands": [{"left": 4, "right": 4, "steps": 100}],
            "sensors": [{"type": "encoder", "rate_hz": 100}],
            "faults": [
                {
                    "name": "slip",
                    "kind": "wheel_slip",
                    "target": "left",
                    "start": 0.1,
                    "end": 0.8,
                    "magnitude": 0.5,
                }
            ],
        }
    )
    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.service
        run = finished(service, service.submit(config)["id"])
        url = f"/api/runs/{run['id']}/odometry"
        response = client.get(url, params={"sensor": "encoders", "max_points": 2})
        assert response.status_code == 200
        data = response.json()
        assert data["metrics"]["final_position_error_m"] > 0.01
        assert data["sampled"] and len(data["samples"]) == 2
        assert "attachment" in response.headers["content-disposition"]
        assert client.get(url, params={"sensor": "bad"}).status_code == 422
        assert client.get(url, params={"sensor": "encoders", "max_points": 2001}).status_code == 422
        assert service.list()["total"] == 1
