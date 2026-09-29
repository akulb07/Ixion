from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from roboforge.api import create_app, presets
from roboforge.config import RunConfig
from roboforge.control_analysis import analyze_control
from roboforge.service import ServiceError
from tests.integration.test_service import finished


def replay():
    samples = []
    for i, measurement in enumerate([0, 3, 4]):
        pid = dict(
            setpoint=4,
            measurement=measurement,
            error=4 - measurement,
            proportional=4 - measurement,
            integral=0,
            derivative=0,
            feedforward=4,
            unclamped=8 - measurement,
            output=6 if i == 0 else 8 - measurement,
            saturated=i == 0,
        )
        samples.append(
            dict(
                time=(i + 1) * 0.1,
                capture_time=(i + 1) * 0.1,
                measurement_dt=0.1,
                left=pid.copy(),
                right=pid.copy(),
            )
        )
    return SimpleNamespace(
        config=RunConfig.model_validate(presets()[2]["config"]),
        control_samples=samples,
        states=[SimpleNamespace(time=0.4)],
    )


def test_metrics_use_full_recorded_response_not_plot_subsample():
    result = analyze_control(replay(), 2)
    left = result["metrics"]["left"]
    assert left["tracking_rmse_rad_s"] == pytest.approx((17 / 3) ** 0.5)
    assert left["mean_absolute_error_rad_s"] == pytest.approx(5 / 3)
    assert left["final_error_rad_s"] == 0
    assert left["saturated_update_percent"] == pytest.approx(100 / 3)
    assert left["peak_absolute_output_rad_s"] == 6
    assert result["sampled"] and len(result["samples"]) == 2
    assert result["metrics"] == analyze_control(replay())["metrics"]


@pytest.mark.parametrize(
    "change",
    [
        lambda log: log.control_samples.clear(),
        lambda log: log.control_samples[1].update(capture_time=0.1),
        lambda log: log.control_samples[1].update(time=0.01),
        lambda log: log.control_samples[1]["left"].update(output=float("nan")),
        lambda log: log.control_samples[1]["left"].update(saturated=1),
    ],
)
def test_bad_or_empty_telemetry_is_explicit(change):
    log = replay()
    change(log)
    with pytest.raises(ServiceError):
        analyze_control(log)


def test_api_feedback_and_open_loop_and_isolated_replay_samples(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.service
        for preset in (presets()[2], presets()[0]):
            config = RunConfig.model_validate(preset["config"])
            run = finished(service, service.submit(config)["id"])
            assert run["status"] == "completed"
            url = f"/api/runs/{run['id']}/control"
            result = client.get(url, params={"max_points": 2})
            if config.wheel_controller is None:
                assert result.status_code == 422
                continue
            assert result.status_code == 200
            assert "attachment" in result.headers["content-disposition"]
            data = result.json()
            assert data["total_updates"] > 2 and data["sampled"]
            assert data["settings"]["left"]["kp"] == 0.8
            assert data["metrics"]["left"]["tracking_rmse_rad_s"] > 0
            log = service.replay(run["id"])
            log.control_samples[0]["left"]["output"] = 12345
            assert log.control_samples[0]["left"]["output"] != 12345
            assert client.get(url, params={"max_points": 1}).status_code == 422
        assert service.list()["total"] == 2
