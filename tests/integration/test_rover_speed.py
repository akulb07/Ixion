import math

import pytest

from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.encoder_speed import EncoderSpeedConfig
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.speed_firmware import EncoderSpeedFirmware
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    source = load_rover_scenario(REFERENCE.with_name("rover_wall_debug.yaml"))
    return type(source).model_validate(
        {**source.model_dump(), "duration_s": 4, "control_every_steps": 20}
    )


def run(model, scenario):
    controller = EncoderSpeedFirmware(
        encoder=EncoderSpeedConfig.from_project(model, scenario.left_motor, scenario.right_motor),
        brake_threshold_m=scenario.brake_threshold_m,
        timeout_action=scenario.timeout_action,
    )
    report = run_rover_brake(model, scenario, firmware=controller)
    report["speed_control"] = controller.report()
    return report


@pytest.fixture(scope="module")
def report(model, scenario):
    return run(model, scenario)


def test_speed_controller_tracks_reference_target_and_retains_sonar_stop(report):
    assert report["status"] == "stopped_before_wall" and report["brake_reason"] == "obstacle"
    assert report["metrics"]["final_front_clearance_m"] > 0.4
    trace = report["speed_control"]["samples"]
    driving = [s for s in trace if not s["braking"] and s["observation"] is not None]
    assert abs(driving[-1]["observation"]["mean_speed_m_s"] - 0.2) < 0.02
    assert max(s["target_speed_m_s"] for s in driving) == 0.2
    assert report["speed_control"]["metrics"]["post_ramp_tracking_rmse_m_s"] < 0.02
    assert report["serial"]["dropped_writes"] == []
    assert report["imu"]["readings"] and report["readings"]


def test_estimated_speed_uses_observed_count_windows_with_expected_quantization_error(
    report, model
):
    samples = {s["time_s"]: s for s in report["samples"]}
    for sample in report["speed_control"]["samples"]:
        observation = sample["observation"]
        if observation is None:
            continue
        before, after = samples[observation["interval_start_s"]], samples[observation["time_s"]]
        delta = [
            after["encoder_counts"][name] - before["encoder_counts"][name]
            for name in ("left", "right")
        ]
        assert observation["count_delta"] == delta
        average = (after["distance_m"] - before["distance_m"]) / observation["interval_s"]
        bound = math.tau * model.assembly.wheel_radius_m / 4800 / observation["interval_s"]
        assert abs(observation["mean_speed_m_s"] - average) <= bound + 1e-12


def test_controller_trace_matches_applied_pwm_and_freezes_integral_while_braking(report):
    decisions = {d["time_s"]: d for d in report["control_decisions"]}
    trace = report["speed_control"]["samples"]
    config = report["speed_control"]["config"]
    driving = [s for s in trace if not s["braking"]]
    for sample in driving:
        assert sample["requested_pwm_duty"] == decisions[sample["time_s"]]["pwm_duty"]
        assert (
            config["minimum_pwm_duty"] <= sample["requested_pwm_duty"] <= config["maximum_pwm_duty"]
        )
    for before, after in zip(driving, driving[1:]):
        maximum = config["maximum_duty_rate_per_s"] * (after["time_s"] - before["time_s"])
        assert abs(after["requested_pwm_duty"] - before["requested_pwm_duty"]) <= maximum + 1e-12
    braking = [s for s in trace if s["braking"]]
    assert braking
    assert all(
        s["requested_pwm_duty"] is None and decisions[s["time_s"]]["command"] == "brake"
        for s in braking
    )
    assert len({s["integral_error_m"] for s in braking}) == 1
    assert report["firmware"]["drive_requests_ignored_after_brake"] == 0


def test_saved_tracking_metric_excludes_ramp_and_braking_and_uses_observed_speed(report):
    control = report["speed_control"]
    selected = [
        s
        for s in control["samples"]
        if s["observation"] is not None
        and not s["braking"]
        and s["time_s"] >= control["config"]["target_ramp_s"]
    ]
    rmse = math.sqrt(
        sum((s["observation"]["mean_speed_m_s"] - s["target_speed_m_s"]) ** 2 for s in selected)
        / len(selected)
    )
    assert control["metrics"]["post_ramp_observation_count"] == len(selected)
    assert control["metrics"]["post_ramp_tracking_rmse_m_s"] == pytest.approx(rmse)


def test_extra_mass_changes_encoder_feedback_and_the_controllers_power_requests(model, scenario):
    config = scenario.model_copy(update={"duration_s": 0.8})
    light = run(model, config)
    data = model.model_dump()
    data["assembly"]["total_mass_kg"] *= 2
    heavy = run(RobotProject.model_validate(data), config)
    light_trace = light["speed_control"]["samples"]
    heavy_trace = heavy["speed_control"]["samples"]
    assert any(
        heavy_sample["requested_pwm_duty"] > light_sample["requested_pwm_duty"] + 0.001
        for light_sample, heavy_sample in zip(light_trace, heavy_trace)
    )
    assert (
        heavy["control_decisions"][-1]["encoder_counts"]["left"]
        < light["control_decisions"][-1]["encoder_counts"]["left"]
    )


def test_console_backpressure_cannot_change_speed_control_observations_or_commands(model, scenario):
    config = scenario.model_copy(update={"duration_s": 0.4})
    baseline = run(model, config)
    full = config.serial.model_copy(
        update={
            "console": config.serial.console.model_copy(
                update={"baud_rate": 100, "buffer_bytes": 120}
            )
        }
    )
    for serial in (None, full):
        result = run(model, config.model_copy(update={"serial": serial}))
        for field in (
            "speed_control",
            "metrics",
            "samples",
            "imu",
            "readings",
            "control_decisions",
        ):
            assert result[field] == baseline[field]
        if serial is full:
            assert result["serial"]["dropped_writes"]


def test_fresh_speed_controller_runs_are_deterministic(model, scenario, report):
    assert run(model, scenario) == report
