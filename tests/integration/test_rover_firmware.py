import json
import runpy

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_firmware import BrakeRequest
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    return load_rover_scenario(REFERENCE.with_name("rover_wall_debug.yaml"))


@pytest.fixture(scope="module")
def encoder_program():
    return runpy.run_path(str(REFERENCE.with_name("rover_encoder_firmware.py")))[
        "EncoderLimitFirmware"
    ]


@pytest.fixture(scope="module")
def encoder_report(model, scenario, encoder_program):
    return run_rover_brake(model, scenario, firmware=encoder_program())


def test_separate_encoder_program_brakes_from_delivered_counts(encoder_report):
    report = encoder_report
    assert report["status"] == "stopped_before_wall"
    assert report["brake_reason"] == "encoder_limit"
    assert report["firmware"]["custom_program"] is True
    assert report["firmware"]["parameters"] == {"target_count": 6000}
    metrics = report["metrics"]
    assert metrics["controller_response_delay_s"] == 0
    decisions = report["control_decisions"]
    first = next(i for i, d in enumerate(decisions) if d["command"] == "brake")
    assert min(decisions[first]["encoder_counts"].values()) >= 6000
    assert min(decisions[first - 1]["encoder_counts"].values()) < 6000
    assert decisions[first]["time_s"] == metrics["brake_command_time_s"]
    samples = {s["time_s"]: s for s in report["samples"]}
    assert (
        decisions[first]["encoder_counts"] == samples[decisions[first]["time_s"]]["encoder_counts"]
    )
    assert report["imu"]["readings"] and report["ultrasonic_pin_events"]
    message = next(
        json.loads(t["text"])
        for t in report["serial"]["completed"]
        if json.loads(t["text"])["event"] == "brake"
    )
    assert message["reason"] == "encoder_limit"
    assert message["time_s"] == metrics["brake_command_time_s"]


def test_custom_program_is_repeatable_with_a_fresh_instance(
    model, scenario, encoder_program, encoder_report
):
    assert run_rover_brake(model, scenario, firmware=encoder_program()) == encoder_report


class IgnoreRange:
    def on_range(self, observation, inputs):
        return None

    def on_tick(self, inputs):
        return None


def test_custom_program_replaces_default_threshold_policy(model, scenario):
    report = run_rover_brake(
        model, scenario.model_copy(update={"duration_s": 0.8}), firmware=IgnoreRange()
    )
    assert report["brake_reason"] is None
    assert report["status"] == "completed_without_brake"
    assert min(r["distance_m"] for r in report["readings"]) < scenario.brake_threshold_m
    assert all(d["command"] == "drive" for d in report["control_decisions"])


def test_observations_only_expose_delivered_inputs_and_wait_for_control_boundary(model, scenario):
    class FirstEcho(IgnoreRange):
        def on_range(self, observation, inputs):
            assert set(inputs.model_dump()) == {
                "time_s",
                "left_encoder_count",
                "right_encoder_count",
            }
            assert set(observation.model_dump()) == {"time_s", "status", "distance_m"}
            assert observation.time_s == inputs.time_s
            return BrakeRequest(reason="first_echo")

    report = run_rover_brake(
        model,
        scenario.model_copy(update={"duration_s": 0.03, "control_every_steps": 20}),
        firmware=FirstEcho(),
    )
    metrics = report["metrics"]
    assert report["brake_reason"] == "first_echo"
    assert metrics["detection_time_s"] == report["readings"][0]["time_s"]
    assert metrics["brake_command_time_s"] == 0.02
    assert 0 < metrics["controller_response_delay_s"] < 0.02


@pytest.mark.parametrize("end,expected", [(0.030009, None), (0.03001, "missing_echo")])
def test_timeout_callback_delivers_at_exact_deadline_with_no_hidden_end_step_delay(
    model, scenario, end, expected
):
    class MissingEcho(IgnoreRange):
        def on_range(self, observation, inputs):
            assert observation.time_s == inputs.time_s
            assert observation.status == "timeout" and observation.distance_m is None
            return BrakeRequest(reason="missing_echo")

    report = run_rover_brake(
        model,
        scenario.model_copy(update={"wall_distance_m": 5, "duration_s": end}),
        firmware=MissingEcho(),
    )
    assert report["brake_reason"] == expected
    if expected is None:
        assert report["readings"] == [] and report["pending_measurement"] is not None
    else:
        deadline = report["readings"][0]["deadline_s"]
        assert report["status"] == "brake_pending"
        assert report["metrics"]["detection_time_s"] == deadline
        log = next(
            t
            for t in report["serial"]["pending"]
            if json.loads(t["text"])["event"] == "sonar_timeout"
        )
        assert log["requested_time_s"] == deadline


def test_firmware_error_aborts_instead_of_returning_a_successful_report(model, scenario):
    class Broken(IgnoreRange):
        def on_tick(self, inputs):
            raise RuntimeError("broken controller")

    with pytest.raises(ValueError, match="host firmware on_tick failed: broken controller"):
        run_rover_brake(model, scenario, firmware=Broken())


def test_echo_completed_at_the_timeout_deadline_wins(model, scenario):
    distance = scenario.wall_distance_m - scenario.sensor_offset_m
    sensor = type(scenario.sensor).model_validate(
        {
            **scenario.sensor.model_dump(),
            "maximum_distance_m": distance,
            "timeout_s": scenario.sensor.response_delay_s
            + 2 * distance / scenario.sensor.sound_speed_m_s,
        }
    )
    report = run_rover_brake(
        model, scenario.model_copy(update={"sensor": sensor, "duration_s": 0.01})
    )
    assert len(report["readings"]) == 1
    reading = report["readings"][0]
    assert reading["status"] == "echo"
    assert reading["time_s"] == reading["deadline_s"]
    assert report["pending_measurement"] is None
    assert report["brake_reason"] is None
