import json

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.encoder_faults import EncoderCountFault
from roboforge.hardware.encoder_health import EncoderHealthConfig
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


def run(model, scenario, health=True):
    controller = EncoderSpeedFirmware(
        encoder=EncoderSpeedConfig.from_project(model, scenario.left_motor, scenario.right_motor),
        encoder_health=EncoderHealthConfig() if health else None,
        brake_threshold_m=scenario.brake_threshold_m,
        timeout_action=scenario.timeout_action,
    )
    result = run_rover_brake(model, scenario, firmware=controller)
    result["speed_control"] = controller.report()
    return result


def with_faults(scenario, profile, time=0.8):
    if profile == "left-jump":
        faults = (
            EncoderCountFault(
                channel="left", start_time_s=time, kind="offset", offset_counts=10_000
            ),
        )
    else:
        channels = ("left", "right") if profile == "both-freeze" else ("left",)
        faults = tuple(EncoderCountFault(channel=name, start_time_s=time) for name in channels)
    return type(scenario).model_validate({**scenario.model_dump(), "encoder_faults": faults})


@pytest.fixture(scope="module", params=["left-freeze", "both-freeze", "left-jump"])
def fault_run(request, model, scenario):
    config = with_faults(scenario, request.param)
    return request.param, config, run(model, config)


def test_fault_profiles_brake_on_observed_evidence_and_keep_raw_decoder_counts(fault_run):
    profile, config, report = fault_run
    expected = {
        "left-freeze": ("encoder_disagreement", 0.92),
        "both-freeze": ("encoder_no_progress", 1.02),
        "left-jump": ("encoder_speed_implausible", 0.8),
    }
    code, detection = expected[profile]
    assert report["schema_version"] == 7
    assert report["status"] == "stopped_before_wall" and report["brake_reason"] == code
    metrics = report["metrics"]
    assert metrics["detection_time_s"] == pytest.approx(detection)
    assert metrics["brake_command_time_s"] == metrics["detection_time_s"]
    assert metrics["final_front_clearance_m"] > 0.7
    assert metrics["final_speed_m_s"] <= config.stopped_speed_m_s
    assert report["encoder_feedback"]["layer"] == "decoded_counter_read"
    assert [a["time_s"] for a in report["encoder_feedback"]["activations"]] == [0.8] * len(
        config.encoder_faults
    )
    samples = report["samples"]
    final_raw = samples[-1]["decoded_encoder_counts"]
    assert report["encoder_edge_count"] == sum(final_raw.values())
    assert final_raw["left"] == final_raw["right"]
    activation = report["encoder_feedback"]["activations"][0]
    assert final_raw["left"] > activation["decoded_count_at_activation"]
    assert any(s["encoder_counts"] != s["decoded_encoder_counts"] for s in samples)
    for sample in samples:
        if sample["time_s"] < 0.8:
            assert sample["encoder_counts"] == sample["decoded_encoder_counts"]
        elif profile == "left-jump":
            assert (
                sample["encoder_counts"]["left"]
                == sample["decoded_encoder_counts"]["left"] + 10_000
            )
        else:
            assert sample["encoder_counts"]["left"] == activation["decoded_count_at_activation"]


def test_fault_evidence_matches_count_readings_and_pi_freezes_after_detection(fault_run):
    _, _, report = fault_run
    control = report["speed_control"]
    evidence = control["encoder_health"]["fault"]
    observation = evidence["observation"]
    decisions = {s["time_s"]: s for s in report["control_decisions"]}
    before, after = decisions[observation["interval_start_s"]], decisions[observation["time_s"]]
    assert observation["count_delta"] == [
        after["encoder_counts"][name] - before["encoder_counts"][name] for name in ("left", "right")
    ]
    trace = control["samples"]
    brake_index = next(i for i, sample in enumerate(trace) if sample["braking"])
    assert trace[brake_index]["time_s"] == evidence["time_s"]
    integral = trace[brake_index - 1]["integral_error_m"]
    assert evidence["requested_pwm_duty"] == trace[brake_index - 1]["requested_pwm_duty"]
    for sample in trace[brake_index:]:
        assert sample["requested_pwm_duty"] is None
        assert sample["integral_error_m"] == integral
        assert sample["feedback_fault_reason"] == evidence["code"]
        assert decisions[sample["time_s"]]["command"] == "brake"
    assert report["firmware"]["drive_requests_ignored_after_brake"] == 0
    assert report["metrics"]["stop_time_s"] > evidence["time_s"]


def test_normal_feedback_checks_leave_motion_commands_and_console_unchanged(model, scenario):
    guarded, unchecked = run(model, scenario), run(model, scenario, health=False)
    assert guarded["speed_control"]["encoder_health"] == {"fault": None}
    assert unchecked["speed_control"]["encoder_health"] is None
    for field in ("metrics", "samples", "readings", "imu", "serial", "control_decisions"):
        assert guarded[field] == unchecked[field]
    assert guarded["speed_control"]["samples"] == unchecked["speed_control"]["samples"]
    assert all(s["encoder_counts"] == s["decoded_encoder_counts"] for s in guarded["samples"])


def test_fault_brake_does_not_depend_on_serial_acceptance_or_a_console(model, scenario):
    config = with_faults(scenario, "both-freeze")
    baseline = run(model, config)
    full = config.serial.model_copy(
        update={
            "console": config.serial.console.model_copy(
                update={"baud_rate": 100, "buffer_bytes": 120}
            )
        }
    )
    for serial in (None, full):
        report = run(model, config.model_copy(update={"serial": serial}))
        for field in (
            "speed_control",
            "encoder_feedback",
            "metrics",
            "samples",
            "readings",
            "control_decisions",
        ):
            assert report[field] == baseline[field]
        if serial is full:
            assert report["serial"]["dropped_writes"]
    events = [json.loads(m["text"]) for m in baseline["serial"]["completed"]]
    assert len([e for e in events if e["event"] == "encoder_counter_fault"]) == 2
    assert any(e["event"] == "firmware_log" and e["level"] == "error" for e in events)


def test_fault_at_run_horizon_activates_without_an_extra_control_tick(model, scenario):
    short = scenario.model_copy(update={"duration_s": 0.8})
    normal = run(model, short)
    faulted = run(model, with_faults(short, "left-jump"))
    assert faulted["encoder_feedback"]["activations"][0]["time_s"] == 0.8
    assert faulted["metrics"] == normal["metrics"]
    assert faulted["control_decisions"] == normal["control_decisions"]
    assert faulted["speed_control"] == normal["speed_control"]
    assert (
        faulted["samples"][-1]["encoder_counts"]["left"]
        == normal["samples"][-1]["encoder_counts"]["left"] + 10_000
    )
    assert faulted["speed_control"]["encoder_health"]["fault"] is None


def test_initial_count_offset_is_not_detectable_from_count_differences(model, scenario):
    short = scenario.model_copy(update={"duration_s": 0.4})
    baseline = run(model, short)
    offset = run(model, with_faults(short, "left-jump", time=0))
    assert offset["speed_control"] == baseline["speed_control"]
    assert offset["metrics"] == baseline["metrics"]
    assert offset["control_decisions"][0]["encoder_counts"] == {"left": 10_000, "right": 0}


def test_ignored_count_fault_cannot_change_physics_or_other_sensor_signals(model, scenario):
    short = scenario.model_copy(update={"duration_s": 0.15})
    normal = run_rover_brake(model, short)
    faulted = run_rover_brake(model, with_faults(short, "both-freeze", time=0.05))
    for field in ("metrics", "imu", "readings", "ultrasonic_pin_events", "encoder_edge_count"):
        assert faulted[field] == normal[field]
    for before, after in zip(normal["samples"], faulted["samples"], strict=True):
        assert {k: v for k, v in before.items() if k != "encoder_counts"} == {
            k: v for k, v in after.items() if k != "encoder_counts"
        }
    assert faulted["samples"][-1]["encoder_counts"] != normal["samples"][-1]["encoder_counts"]


def test_negative_count_jump_trips_reverse_check_before_the_next_power_update(model, scenario):
    fault = EncoderCountFault(
        channel="right", start_time_s=0.8, kind="offset", offset_counts=-10_000
    )
    config = type(scenario).model_validate({**scenario.model_dump(), "encoder_faults": (fault,)})
    result = run(model, config)
    evidence = result["speed_control"]["encoder_health"]["fault"]
    assert result["brake_reason"] == evidence["code"] == "encoder_reverse"
    assert evidence["wheels"] == ["right"] and evidence["time_s"] == 0.8
    assert result["metrics"]["brake_command_time_s"] == 0.8


def test_faulted_runs_are_deterministic(model, fault_run):
    _, config, report = fault_run
    assert run(model, config) == report
