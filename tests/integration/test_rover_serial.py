import json

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    return load_rover_scenario(REFERENCE.with_name("rover_wall_debug.yaml"))


@pytest.fixture(scope="module")
def report(model, scenario):
    return run_rover_brake(model, scenario)


def test_shared_console_records_delivered_controller_and_sensor_events(report, scenario):
    assert report["schema_version"] == 6 and report["status"] == "stopped_before_wall"
    serial = report["serial"]
    assert serial["dropped_writes"] == []
    messages = [json.loads(t["text"]) for t in serial["completed"]]
    assert {"run_start", "status", "imu_ready", "imu_read", "sonar_echo", "brake"} <= {
        m["event"] for m in messages
    }
    end = report["samples"][-1]["time_s"]
    previous = 0
    for transfer, message in zip(serial["completed"], messages):
        assert transfer["requested_time_s"] == message["time_s"]
        assert transfer["start_time_s"] >= transfer["requested_time_s"]
        assert transfer["start_time_s"] >= previous
        assert transfer["completion_time_s"] <= end
        assert transfer["byte_count"] == len(transfer["text"].encode("utf-8"))
        assert transfer["completion_time_s"] - transfer["start_time_s"] == pytest.approx(
            10 * transfer["byte_count"] / scenario.serial.console.baud_rate
        )
        previous = transfer["completion_time_s"]


def test_brake_console_event_has_the_actual_command_time_and_observed_counts(report):
    transmission = next(
        t for t in report["serial"]["completed"] if json.loads(t["text"])["event"] == "brake"
    )
    message = json.loads(transmission["text"])
    assert message["time_s"] == report["metrics"]["brake_command_time_s"]
    decision = next(d for d in report["control_decisions"] if d["time_s"] == message["time_s"])
    assert message["encoder_counts"] == decision["encoder_counts"]
    assert message["reason"] == report["brake_reason"]
    assert transmission["completion_time_s"] > message["time_s"]


def test_imu_logs_reference_delivered_frames_and_keep_sample_age(report):
    frames = {r["time_s"]: r for r in report["imu"]["readings"]}
    messages = [json.loads(t["text"]) for t in report["serial"]["completed"]]
    messages = [m for m in messages if m["event"] == "imu_read"]
    assert len(messages) > 10
    for message in messages:
        frame = frames[message["time_s"]]
        assert message["sample_time_s"] == frame["transfer"]["sample_time_s"]
        assert message["sample_time_s"] < message["time_s"]
        assert message["specific_force_m_s2"] == frame["decoded"]["specific_force_m_s2"]


def test_final_message_is_pending_and_does_not_advance_the_robot_to_drain_serial(report):
    serial = report["serial"]
    end = report["samples"][-1]["time_s"]
    assert serial["pending"]
    final = serial["pending"][-1]
    assert final["requested_time_s"] == end
    assert final["completion_time_s"] > end
    assert json.loads(final["text"])["event"] == "run_end"
    assert all(json.loads(t["text"])["event"] != "run_end" for t in serial["completed"])


def test_logging_does_not_change_rover_motion_sensor_data_or_i2c_timing(model, scenario, report):
    without = run_rover_brake(model, scenario.model_copy(update={"serial": None}))
    assert without["serial"] is None
    for field in (
        "metrics",
        "samples",
        "readings",
        "control_decisions",
        "imu",
        "ultrasonic_pin_events",
    ):
        assert report[field] == without[field]


def test_slow_full_console_drops_messages_while_control_and_sensors_continue(model, scenario):
    slow = scenario.serial.model_copy(
        update={
            "console": scenario.serial.console.model_copy(
                update={"baud_rate": 100, "buffer_bytes": 200}
            )
        }
    )
    config = scenario.model_copy(update={"serial": slow, "duration_s": 0.1})
    result = run_rover_brake(model, config)
    serial = result["serial"]
    assert serial["dropped_writes"] and serial["completed"] == []
    assert serial["buffered_bytes"] <= 200
    assert result["encoder_edge_count"] > 0
    assert result["imu"]["readings"] and result["readings"]
    without = run_rover_brake(model, config.model_copy(update={"serial": None}))
    assert result["metrics"] == without["metrics"]
    assert result["samples"] == without["samples"]


def test_transmission_completing_exactly_at_run_end_is_included(model, scenario, report):
    first_end = report["serial"]["completed"][0]["completion_time_s"]
    result = run_rover_brake(model, scenario.model_copy(update={"duration_s": first_end}))
    serial = result["serial"]
    assert len(serial["completed"]) == 1
    assert serial["completed"][0]["completion_time_s"] == first_end
    assert json.loads(serial["completed"][0]["text"])["event"] == "run_start"
