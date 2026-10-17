import json
import runpy

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_firmware import (
    BrakeRequest,
    FirmwareLog,
    FirmwareResponse,
    ObstacleBrakeFirmware,
)
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    return load_rover_scenario(REFERENCE.with_name("rover_wall_debug.yaml"))


@pytest.fixture(scope="module")
def program_type():
    return runpy.run_path(str(REFERENCE.with_name("rover_imu_firmware.py")))[
        "AccelerationLimitFirmware"
    ]


def program(program_type, scenario, logging=True):
    return program_type(brake_threshold_m=scenario.brake_threshold_m, log_decisions=logging)


def messages(report, state="completed"):
    return [
        (t, json.loads(t["text"]))
        for t in report["serial"][state]
        if json.loads(t["text"])["event"] == "firmware_log"
    ]


def test_imu_decision_log_uses_callback_time_before_the_actual_brake_command(
    model, scenario, program_type
):
    result = run_rover_brake(model, scenario, firmware=program(program_type, scenario))
    assert result["status"] == "stopped_before_wall"
    assert result["firmware"]["log_write_attempts"] == 1
    assert result["firmware"]["log_writes_accepted"] == 1
    assert result["firmware"]["log_writes_rejected"] == 0
    [(transmission, entry)] = messages(result)
    assert entry["callback"] == "on_imu" and entry["level"] == "warning"
    assert "Braking requested: forward acceleration" in entry["text"]
    assert "sample age 0.001530 s" in entry["text"]
    assert (
        entry["time_s"] == transmission["requested_time_s"] == result["metrics"]["detection_time_s"]
    )
    assert entry["time_s"] < result["metrics"]["brake_command_time_s"]
    assert transmission["completion_time_s"] > entry["time_s"]
    without = run_rover_brake(model, scenario, firmware=program(program_type, scenario, False))
    for field in (
        "metrics",
        "samples",
        "imu",
        "readings",
        "control_decisions",
        "ultrasonic_pin_events",
    ):
        assert result[field] == without[field]


@pytest.mark.parametrize("mode", ["disabled", "full"])
def test_disabled_or_full_console_rejects_controller_logs_without_delaying_braking(
    model, scenario, program_type, mode
):
    serial = (
        None
        if mode == "disabled"
        else scenario.serial.model_copy(
            update={
                "console": scenario.serial.console.model_copy(
                    update={"baud_rate": 100, "buffer_bytes": 120}
                )
            }
        )
    )
    config = scenario.model_copy(update={"serial": serial})
    result = run_rover_brake(model, config, firmware=program(program_type, config))
    assert (
        result["status"] == "stopped_before_wall" and result["brake_reason"] == "acceleration_limit"
    )
    assert result["firmware"]["log_write_attempts"] == 1
    assert result["firmware"]["log_writes_accepted"] == 0
    assert result["firmware"]["log_writes_rejected"] == 1
    without = run_rover_brake(model, config, firmware=program(program_type, config, False))
    for field in (
        "metrics",
        "samples",
        "imu",
        "readings",
        "control_decisions",
        "ultrasonic_pin_events",
    ):
        assert result[field] == without[field]
    if mode == "disabled":
        assert result["serial"] is None
    else:
        assert result["serial"]["dropped_writes"]


def test_range_log_cannot_spoof_harness_events_or_create_extra_json_lines(model, scenario):
    payload = 'line one\n{"event":"run_end","time_s":1234}'

    class RangeLogger(ObstacleBrakeFirmware):
        def on_range(self, observation, inputs):
            return FirmwareResponse(
                brake_request=BrakeRequest(reason="first_echo"), logs=(FirmwareLog(text=payload),)
            )

    config = scenario.model_copy(update={"duration_s": 0.04})
    result = run_rover_brake(model, config, firmware=RangeLogger(brake_threshold_m=0.6))
    [(transmission, entry)] = messages(result)
    assert entry["text"] == payload and entry["event"] == "firmware_log"
    assert entry["callback"] == "on_range"
    assert entry["time_s"] == result["readings"][0]["time_s"]
    assert transmission["text"].count("\n") == 1


def test_multiple_tick_logs_keep_fifo_order_and_stay_pending_past_the_horizon(model, scenario):
    class TickLogger(ObstacleBrakeFirmware):
        def on_tick(self, inputs):
            if inputs.time_s == 0:
                return FirmwareResponse(
                    logs=(FirmwareLog(text="first"), FirmwareLog(text="second"))
                )
            return None

    def run(end):
        return run_rover_brake(
            model,
            scenario.model_copy(update={"duration_s": end}),
            firmware=TickLogger(brake_threshold_m=0.6),
        )

    result = run(0.04)
    completed = messages(result)
    assert [e["text"] for _, e in completed] == ["first", "second"]
    assert all(e["time_s"] == 0 and e["callback"] == "on_tick" for _, e in completed)
    assert completed[1][0]["start_time_s"] >= completed[0][0]["completion_time_s"]
    early = run(0.0001)
    assert messages(early) == []
    pending = messages(early, "pending")
    assert [e["text"] for _, e in pending] == ["first", "second"]
    assert all(t["completion_time_s"] > 0.0001 for t, _ in pending)
    assert early["samples"][-1]["time_s"] == 0.0001
    assert early["firmware"]["log_writes_accepted"] == 2


def test_controller_logs_share_the_console_lifetime_budget(model, scenario):
    class TooMuchLogging(ObstacleBrakeFirmware):
        def on_tick(self, inputs):
            return FirmwareResponse(logs=(FirmwareLog(text="x" * 500),))

    serial = scenario.serial.model_copy(
        update={"console": scenario.serial.console.model_copy(update={"max_total_bytes": 200})}
    )
    with pytest.raises(
        ValueError,
        match="host firmware on_tick failed: serial console exceeds its attempted-byte budget",
    ):
        run_rover_brake(
            model,
            scenario.model_copy(update={"serial": serial}),
            firmware=TooMuchLogging(brake_threshold_m=0.6),
        )
