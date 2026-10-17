import json
import runpy

import pytest

from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_firmware import ObstacleBrakeFirmware
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


def limit_program(program_type, scenario, **kwargs):
    return program_type(
        brake_threshold_m=scenario.brake_threshold_m,
        timeout_action=scenario.timeout_action,
        **kwargs,
    )


class RecordingFirmware:
    def __init__(self, scenario):
        self.policy = ObstacleBrakeFirmware(
            brake_threshold_m=scenario.brake_threshold_m, timeout_action=scenario.timeout_action
        )
        self.frames = []

    def on_range(self, observation, inputs):
        return self.policy.on_range(observation, inputs)

    def on_tick(self, inputs):
        return None

    def on_imu(self, observation, inputs):
        self.frames.append((observation, inputs))
        return None


def test_firmware_receives_only_decoded_completed_frames_and_delivered_encoder_counts(
    model, scenario
):
    program = RecordingFirmware(scenario)
    report = run_rover_brake(model, scenario, firmware=program)
    readings = report["imu"]["readings"]
    assert len(program.frames) == len(readings) > 100
    assert report["firmware"]["imu_callback"] is True
    assert report["firmware"]["imu_callback_count"] == len(readings)
    seen_acceleration = []
    for (observation, inputs), reading in zip(program.frames, readings):
        assert set(observation.model_dump()) == {
            "time_s",
            "sample_time_s",
            "specific_force_m_s2",
            "angular_velocity_rad_s",
            "temperature_c",
        }
        assert observation.time_s == inputs.time_s == reading["time_s"]
        assert observation.sample_time_s == reading["transfer"]["sample_time_s"]
        assert observation.sample_time_s < inputs.time_s
        assert observation.specific_force_m_s2 == tuple(reading["decoded"]["specific_force_m_s2"])
        assert observation.angular_velocity_rad_s == tuple(
            reading["decoded"]["angular_velocity_rad_s"]
        )
        assert observation.temperature_c == reading["decoded"]["temperature_c"]
        # Counts at delivery are between those of the surrounding physics endpoints.
        before = max(
            (s for s in report["samples"] if s["time_s"] <= inputs.time_s),
            key=lambda s: s["time_s"],
        )
        after = min(
            (s for s in report["samples"] if s["time_s"] >= inputs.time_s),
            key=lambda s: s["time_s"],
        )
        assert (
            before["encoder_counts"]["left"]
            <= inputs.left_encoder_count
            <= after["encoder_counts"]["left"]
        )
        assert (
            before["encoder_counts"]["right"]
            <= inputs.right_encoder_count
            <= after["encoder_counts"]["right"]
        )
        seen_acceleration.append(observation.specific_force_m_s2[0])
    assert max(seen_acceleration) > 0 and min(seen_acceleration) < 0


def test_acceleration_program_brakes_after_read_delivery_at_the_next_control_tick(
    model, scenario, program_type
):
    config = scenario.model_copy(update={"control_every_steps": 20})
    report = run_rover_brake(model, config, firmware=limit_program(program_type, config))
    first = report["imu"]["readings"][0]
    metrics = report["metrics"]
    assert report["status"] == "stopped_before_wall"
    assert report["brake_reason"] == "acceleration_limit"
    assert first["decoded"]["specific_force_m_s2"][0] >= 2
    assert first["transfer"]["sample_time_s"] == 0.01
    assert metrics["detection_time_s"] == first["time_s"] == 0.01153
    assert metrics["brake_command_time_s"] == 0.02
    assert metrics["controller_response_delay_s"] == pytest.approx(0.02 - 0.01153)
    assert all(
        s["driver_modes"] == ["brake", "brake"] for s in report["samples"] if s["time_s"] > 0.02
    )
    message = next(
        json.loads(t["text"])
        for t in report["serial"]["completed"]
        if json.loads(t["text"])["event"] == "brake"
    )
    assert message["reason"] == "acceleration_limit" and message["time_s"] == 0.02


@pytest.mark.parametrize(
    "end,count,status",
    [
        (0.0001, 0, "completed_without_brake"),
        (0.0105, 0, "completed_without_brake"),
        (0.01153, 1, "brake_pending"),
    ],
)
def test_initializing_or_pending_reads_never_reach_firmware_early(
    model, scenario, program_type, end, count, status
):
    config = scenario.model_copy(update={"duration_s": end})
    report = run_rover_brake(model, config, firmware=limit_program(program_type, config))
    assert report["firmware"]["imu_callback_count"] == count
    assert len(report["imu"]["readings"]) == count
    assert report["status"] == status
    assert report["metrics"]["brake_command_time_s"] is None
    assert report["metrics"]["detection_time_s"] == (0.01153 if count else None)


def test_serial_disabled_or_decimated_does_not_disable_firmware_imu_delivery(model, scenario):
    config = scenario.model_copy(update={"duration_s": 0.04})
    program = RecordingFirmware(config)
    report = run_rover_brake(model, config, firmware=program)
    full_console = config.serial.model_copy(
        update={
            "console": config.serial.console.model_copy(
                update={"baud_rate": 100, "buffer_bytes": 200}
            )
        }
    )
    for serial in (
        None,
        config.serial.model_copy(update={"imu_every_readings": 10000}),
        full_console,
    ):
        other = RecordingFirmware(config)
        result = run_rover_brake(
            model, config.model_copy(update={"serial": serial}), firmware=other
        )
        assert other.frames == program.frames
        assert result["firmware"]["imu_callback_count"] == report["firmware"]["imu_callback_count"]
        assert result["metrics"] == report["metrics"]
        if serial is full_console:
            assert result["serial"]["dropped_writes"]


def test_stale_frames_are_rejected_by_example_policy_while_pending_reads_stay_pending(
    model, scenario, program_type
):
    data = model.model_dump()
    data["i2c_buses"][0]["latency_s"] = 0.03
    config = scenario.model_copy(update={"duration_s": 0.16})
    report = run_rover_brake(
        RobotProject.model_validate(data),
        config,
        firmware=limit_program(program_type, config, maximum_forward_acceleration_m_s2=0.1),
    )
    assert report["firmware"]["imu_callback_count"] >= 1
    for reading in report["imu"]["readings"]:
        assert reading["time_s"] - reading["transfer"]["sample_time_s"] > 0.02
        assert reading["decoded"]["specific_force_m_s2"][0] > 0.1
    assert report["status"] == "completed_without_brake" and report["brake_reason"] is None
    assert report["imu"]["pending_transfer"]["completion_time_s"] > 0.16


def test_imu_limit_can_be_disabled_by_threshold_and_sonar_still_brakes(
    model, scenario, program_type
):
    baseline = run_rover_brake(model, scenario)
    report = run_rover_brake(
        model,
        scenario,
        firmware=limit_program(program_type, scenario, maximum_forward_acceleration_m_s2=100),
    )
    assert report["brake_reason"] == "obstacle"
    for field in (
        "metrics",
        "samples",
        "readings",
        "control_decisions",
        "ultrasonic_pin_events",
        "imu",
        "serial",
    ):
        assert report[field] == baseline[field]


def test_missing_imu_does_not_invent_firmware_observations(model, scenario, program_type):
    config = scenario.model_copy(update={"imu": None, "duration_s": 0.03})
    report = run_rover_brake(model, config, firmware=limit_program(program_type, config))
    assert report["imu"] is None
    assert report["firmware"]["imu_callback"] is True
    assert report["firmware"]["imu_callback_count"] == 0
    assert report["brake_reason"] is None


def test_imu_callback_failure_aborts_the_run(model, scenario):
    class Broken(RecordingFirmware):
        def on_imu(self, observation, inputs):
            raise RuntimeError("failed IMU controller")

    with pytest.raises(ValueError, match="host firmware on_imu failed: failed IMU controller"):
        run_rover_brake(model, scenario, firmware=Broken(scenario))
