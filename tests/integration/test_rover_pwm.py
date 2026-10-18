import json
import runpy

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_firmware import BrakeRequest, DriveRequest, ObstacleBrakeFirmware
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    return load_rover_scenario(REFERENCE.with_name("rover_wall_debug.yaml"))


@pytest.fixture(scope="module")
def ramp_type():
    return runpy.run_path(str(REFERENCE.with_name("rover_pwm_firmware.py")))["RampedStartFirmware"]


class LowPower(ObstacleBrakeFirmware):
    def on_tick(self, inputs):
        return DriveRequest(pwm_duty=0.25)


def test_initial_sample_uses_the_first_command_for_current_traction_and_energy(model, scenario):
    config = scenario.model_copy(update={"duration_s": 0.1})
    low = run_rover_brake(model, config, firmware=LowPower(brake_threshold_m=0.6))
    full = run_rover_brake(model, config)
    assert low["samples"][0]["driver_pwm_duty"] == [0.25, 0.25]
    assert low["control_decisions"][0]["pwm_duty"] == 0.25
    assert low["control_decisions"][0]["drive_request_time_s"] == 0
    assert low["samples"][0]["pack_current_a"] < full["samples"][0]["pack_current_a"]
    assert (
        low["samples"][0]["contact_force_per_wheel_n"]
        < full["samples"][0]["contact_force_per_wheel_n"]
    )
    assert low["metrics"]["terminal_energy_j"] < full["metrics"]["terminal_energy_j"]
    assert low["metrics"]["final_distance_m"] < full["metrics"]["final_distance_m"]
    assert all(s["driver_pwm_duty"] == [0.25, 0.25] for s in low["samples"])


def test_soft_start_can_be_valid_when_full_power_exceeds_initial_traction(model, scenario):
    config = scenario.model_copy(update={"friction_coefficient": 0.1, "duration_s": 0.03})
    with pytest.raises(ValueError, match="traction exceeded"):
        run_rover_brake(model, config)
    result = run_rover_brake(model, config, firmware=LowPower(brake_threshold_m=0.6))
    assert result["status"] == "completed_without_brake"
    assert all(
        abs(s["contact_force_per_wheel_n"]) <= s["traction_limit_per_wheel_n"]
        for s in result["samples"]
    )


@pytest.mark.parametrize("end,applied", [(0.006, False), (0.03, True)])
def test_sensor_duty_request_waits_until_control_tick_or_remains_pending(
    model, scenario, end, applied
):
    class RangePower(ObstacleBrakeFirmware):
        def on_range(self, observation, inputs):
            return DriveRequest(pwm_duty=0.5)

    config = scenario.model_copy(update={"duration_s": end, "control_every_steps": 20})
    result = run_rover_brake(model, config, firmware=RangePower(brake_threshold_m=0.6))
    arrival = result["readings"][0]["time_s"]
    assert all(s["driver_pwm_duty"] == [1, 1] for s in result["samples"] if s["time_s"] <= 0.02)
    if applied:
        decision = result["control_decisions"][1]
        assert decision["time_s"] == 0.02 and decision["pwm_duty"] == 0.5
        assert decision["drive_request_time_s"] == arrival
        assert all(
            s["driver_pwm_duty"] == [0.5, 0.5] for s in result["samples"] if s["time_s"] > 0.02
        )
        assert result["firmware"]["pending_drive_request"] is None
    else:
        assert result["firmware"]["pwm_duty"] == 1
        assert result["firmware"]["pending_drive_request"] == {"pwm_duty": 0.5, "time_s": arrival}


def test_ramp_example_changes_both_pwm_channels_and_cannot_override_sonar_braking(
    model, scenario, ramp_type
):
    config = scenario.model_copy(update={"control_every_steps": 20})
    result = run_rover_brake(model, config, firmware=ramp_type(brake_threshold_m=0.6))
    assert result["schema_version"] == 7 and result["status"] == "stopped_before_wall"
    assert result["brake_reason"] == "obstacle"
    drive = [d for d in result["control_decisions"] if d["command"] == "drive"]
    assert drive[0]["pwm_duty"] == 0.25 and drive[-1]["pwm_duty"] == 1
    assert all(a["pwm_duty"] <= b["pwm_duty"] for a, b in zip(drive, drive[1:]))
    brake_time = result["metrics"]["brake_command_time_s"]
    assert all(
        s["driver_modes"] == ["brake", "brake"] and s["driver_pwm_duty"] == [1, 1]
        for s in result["samples"]
        if s["time_s"] > brake_time
    )
    assert result["firmware"]["drive_requests_ignored_after_brake"] > 0
    changes = [
        json.loads(t["text"])
        for t in result["serial"]["completed"]
        if json.loads(t["text"])["event"] == "drive_pwm"
    ]
    assert changes and changes[0]["time_s"] == 0 and changes[0]["pwm_duty"] == 0.25
    decisions = {d["time_s"]: d for d in drive}
    assert all(e["pwm_duty"] == decisions[e["time_s"]]["pwm_duty"] for e in changes)
    assert result["serial"]["dropped_writes"] == []


def test_immediate_brake_has_no_phantom_full_power_initial_sample(model, scenario):
    class Stop(ObstacleBrakeFirmware):
        def on_tick(self, inputs):
            return BrakeRequest(reason="initial_stop")

    result = run_rover_brake(model, scenario, firmware=Stop(brake_threshold_m=0.6))
    assert result["status"] == "stopped_before_wall"
    assert result["metrics"]["brake_command_time_s"] == 0
    assert result["metrics"]["peak_sampled_pack_current_a"] == 0
    assert all(
        s["driver_modes"] == ["brake", "brake"] and s["pack_current_a"] == 0
        for s in result["samples"]
    )


def test_early_power_reduction_is_supported_when_it_does_not_require_regeneration(model, scenario):
    class EarlyReduction(ObstacleBrakeFirmware):
        def on_tick(self, inputs):
            return DriveRequest(pwm_duty=1 if inputs.time_s < 0.01 else 0.5)

    result = run_rover_brake(
        model,
        scenario.model_copy(update={"duration_s": 0.08}),
        firmware=EarlyReduction(brake_threshold_m=0.6),
    )
    assert result["status"] == "completed_without_brake"
    assert result["control_decisions"][10]["pwm_duty"] == 0.5
    assert all(s["pack_current_a"] >= 0 for s in result["samples"])


def test_large_power_reduction_at_speed_rejects_unmodeled_regeneration(model, scenario):
    class SharpReduction(ObstacleBrakeFirmware):
        def on_tick(self, inputs):
            return DriveRequest(pwm_duty=1 if inputs.time_s < 0.4 else 0.01)

    with pytest.raises(ValueError, match="regenerative channel"):
        run_rover_brake(
            model,
            scenario.model_copy(update={"duration_s": 0.42}),
            firmware=SharpReduction(brake_threshold_m=0.6),
        )


def test_serial_disabled_does_not_change_ramped_motion_or_sensor_delivery(
    model, scenario, ramp_type
):
    config = scenario.model_copy(update={"control_every_steps": 20, "duration_s": 0.1})
    with_console = run_rover_brake(model, config, firmware=ramp_type(brake_threshold_m=0.6))
    without = run_rover_brake(
        model, config.model_copy(update={"serial": None}), firmware=ramp_type(brake_threshold_m=0.6)
    )
    for field in (
        "metrics",
        "samples",
        "readings",
        "control_decisions",
        "imu",
        "ultrasonic_pin_events",
    ):
        assert with_console[field] == without[field]
