"""Declared driver responses after a voltage boundary, with supplies retained."""

import math
from copy import deepcopy

import pytest

from roboforge.cli import main
from roboforge.hardware import load_project
from roboforge.hardware.build_reference import verify_build_reference, write_build_reference
from roboforge.hardware.motor import MotorDatasheet
from roboforge.hardware.rolling import RollingBody, rolling_inertia
from roboforge.hardware.rover_brake import (
    RoverBrakeConfig,
    RoverSerialConfig,
    load_rover_scenario,
    run_rover_brake,
)
from roboforge.hardware.rover_comparison import compare_rover_suites
from roboforge.hardware.rover_firmware import DriveRequest
from roboforge.hardware.rover_imu import RoverIMUConfig
from roboforge.hardware.rover_report import project_rover_evidence
from roboforge.hardware.rover_suite import RoverSuiteConfig, load_rover_suite, run_rover_suite
from roboforge.hardware.serial_console import SerialConsoleConfig
from tests.unit.test_hardware_project import REFERENCE


class Jump:
    def on_tick(self, inputs):
        return DriveRequest(pwm_duty=0.2 if inputs.time_s < 0.02 else 1)

    def on_range(self, observation, inputs):
        return None


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    return load_rover_scenario(REFERENCE.with_name("rover_wall.yaml"))


@pytest.fixture(scope="module")
def tiny(model):
    return model.model_copy(
        update={
            "components": tuple(
                part.model_copy(
                    update={
                        "parameters": tuple(
                            p.model_copy(update={"value": 1e-5}) if p.name == "capacity" else p
                            for p in part.parameters
                        )
                    }
                )
                if part.id == "battery"
                else part
                for part in model.components
            )
        }
    )


@pytest.fixture(scope="module")
def brake(model, scenario):
    return run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 8,
                "voltage_limit_action": "brake",
            }
        ),
        firmware=Jump(),
    )


def test_command_boundary_response_latches_and_stops_with_supplies_retained(brake):
    assert brake["schema_version"] == 11
    assert brake["status"] == "stopped_after_power_limit"
    assert brake["power_limit_event"]["phase"] == "command_boundary"
    assert brake["power_response"]["action"] == "brake"
    time = brake["power_limit_event"]["time_s"]
    assert time == 0.02
    assert brake["metrics"]["stop_time_s"] > time
    assert 0 < brake["metrics"]["final_speed_m_s"] <= 0.005
    assert brake["metrics"]["distance_after_power_limit_m"] > 0
    assert brake["metrics"]["energy_after_power_limit_j"] == 0
    later = [s for s in brake["samples"] if s["time_s"] > time]
    assert all(s["driver_modes"] == ["brake", "brake"] for s in later)
    assert all(s["pack_current_a"] == 0 for s in later)
    assert all(s["pack_voltage_v"] > 8 for s in later)  # Rebound does not release the latch.
    assert all(
        d["motor_override"] == "brake" for d in brake["control_decisions"] if d["time_s"] > time
    )
    assert brake["control_decisions"][-1]["command"] == "drive"
    view = project_rover_evidence(brake)["cases"][0]["run"]
    assert len([e for e in view["events"] if e["kind"] == "power"]) == 2


def test_interior_crossing_splits_step_and_preserves_the_poll_and_control_grid(tiny, scenario):
    result = run_rover_brake(
        tiny,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 6.4,
                "voltage_limit_action": "brake",
            }
        ),
        firmware=Jump(),
    )
    assert result["status"] == "stopped_after_power_limit"
    event = result["power_limit_event"]
    assert event["phase"] == "physics_step"
    times = [s["time_s"] for s in result["samples"]]
    assert event["time_s"] in times
    after = times[times.index(event["time_s"]) + 1]
    assert after == pytest.approx((int(event["time_s"] / scenario.step_s) + 1) * scenario.step_s)
    assert len(times) == len(set(times))
    assert all(
        d["time_s"] / scenario.step_s == pytest.approx(round(d["time_s"] / scenario.step_s))
        for d in result["control_decisions"]
    )
    project_rover_evidence(result)


def test_response_at_startup_does_not_invent_motion_or_time(model, scenario):
    result = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 8,
                "voltage_limit_action": "coast",
            }
        ),
    )
    assert result["status"] == "stopped_after_power_limit"
    assert result["metrics"]["stop_time_s"] == 0
    assert result["metrics"]["power_response_elapsed_s"] == 0
    assert result["metrics"]["distance_after_power_limit_m"] == 0
    assert result["metrics"]["energy_after_power_limit_j"] == 0
    assert len(result["samples"]) == 1
    project_rover_evidence(result)


def test_coast_can_finish_the_horizon_still_moving(model, scenario):
    result = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 8,
                "voltage_limit_action": "coast",
                "duration_s": 0.1,
            }
        ),
        firmware=Jump(),
    )
    assert result["status"] == "power_response_incomplete"
    assert result["metrics"]["final_speed_m_s"] > 0.005
    assert result["metrics"]["stop_time_s"] is None
    assert result["samples"][-1]["time_s"] == 0.1
    assert result["samples"][-1]["driver_modes"] == ["coast", "coast"]
    project_rover_evidence(result)


@pytest.mark.parametrize("action", ["coast", "brake"])
def test_action_requires_a_limit_and_default_encoding_stays_unchanged(action, scenario):
    with pytest.raises(ValueError, match="requires a declared voltage limit"):
        RoverBrakeConfig.model_validate({**scenario.model_dump(), "voltage_limit_action": action})
    assert "voltage_limit_action" not in scenario.model_dump()


def test_unreached_response_keeps_normal_behavior(model, scenario):
    result = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 6,
                "voltage_limit_action": "brake",
            }
        ),
    )
    assert result["status"] == "stopped_before_wall"
    assert result["power_response"] is None
    assert result["metrics"]["distance_after_power_limit_m"] is None
    project_rover_evidence(result)


@pytest.mark.parametrize(
    "field",
    [
        "time",
        "action",
        "inputs",
        "distance",
        "energy",
        "elapsed",
        "status",
        "stop",
        "latch",
        "version",
        "missing",
    ],
)
def test_viewer_rejects_inconsistent_response_evidence(brake, field):
    data = deepcopy(brake)
    response = data["power_response"]
    if field == "time":
        response["time_s"] += 0.001
    elif field == "action":
        response["action"] = "coast"
    elif field == "inputs":
        response["motor_inputs"][0]["in2"] = False
    elif field == "distance":
        response["distance_m"] += 0.1
    elif field == "energy":
        response["terminal_energy_j"] += 0.1
    elif field == "elapsed":
        data["metrics"]["power_response_elapsed_s"] += 0.1
    elif field == "status":
        data["status"] = "power_limit_reached"
    elif field == "stop":
        data["metrics"]["stop_time_s"] = None
    elif field == "latch":
        data["samples"][-1]["driver_modes"] = ["forward", "forward"]
    elif field == "version":
        data["schema_version"] = 10
    else:
        del data["power_response"]
    with pytest.raises(ValueError):
        project_rover_evidence(data)


def test_suite_comparison_and_build_reference_preserve_response_policy(model, scenario, tmp_path):
    config = RoverSuiteConfig.model_validate(
        {
            "name": "response",
            "cases": [
                {
                    "name": "startup_brake",
                    "controller": "sonar",
                    "scenario": {
                        **scenario.model_dump(),
                        "minimum_pack_voltage_v": 8,
                        "voltage_limit_action": "brake",
                    },
                    "acceptance": {
                        "expected_status": "stopped_after_power_limit",
                        "maximum_power_limit_time_s": 0,
                        "maximum_final_speed_m_s": 0,
                    },
                }
            ],
        }
    )
    result = run_rover_suite(model, config)
    assert result["summary"] == {"total": 1, "passed": 1, "failed": 0, "error": 0}
    assert compare_rover_suites(result, result)["status"] == "passed"
    archive = tmp_path / "response.zip"
    write_build_reference(archive, model, config, "startup_brake")
    assert verify_build_reference(archive)["schema_version"] == 5


@pytest.mark.parametrize("action", ["brake", "coast"])
def test_wall_contact_after_response_retains_the_limit_and_approach_speed(model, scenario, action):
    result = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 8,
                "voltage_limit_action": action,
                "wall_distance_m": scenario.front_offset_m + 0.0002,
            }
        ),
        firmware=Jump(),
    )
    assert result["status"] == "wall_contact"
    assert result["metrics"]["wall_contact_time_s"] > result["power_response"]["time_s"]
    assert result["metrics"]["final_speed_m_s"] > 0.005
    assert abs(result["metrics"]["final_front_clearance_m"]) < 1e-12
    project_rover_evidence(result)


def test_earlier_contact_does_not_fabricate_a_later_response(model, scenario):
    result = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 8,
                "voltage_limit_action": "brake",
                "wall_distance_m": scenario.front_offset_m + 1e-6,
            }
        ),
        firmware=Jump(),
    )
    assert result["status"] == "wall_contact"
    assert result["power_response"] is None
    assert result["power_limit_event"] is None
    project_rover_evidence(result)


@pytest.mark.parametrize("buffer", [1, 8192])
def test_auxiliary_energy_and_peripheral_timing_continue_after_the_limit(model, scenario, buffer):
    result = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 8,
                "voltage_limit_action": "brake",
                "auxiliary_current_a": 0.25,
                "imu": RoverIMUConfig(temperature_c=25),
                "serial": RoverSerialConfig(console=SerialConsoleConfig(buffer_bytes=buffer)),
            }
        ),
        firmware=Jump(),
    )
    assert result["status"] == "stopped_after_power_limit"
    response = result["power_response"]
    assert response["time_s"] == 0.02
    assert result["metrics"]["energy_after_power_limit_j"] > 0
    assert any(r["time_s"] > response["time_s"] for r in result["readings"])
    assert any(r["time_s"] > response["time_s"] for r in result["imu"]["readings"])
    assert all(
        s["pack_current_a"] == 0.25 for s in result["samples"] if s["time_s"] > response["time_s"]
    )
    if buffer == 1:
        assert result["serial"]["dropped_writes"]
    else:
        events = [
            item
            for state in ("completed", "pending")
            for item in result["serial"][state]
            if '"event":"power_response"' in item["text"]
        ]
        assert len(events) == 1
        assert events[0]["requested_time_s"] == response["time_s"]
    project_rover_evidence(result)


def test_packaged_response_suite_keeps_stopped_and_unfinished_results_distinct(model):
    suite = load_rover_suite(REFERENCE.with_name("rover_power_response_suite.yaml"))
    result = run_rover_suite(model, suite)
    assert result["summary"] == {"total": 3, "passed": 3, "failed": 0, "error": 0}
    runs = {r["name"]: r["report"] for r in result["cases"]}
    assert runs["sag_brake"]["status"] == "stopped_after_power_limit"
    assert runs["sag_coast"]["status"] == "power_response_incomplete"
    assert (
        runs["sag_brake"]["metrics"]["distance_after_power_limit_m"]
        < runs["sag_coast"]["metrics"]["distance_after_power_limit_m"]
    )
    compare_rover_suites(result, result)


def test_response_cli_saves_success_for_a_stopped_response(model, scenario, tmp_path):
    import json

    project = tmp_path / "project.json"
    config = tmp_path / "scenario.json"
    output = tmp_path / "run.json"
    project.write_text(model.model_dump_json())
    config.write_text(
        scenario.model_copy(
            update={"minimum_pack_voltage_v": 8, "voltage_limit_action": "brake"}
        ).model_dump_json()
    )
    assert main(["rover-brake", str(project), str(config), "--output", str(output)]) == 0
    assert json.loads(output.read_text())["status"] == "stopped_after_power_limit"


@pytest.mark.parametrize("action", ["coast", "brake"])
def test_post_response_motion_matches_independent_linear_decay_model(model, scenario, action):
    duration = 0.1
    result = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 8,
                "voltage_limit_action": action,
                "duration_s": duration,
            }
        ),
        firmware=Jump(),
    )
    motor = MotorDatasheet.from_component(
        next(p for p in model.components if p.id == "left_motor")
    ).derive()
    body = RollingBody.model_validate(result["body"])
    damping = motor.rotor_friction_nm_per_rad_s
    if action == "brake":
        damping += motor.motor_constant_si**2 / (motor.winding_resistance_ohm + 0.5)
    # Positive-speed backdrive reflects rotor losses through inverse gear efficiency.
    rate = damping * motor.gear_ratio**2 / motor.gear_efficiency / rolling_inertia(body)
    boundary = next(s for s in result["samples"] if s["time_s"] == 0.02)
    elapsed = duration - boundary["time_s"]
    steps = round(elapsed / scenario.step_s)
    decay = (1 - rate * scenario.step_s) ** steps
    expected_speed = boundary["speed_m_s"] * decay
    expected_distance = boundary["speed_m_s"] * (1 - decay) / rate
    assert result["metrics"]["final_speed_m_s"] == pytest.approx(expected_speed, rel=1e-12)
    assert result["metrics"]["distance_after_power_limit_m"] == pytest.approx(
        expected_distance, rel=1e-12
    )
    exact_speed = boundary["speed_m_s"] * math.exp(-rate * elapsed)
    assert abs(expected_speed - exact_speed) / exact_speed < 0.001
    assert all(s["pack_current_a"] == 0 for s in result["samples"] if s["time_s"] > 0.02)


def test_step_refinement_reduces_response_decay_error(model, scenario):
    motor = MotorDatasheet.from_component(
        next(p for p in model.components if p.id == "left_motor")
    ).derive()
    errors = []
    for factor in (1, 2, 4):
        result = run_rover_brake(
            model,
            scenario.model_copy(
                update={
                    "minimum_pack_voltage_v": 8,
                    "voltage_limit_action": "brake",
                    "duration_s": 0.1,
                    "step_s": scenario.step_s / factor,
                    "poll_every_steps": 61 * factor,
                    "control_every_steps": factor,
                }
            ),
            firmware=Jump(),
        )
        body = RollingBody.model_validate(result["body"])
        rate = (
            (
                motor.rotor_friction_nm_per_rad_s
                + motor.motor_constant_si**2 / (motor.winding_resistance_ohm + 0.5)
            )
            * motor.gear_ratio**2
            / motor.gear_efficiency
            / rolling_inertia(body)
        )
        time = result["power_response"]["time_s"]
        initial = next(s["speed_m_s"] for s in result["samples"] if s["time_s"] == time)
        reference = initial * math.exp(-rate * (0.1 - time))
        errors.append(abs(result["metrics"]["final_speed_m_s"] - reference) / reference)
    assert errors[1] < errors[0] * 0.51
    assert errors[2] < errors[1] * 0.51


@pytest.mark.parametrize(
    "field",
    [
        "time",
        "speed",
        "soc",
        "power",
        "current",
        "mode",
        "interval_energy",
        "total_energy",
        "peak",
        "initial",
    ],
)
def test_response_interval_evidence_rejects_edited_energy_or_command_boundaries(brake, field):
    data = deepcopy(brake)
    raw = next(s for s in data["samples"] if s["time_s"] > data["power_response"]["time_s"])
    start = raw["interval_start"]
    if field == "time":
        start["time_s"] += 0.0001
    elif field == "speed":
        start["speed_m_s"] += 0.01
    elif field == "soc":
        start["soc"] = 0.5
    elif field == "power":
        start["pack_power_w"] = 1
    elif field == "current":
        start["pack_current_a"] = 1
    elif field == "mode":
        start["driver_modes"] = ["forward", "forward"]
    elif field == "interval_energy":
        raw["interval_terminal_energy_j"] = 1
    elif field == "total_energy":
        data["metrics"]["terminal_energy_j"] += 1
        data["metrics"]["energy_after_power_limit_j"] += 1
    elif field == "peak":
        data["metrics"]["peak_sampled_pack_current_a"] += 1
    else:
        data["samples"][0]["interval_start"] = deepcopy(start)
    with pytest.raises(ValueError):
        project_rover_evidence(data)


def test_response_interval_retains_command_change_and_auditable_energy(brake):
    time = brake["power_response"]["time_s"]
    boundary = next(s for s in brake["samples"] if s["time_s"] == time)
    later = next(s for s in brake["samples"] if s["time_s"] > time)
    assert boundary["driver_modes"] == ["forward", "forward"]
    assert boundary["pack_power_w"] > 0
    assert later["interval_start"]["time_s"] == time
    assert later["interval_start"]["driver_modes"] == ["brake", "brake"]
    assert later["interval_start"]["pack_power_w"] == 0
    assert later["interval_terminal_energy_j"] == 0
    projected = project_rover_evidence(brake)["cases"][0]["run"]["samples"]
    assert projected[brake["samples"].index(later)]["interval_start"] == later["interval_start"]


def test_retained_auxiliary_load_can_exhaust_the_pack_after_coast_and_remains_an_error(
    tiny, scenario
):
    config = scenario.model_copy(
        update={
            "minimum_pack_voltage_v": 8.1,
            "voltage_limit_action": "coast",
            "auxiliary_current_a": 1,
            "stopped_speed_m_s": 1e-9,
            "duration_s": 0.1,
        }
    )
    with pytest.raises(ValueError, match="step exceeds remaining charge"):
        run_rover_brake(tiny, config, firmware=Jump())
