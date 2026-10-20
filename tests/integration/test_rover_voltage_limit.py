import json
from copy import deepcopy

import pytest

from roboforge.cli import main
from roboforge.hardware import load_project
from roboforge.hardware.build_reference import verify_build_reference, write_build_reference
from roboforge.hardware.rover_brake import RoverSerialConfig, load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_comparison import compare_rover_suites
from roboforge.hardware.rover_firmware import DriveRequest
from roboforge.hardware.rover_report import project_rover_evidence
from roboforge.hardware.rover_suite import RoverSuiteConfig, run_rover_suite
from roboforge.hardware.serial_console import SerialConsoleConfig
from tests.unit.test_hardware_project import REFERENCE


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
def falling(tiny, scenario):
    return run_rover_brake(tiny, scenario.model_copy(update={"minimum_pack_voltage_v": 6.4}))


def test_interior_limit_ends_with_motion_evidence_and_no_claim_of_a_stopped_rover(falling):
    assert falling["status"] == "power_limit_reached"
    assert falling["schema_version"] == 10
    event = falling["power_limit_event"]
    assert event["phase"] == "physics_step"
    assert 0.006 < event["time_s"] < 0.007
    assert falling["samples"][-1]["time_s"] == event["time_s"]
    assert falling["samples"][-1]["pack_voltage_v"] > 6.4 >= event["bracket_end_voltage_v"]
    assert falling["metrics"]["final_speed_m_s"] > 0
    assert falling["metrics"]["stop_time_s"] is None
    assert falling["metrics"]["wall_contact_time_s"] is None
    assert falling["metrics"]["power_limit_time_s"] == event["time_s"]
    view = project_rover_evidence(falling)["cases"][0]["run"]
    assert view["events"][-1]["kind"] == "power"
    assert all(e["time_s"] <= event["time_s"] for e in view["events"])


def test_initial_voltage_limit_does_not_fabricate_progress_or_energy(model, scenario):
    result = run_rover_brake(model, scenario.model_copy(update={"minimum_pack_voltage_v": 8}))
    assert result["status"] == "power_limit_reached"
    assert len(result["samples"]) == 1
    assert result["power_limit_event"]["time_s"] == 0
    assert result["metrics"]["terminal_energy_j"] == 0
    assert result["metrics"]["final_distance_m"] == 0
    assert not result["readings"]
    project_rover_evidence(result)


def test_firmware_pwm_jump_can_hit_limit_before_the_next_motion_interval(model, scenario):
    class Jump:
        def on_tick(self, inputs):
            return DriveRequest(pwm_duty=0.2 if inputs.time_s < 0.02 else 1)

        def on_range(self, observation, inputs):
            return None

    result = run_rover_brake(
        model, scenario.model_copy(update={"minimum_pack_voltage_v": 8}), firmware=Jump()
    )
    assert result["power_limit_event"]["phase"] == "command_boundary"
    assert result["power_limit_event"]["time_s"] == 0.02
    assert result["samples"][-1]["driver_pwm_duty"] == [0.2, 0.2]
    assert result["power_limit_event"]["motor_inputs"][0]["pwm_duty"] == 1
    assert result["metrics"]["final_speed_m_s"] > 0
    project_rover_evidence(result)


def test_configured_limit_that_is_not_reached_keeps_the_normal_outcome(model, scenario):
    result = run_rover_brake(model, scenario.model_copy(update={"minimum_pack_voltage_v": 6}))
    assert result["status"] == "stopped_before_wall"
    assert result["power_limit_event"] is None
    assert result["metrics"]["power_limit_time_s"] is None
    project_rover_evidence(result)


def test_explicit_limit_outcomes_can_be_accepted_and_compared_without_rerunning(
    model, scenario, tmp_path
):
    config = RoverSuiteConfig.model_validate(
        {
            "name": "voltage_limit",
            "cases": [
                {
                    "name": "startup_limit",
                    "controller": "sonar",
                    "scenario": {**scenario.model_dump(), "minimum_pack_voltage_v": 8},
                    "acceptance": {
                        "expected_status": "power_limit_reached",
                        "minimum_front_clearance_m": 0.8,
                        "maximum_final_speed_m_s": 0,
                        "maximum_terminal_energy_j": 0,
                        "maximum_power_limit_time_s": 0,
                    },
                }
            ],
        }
    )
    report = run_rover_suite(model, config)
    assert report["status"] == "passed"
    comparison = compare_rover_suites(report, report)
    assert comparison["cases"][0]["metrics"]["power_limit_time_s"]["delta"] == 0
    assert project_rover_evidence(comparison)["cases"]
    archive = tmp_path / "limit.zip"
    write_build_reference(archive, model, config, "startup_limit")
    assert verify_build_reference(archive)["schema_version"] == 4


@pytest.mark.parametrize(
    "change",
    [
        "hidden_version",
        "missing_event",
        "wrong_status",
        "wrong_time",
        "wrong_threshold",
        "not_bracketed",
        "wide_bracket",
        "claims_stop",
        "wrong_sample_voltage",
    ],
)
def test_inconsistent_saved_voltage_limit_is_rejected(falling, change):
    report = deepcopy(falling)
    event = report["power_limit_event"]
    if change == "hidden_version":
        report["schema_version"] = 9
    elif change == "missing_event":
        report["power_limit_event"] = None
    elif change == "wrong_status":
        report["status"] = "stopped_before_wall"
    elif change == "wrong_time":
        report["metrics"]["power_limit_time_s"] = 1
    elif change == "wrong_threshold":
        event["threshold_v"] = 6.3
    elif change == "not_bracketed":
        event["bracket_end_voltage_v"] = 7
    elif change == "wide_bracket":
        event["bracket_end_time_s"] += 0.1
    elif change == "claims_stop":
        report["metrics"]["stop_time_s"] = event["time_s"]
    elif change == "wrong_sample_voltage":
        event["voltage_v"] += 0.1
    with pytest.raises(ValueError):
        project_rover_evidence(report)


def test_cli_saves_limit_evidence_with_a_non_successful_wall_brake_exit(tmp_path, model, scenario):
    source, output = tmp_path / "limit.json", tmp_path / "run.json"
    source.write_text(json.dumps({**scenario.model_dump(), "minimum_pack_voltage_v": 8}))
    assert main(["rover-brake", str(REFERENCE), str(source), "--output", str(output)]) == 3
    assert json.loads(output.read_text())["status"] == "power_limit_reached"


def test_declared_voltage_suite_checks_motion_and_energy_as_well_as_outcomes(model):
    from roboforge.hardware.rover_suite import load_rover_suite

    config = load_rover_suite(REFERENCE.with_name("rover_voltage_limit_suite.yaml"))
    result = run_rover_suite(model, config)
    assert result["status"] == "passed", result["cases"]
    assert [c["report"]["status"] for c in result["cases"]] == [
        "stopped_before_wall",
        "power_limit_reached",
        "power_limit_reached",
    ]
    assert all(c["report"]["schema_version"] == 10 for c in result["cases"])


def test_moving_voltage_boundary_converges_as_the_physics_step_is_reduced(tiny, scenario):
    times = []
    for factor in (1, 2, 4, 8):
        config = scenario.model_copy(
            update={
                "minimum_pack_voltage_v": 6.4,
                "duration_s": 0.1,
                "step_s": 0.001 / factor,
                "poll_every_steps": 61 * factor,
                "control_every_steps": factor,
            }
        )
        result = run_rover_brake(tiny, config)
        assert result["status"] == "power_limit_reached"
        times.append(result["power_limit_event"]["time_s"])
    differences = [abs(a - b) for a, b in zip(times, times[1:])]
    assert differences[1] < differences[0] * 0.75
    assert differences[2] < differences[1] * 0.75


@pytest.mark.parametrize("buffer_bytes", [1, 8192])
def test_voltage_event_is_retained_when_serial_output_is_pending_or_dropped(
    model, scenario, buffer_bytes
):
    serial = RoverSerialConfig(console=SerialConsoleConfig(buffer_bytes=buffer_bytes))
    result = run_rover_brake(
        model, scenario.model_copy(update={"minimum_pack_voltage_v": 8, "serial": serial})
    )
    assert result["status"] == "power_limit_reached"
    assert result["serial"]["completed"] == []
    pending = [json.loads(e["text"]) for e in result["serial"]["pending"]]
    if buffer_bytes > 1:
        assert any(e["event"] == "power_limit" for e in pending)
    else:
        assert pending == []
    assert any(
        e["kind"] == "power" for e in project_rover_evidence(result)["cases"][0]["run"]["events"]
    )


def test_voltage_boundary_does_not_make_an_invalid_initial_driver_supply_supported(model, scenario):
    with pytest.raises(ValueError, match="VM"):
        run_rover_brake(
            model,
            scenario.model_copy(update={"minimum_pack_voltage_v": 6, "auxiliary_current_a": 100}),
        )


@pytest.mark.parametrize(
    "clearance,expected", [(1e-6, "wall_contact"), (1e-4, "power_limit_reached")]
)
def test_first_boundary_wins_between_wall_contact_and_voltage_limit(
    tiny, scenario, clearance, expected
):
    class Continue:
        def on_tick(self, inputs):
            return DriveRequest(pwm_duty=1)

        def on_range(self, observation, inputs):
            return None

    config = scenario.model_copy(
        update={
            "minimum_pack_voltage_v": 6.4,
            "wall_distance_m": scenario.front_offset_m + clearance,
            "duration_s": 0.1,
        }
    )
    result = run_rover_brake(tiny, config, firmware=Continue())
    assert result["status"] == expected
    if expected == "wall_contact":
        assert result["power_limit_event"] is None
        assert result["metrics"]["wall_contact_time_s"] < 0.002
        assert abs(result["metrics"]["final_front_clearance_m"]) < 1e-12
    else:
        assert result["metrics"]["wall_contact_time_s"] is None
        assert result["metrics"]["final_front_clearance_m"] > 0
    project_rover_evidence(result)
