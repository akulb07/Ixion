from copy import deepcopy

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.build_reference import verify_build_reference, write_build_reference
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_comparison import compare_rover_suites
from roboforge.hardware.rover_report import project_rover_evidence
from roboforge.hardware.rover_suite import RoverSuiteCase, load_rover_suite, run_rover_suite
from roboforge.hardware.voltage_firmware import VoltageBrakeFirmware
from roboforge.hardware.voltage_monitor import VoltageMonitorConfig
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    return load_rover_scenario(REFERENCE.with_name("rover_wall.yaml"))


@pytest.fixture(scope="module")
def report(model, scenario):
    return run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "voltage_monitor": VoltageMonitorConfig(conversion_s=0.003),
                "control_every_steps": 20,
            }
        ),
        firmware=VoltageBrakeFirmware(brake_threshold_m=scenario.brake_threshold_m),
    )


def test_voltage_is_delivered_before_a_control_tick_applies_firmware_braking(report):
    assert report["schema_version"] == 12
    assert report["status"] == "stopped_before_wall"
    assert report["brake_reason"] == "low_voltage"
    assert report["metrics"]["detection_time_s"] == 0.003
    assert report["metrics"]["brake_command_time_s"] == 0.02
    assert report["metrics"]["controller_response_delay_s"] == 0.017
    assert report["power_limit_event"] is None and report["power_response"] is None
    assert report["firmware"]["voltage_callback_count"] == len(
        report["voltage_monitor"]["readings"]
    )
    first = report["voltage_monitor"]["readings"][0]
    assert first["observation"]["sample_time_s"] == 0
    assert first["source_voltage_v"] == report["samples"][0]["pack_voltage_v"]
    projected = project_rover_evidence(report)["cases"][0]["run"]
    assert projected["voltage_monitor"] == report["voltage_monitor"]
    assert any(e["kind"] == "voltage" and e["time_s"] == 0.003 for e in projected["events"])


def test_pending_conversion_does_not_reach_firmware_at_early_run_end(model, scenario):
    result = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "voltage_monitor": VoltageMonitorConfig(conversion_s=0.01),
                "duration_s": 0.005,
            }
        ),
        firmware=VoltageBrakeFirmware(brake_threshold_m=scenario.brake_threshold_m),
    )
    assert not result["voltage_monitor"]["readings"]
    assert result["voltage_monitor"]["pending"]["observation"]["time_s"] == 0.01
    assert result["firmware"]["voltage_callback_count"] == 0
    assert result["metrics"]["detection_time_s"] is None
    project_rover_evidence(result)


@pytest.mark.parametrize(
    "field",
    [
        "version",
        "count",
        "source",
        "latency",
        "overrange",
        "callback_count",
        "callback_float",
        "callback_flag",
        "callback_interface",
        "pending",
        "configuration",
    ],
)
def test_edited_monitor_evidence_is_rejected(report, field):
    data = deepcopy(report)
    capture = data["voltage_monitor"]["readings"][0]
    if field == "version":
        data["schema_version"] = 11
    elif field == "count":
        capture["observation"]["raw_count"] += 1
    elif field == "source":
        capture["source_voltage_v"] += 1
    elif field == "latency":
        capture["observation"]["time_s"] += 0.0001
    elif field == "overrange":
        capture["observation"]["over_range"] = True
    elif field == "callback_count":
        data["firmware"]["voltage_callback_count"] += 1
    elif field == "callback_float":
        data["firmware"]["voltage_callback_count"] = float(
            data["firmware"]["voltage_callback_count"]
        )
    elif field == "callback_flag":
        data["firmware"]["voltage_callback"] = False
    elif field == "callback_interface":
        data["firmware"]["interface_version"] = 4
    elif field == "pending":
        data["voltage_monitor"]["pending"] = deepcopy(capture)
    else:
        data["voltage_monitor"]["config"]["resolution_bits"] = 8
    with pytest.raises(ValueError):
        project_rover_evidence(data)


@pytest.mark.parametrize(
    "change,reason",
    [
        ({"conversion_s": 0.02}, "voltage_sample_stale"),
        ({"full_scale_v": 5}, "voltage_monitor_range"),
    ],
)
def test_guard_brakes_on_stale_or_saturated_observations(model, scenario, change, reason):
    result = run_rover_brake(
        model,
        scenario.model_copy(update={"voltage_monitor": VoltageMonitorConfig(**change)}),
        firmware=VoltageBrakeFirmware(brake_threshold_m=scenario.brake_threshold_m),
    )
    assert result["brake_reason"] == reason
    assert result["status"] == "stopped_before_wall"
    project_rover_evidence(result)


def test_monitor_is_passive_without_a_voltage_callback_and_legacy_encoding_stays_unchanged(
    model, scenario
):
    assert "voltage_monitor" not in scenario.model_dump()
    baseline = run_rover_brake(model, scenario)
    observed = run_rover_brake(
        model, scenario.model_copy(update={"voltage_monitor": VoltageMonitorConfig()})
    )
    for metric in (
        "final_distance_m",
        "final_speed_m_s",
        "brake_command_time_s",
        "terminal_energy_j",
    ):
        assert observed["metrics"][metric] == baseline["metrics"][metric]
    assert observed["firmware"] == baseline["firmware"]
    project_rover_evidence(observed)


@pytest.mark.parametrize("action", ["end", "coast", "brake"])
def test_sampled_monitor_preserves_independent_voltage_boundary_and_response(
    model, scenario, action
):
    config = scenario.model_copy(
        update={
            "minimum_pack_voltage_v": 8.39,
            "voltage_limit_action": action,
            "duration_s": 2.4,
        }
    )
    baseline = run_rover_brake(model, config)
    observed = run_rover_brake(
        model, config.model_copy(update={"voltage_monitor": VoltageMonitorConfig()})
    )
    assert observed["schema_version"] == 12
    assert observed["status"] == baseline["status"]
    assert observed["power_limit_event"] == baseline["power_limit_event"]
    for field in ("final_distance_m", "final_speed_m_s", "terminal_energy_j", "power_limit_time_s"):
        assert observed["metrics"][field] == baseline["metrics"][field]
    project_rover_evidence(observed)


def test_feedback_suite_compares_and_exports_the_actual_guard_settings(model, tmp_path):
    suite = load_rover_suite(REFERENCE.with_name("rover_voltage_feedback_suite.yaml"))
    result = run_rover_suite(model, suite)
    assert result["summary"] == {"total": 4, "passed": 4, "failed": 0, "error": 0}
    runs = {row["name"]: row["report"] for row in result["cases"]}
    assert runs["fresh_voltage"]["metrics"]["brake_command_time_s"] == 0.02
    assert runs["slow_voltage"]["metrics"]["brake_command_time_s"] == 0.04
    assert (
        runs["slow_voltage"]["metrics"]["final_distance_m"]
        > runs["fresh_voltage"]["metrics"]["final_distance_m"]
    )
    assert runs["slow_voltage"]["voltage_monitor"]["busy_skips"] > 0
    assert compare_rover_suites(result, result)["status"] == "passed"
    output = tmp_path / "feedback.zip"
    write_build_reference(output, model, suite, "fresh_voltage")
    assert verify_build_reference(output)["schema_version"] == 6


def test_voltage_callback_latches_before_a_tick_at_the_same_timestamp(model, scenario):
    report = run_rover_brake(
        model,
        scenario.model_copy(
            update={
                "voltage_monitor": VoltageMonitorConfig(conversion_s=0.02),
                "control_every_steps": 20,
            }
        ),
        firmware=VoltageBrakeFirmware(
            brake_threshold_m=scenario.brake_threshold_m, maximum_sample_age_s=0.02
        ),
    )
    assert report["brake_reason"] == "low_voltage"
    assert (
        report["metrics"]["detection_time_s"] == report["metrics"]["brake_command_time_s"] == 0.02
    )


@pytest.mark.parametrize(
    "controller,guard,monitor",
    [
        ("voltage_guard", None, None),
        ("sonar", {"minimum_voltage_v": 8}, None),
        ("voltage_guard", {"minimum_voltage_v": 8}, None),
    ],
)
def test_guard_configuration_cannot_silently_run_without_its_monitor(
    scenario, controller, guard, monitor
):
    with pytest.raises(ValueError, match="voltage_guard"):
        RoverSuiteCase.model_validate(
            {
                "name": "invalid",
                "controller": controller,
                "voltage_guard": guard,
                "scenario": {**scenario.model_dump(), "voltage_monitor": monitor},
            }
        )
