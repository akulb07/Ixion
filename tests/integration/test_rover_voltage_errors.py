from copy import deepcopy

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.build_reference import verify_build_reference, write_build_reference
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_comparison import compare_rover_suites
from roboforge.hardware.rover_report import project_rover_evidence
from roboforge.hardware.rover_suite import load_rover_suite, run_rover_suite
from roboforge.hardware.voltage_firmware import VoltageBrakeFirmware
from roboforge.hardware.voltage_monitor import VoltageErrorConfig, VoltageMonitorConfig
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def suite_result():
    project = load_project(REFERENCE)
    suite = load_rover_suite(REFERENCE.with_name("rover_voltage_error_suite.yaml"))
    return project, suite, run_rover_suite(project, suite)


def test_gain_offset_and_noise_change_decisions_from_observed_voltage(suite_result):
    _, _, result = suite_result
    assert result["summary"] == {"total": 6, "passed": 6, "failed": 0, "error": 0}
    runs = {row["name"]: row["report"] for row in result["cases"]}
    low, high = runs["ideal_low"], runs["gain_reads_high"]
    assert low["samples"][0]["pack_voltage_v"] == high["samples"][0]["pack_voltage_v"] < 8
    assert low["brake_reason"] == "low_voltage" and high["brake_reason"] == "obstacle"
    assert high["metrics"]["detection_time_s"] > low["metrics"]["detection_time_s"]
    assert high["voltage_monitor"]["readings"][0]["observation"]["voltage_v"] > 8
    assert runs["ideal_above_threshold"]["samples"][0]["pack_voltage_v"] > 7.5
    assert runs["ideal_above_threshold"]["brake_reason"] == "obstacle"
    assert runs["offset_reads_low"]["brake_reason"] == "low_voltage"
    assert runs["noise_seed_0"]["metrics"]["detection_time_s"] == 0.003
    assert runs["noise_seed_1"]["metrics"]["detection_time_s"] == 0.033
    assert (
        runs["noise_seed_1"]["metrics"]["final_distance_m"]
        > runs["noise_seed_0"]["metrics"]["final_distance_m"]
    )
    project_rover_evidence(result)
    assert compare_rover_suites(result, result)["status"] == "passed"


def test_error_conditions_survive_build_export_and_require_the_new_packet_version(
    suite_result, tmp_path
):
    project, suite, _ = suite_result
    path = tmp_path / "sensor-errors.zip"
    write_build_reference(path, project, suite, "gain_reads_high")
    assert verify_build_reference(path)["schema_version"] == 7


def test_neutral_errors_preserve_physics_and_controller_observations():
    model = load_project(REFERENCE)
    scenario = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml"))

    def run(errors):
        return run_rover_brake(
            model,
            scenario.model_copy(update={"voltage_monitor": VoltageMonitorConfig(errors=errors)}),
            firmware=VoltageBrakeFirmware(brake_threshold_m=scenario.brake_threshold_m),
        )

    ideal, neutral = run(None), run(VoltageErrorConfig())
    for field in ("samples", "metrics", "control_decisions", "firmware"):
        assert ideal[field] == neutral[field]
    assert [c["observation"] for c in ideal["voltage_monitor"]["readings"]] == [
        c["observation"] for c in neutral["voltage_monitor"]["readings"]
    ]
    project_rover_evidence(neutral)


@pytest.mark.parametrize("edit", ["draw", "source", "config", "model"])
def test_viewer_rejects_changed_error_evidence_in_saved_suite(suite_result, edit):
    data = deepcopy(suite_result[2])
    row = next(row for row in data["cases"] if row["name"] == "noise_seed_0")
    monitor = row["report"]["voltage_monitor"]
    if edit == "draw":
        monitor["readings"][0]["noise_v"] *= -1
    elif edit == "source":
        monitor["readings"][0]["unclipped_voltage_v"] += 0.1
    elif edit == "config":
        monitor["config"]["errors"]["seed"] = 99
    else:
        monitor["model"] = "ideal_pack_voltage_monitor"
    with pytest.raises(ValueError):
        project_rover_evidence(data)
