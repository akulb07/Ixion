import json
from copy import deepcopy

import pytest

from roboforge.cli import main
from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel
from roboforge.hardware.bench_trace import (
    BenchMetadata,
    BenchSample,
    BenchTrace,
    compare_bench_trace,
)
from roboforge.hardware.build_reference import verify_build_reference, write_build_reference
from roboforge.hardware.rover_brake import RoverBrakeConfig, load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_comparison import compare_rover_suites
from roboforge.hardware.rover_report import project_rover_evidence
from roboforge.hardware.rover_suite import RoverSuiteConfig, load_rover_suite, run_rover_suite
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def power_suite():
    return load_rover_suite(REFERENCE.with_name("rover_power_suite.yaml"))


@pytest.fixture(scope="module")
def report(model, power_suite):
    return run_rover_suite(model, power_suite)


def test_power_suite_independently_initializes_and_checks_all_three_conditions(report):
    assert report["status"] == "passed"
    assert report["summary"] == {"total": 3, "passed": 3, "failed": 0, "error": 0}
    runs = [c["report"] for c in report["cases"]]
    assert [r["schema_version"] for r in runs] == [7, 9, 9]
    assert [r["samples"][0]["soc"] for r in runs] == [1, 0.5, 0.5]
    for run in runs:
        assert run["status"] == "stopped_before_wall"
        assert run["brake_reason"] == "obstacle"
        assert all(a["soc"] >= b["soc"] for a, b in zip(run["samples"], run["samples"][1:]))
    full, half, loaded = [r["samples"][0] for r in runs]
    assert half["pack_voltage_v"] < full["pack_voltage_v"]
    assert loaded["pack_voltage_v"] < half["pack_voltage_v"]
    assert loaded["pack_current_a"] > half["pack_current_a"]
    assert loaded["acceleration_m_s2"] < half["acceleration_m_s2"]


def test_auxiliary_load_keeps_consuming_charge_after_brake_and_has_separate_energy(report, model):
    run = report["cases"][2]["report"]
    brake_samples = [s for s in run["samples"] if s["driver_modes"] == ["brake", "brake"]]
    assert len(brake_samples) > 2
    first, final = brake_samples[0], brake_samples[-1]
    battery = BatteryModel.from_component(next(c for c in model.components if c.id == "battery"))
    assert all(s["pack_current_a"] == 0.25 for s in brake_samples)
    assert final["soc"] == pytest.approx(
        first["soc"] - 0.25 * (final["time_s"] - first["time_s"]) / (battery.capacity_ah * 3600),
        abs=1e-12,
    )
    assert 0 < run["metrics"]["auxiliary_terminal_energy_j"] < run["metrics"]["terminal_energy_j"]
    assert all(
        s["auxiliary_power_w"] == pytest.approx(0.25 * s["pack_voltage_v"]) for s in run["samples"]
    )
    projected = project_rover_evidence(report)["cases"][2]["run"]
    assert projected["power_conditions"] == {
        "initial_soc": 0.5,
        "auxiliary_current_a": 0.25,
        "vcc_v": 3.3,
    }
    assert projected["samples"][-1]["auxiliary_current_a"] == 0.25


def test_saved_comparison_keeps_power_conditions_and_auxiliary_energy_evidence(report):
    comparison = compare_rover_suites(report, report)
    assert comparison["status"] == "passed"
    assert "auxiliary_terminal_energy_j" not in comparison["cases"][0]["metrics"]
    assert comparison["cases"][2]["metrics"]["auxiliary_terminal_energy_j"]["delta"] == 0
    assert project_rover_evidence(comparison)["cases"]


def test_power_run_remains_usable_for_explicit_synthetic_bench_comparison(report):
    run = report["cases"][2]["report"]
    samples = tuple(
        BenchSample(
            time_s=s["time_s"],
            speed_m_s=s["speed_m_s"],
            pack_voltage_v=s["pack_voltage_v"],
            pack_current_a=s["pack_current_a"],
        )
        for s in run["samples"][::100]
    )
    trace = BenchTrace(
        metadata=BenchMetadata(
            source_kind="synthetic",
            reference="generated test",
            instrument="none",
            conditions="exact saved values",
            speed_reference="wheel",
        ),
        samples=samples,
        csv_sha256="1" * 64,
        metadata_sha256="2" * 64,
    )
    result = compare_bench_trace(
        report,
        trace,
        time_offset_s=0,
        maximum_sample_age_s=0,
        case_name="half_charge_auxiliary_load",
    )
    assert result["source_kind"] == "synthetic"
    assert result["counts"]["paired_rows"] == len(samples)
    assert result["metrics"]["pack_current_a"]["rmse"] == 0
    assert project_rover_evidence(result)["bench"]


def test_encoder_speed_firmware_still_runs_from_delivered_feedback_under_power_load(model):
    config = load_rover_suite(REFERENCE.with_name("rover_feedback_suite.yaml")).model_dump(
        mode="json"
    )
    config["cases"] = config["cases"][:1]
    config["cases"][0]["scenario"].update(initial_soc=0.5, auxiliary_current_a=0.1, duration_s=0.3)
    config["cases"][0]["acceptance"] = {"expected_status": "completed_without_brake"}
    result = run_rover_suite(model, RoverSuiteConfig.model_validate(config))
    assert result["status"] == "passed"
    run = result["cases"][0]["report"]
    assert run["schema_version"] == 9
    control = run["speed_control"]
    assert any(s["observation"] is not None for s in control["samples"])
    assert len({s["driver_pwm_duty"][0] for s in run["samples"]}) > 1


def test_build_reference_preserves_power_conditions_with_version_three(
    tmp_path, model, power_suite
):
    archive = tmp_path / "power.zip"
    # Even selecting the legacy full-charge case retains the entire power-aware suite.
    write_build_reference(archive, model, power_suite, "full_charge")
    assert verify_build_reference(archive)["schema_version"] == 3


def test_different_initial_charge_is_a_different_comparison_condition(model, power_suite):
    source = power_suite.cases[1].model_dump(mode="json")
    source["scenario"]["duration_s"] = 0.01
    source["acceptance"] = {"expected_status": "completed_without_brake"}
    config = RoverSuiteConfig.model_validate({"name": "charge_comparison", "cases": [source]})
    baseline = run_rover_suite(model, config)
    changed = config.model_dump(mode="json")
    changed["cases"][0]["scenario"]["initial_soc"] = 0.4
    candidate = run_rover_suite(model, RoverSuiteConfig.model_validate(changed))
    with pytest.raises(ValueError, match="different case conditions"):
        compare_rover_suites(baseline, candidate)


def test_a_power_model_error_does_not_leak_state_into_the_next_case(model, power_suite):
    data = power_suite.model_dump(mode="json")
    data["cases"] = [data["cases"][2], data["cases"][0]]
    data["cases"][0]["scenario"]["auxiliary_current_a"] = 100
    result = run_rover_suite(model, RoverSuiteConfig.model_validate(data))
    assert result["cases"][0]["status"] == "error"
    assert result["cases"][0]["report"] is None
    assert result["cases"][1]["status"] == "passed"
    assert result["cases"][1]["report"]["samples"][0]["soc"] == 1


def test_grade_imu_and_explicit_logic_supply_can_run_with_partial_charge(model):
    config = load_rover_scenario(REFERENCE.with_name("rover_uphill.yaml"))
    config = RoverBrakeConfig.model_validate(
        {**config.model_dump(), "initial_soc": 0.5, "auxiliary_current_a": 0.1, "vcc_v": 5}
    )
    result = run_rover_brake(model, config)
    assert result["schema_version"] == 9
    assert result["status"] == "stopped_before_wall"
    assert result["imu"]["readings"]
    projected = project_rover_evidence(result)["cases"][0]["run"]
    assert projected["grade_rad"] == config.grade_rad
    assert projected["power_conditions"]["vcc_v"] == 5


def test_cli_saves_power_run_and_never_overwrites_the_output(tmp_path):
    config = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml"))
    source = tmp_path / "half-charge.json"
    source.write_text(
        json.dumps({**config.model_dump(), "initial_soc": 0.5, "auxiliary_current_a": 0.1})
    )
    output = tmp_path / "run.json"
    args = ["rover-brake", str(REFERENCE), str(source), "--output", str(output)]
    assert main(args) == 0
    payload = output.read_bytes()
    assert json.loads(payload)["schema_version"] == 9
    assert main(args) == 2
    assert output.read_bytes() == payload


@pytest.mark.parametrize(
    "change",
    [
        {"initial_soc": 0},
        {"initial_soc": -0.1},
        {"initial_soc": 1.1},
        {"initial_soc": True},
        {"initial_soc": float("nan")},
        {"auxiliary_current_a": -1},
        {"auxiliary_current_a": True},
        {"auxiliary_current_a": float("inf")},
        {"vcc_v": 2.6},
        {"vcc_v": 5.6},
        {"vcc_v": True},
    ],
)
def test_invalid_power_conditions_are_rejected_before_running(power_suite, change):
    config = power_suite.cases[0].scenario.model_dump()
    with pytest.raises(ValueError):
        RoverBrakeConfig.model_validate({**config, **change})


def test_explicit_legacy_defaults_keep_existing_encodings_and_comparisons(report):
    old = deepcopy(report)
    # Restrict this comparison to the untouched schema-7 case.
    old["config"]["cases"] = old["config"]["cases"][:1]
    old["cases"] = old["cases"][:1]
    old["summary"] = {"total": 1, "passed": 1, "failed": 0, "error": 0}
    explicit = deepcopy(old)
    for config in (
        explicit["config"]["cases"][0]["scenario"],
        explicit["cases"][0]["report"]["config"],
    ):
        config.update(initial_soc=1, auxiliary_current_a=0, vcc_v=3.3)
        restored = RoverBrakeConfig.model_validate(config).model_dump()
        assert all(name not in restored for name in ("initial_soc", "auxiliary_current_a", "vcc_v"))
    result = compare_rover_suites(old, explicit)
    assert result["design_changes"]["total"] == 0


def test_moving_rover_cannot_silently_use_an_unconnected_battery(model, power_suite):
    battery = next(c for c in model.components if c.id == "battery")
    spare = battery.model_copy(update={"id": "spare", "pins": ()})
    changed = model.model_copy(update={"components": (*model.components, spare)})
    config = power_suite.cases[1].scenario.model_copy(update={"battery": "spare"})
    with pytest.raises(ValueError, match="directly supply"):
        run_rover_brake(changed, config)
