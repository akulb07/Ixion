import json

import pytest
from pydantic import ValidationError

from roboforge.cli import main
from roboforge.hardware import RobotProject, check_project
from roboforge.hardware.project import LogicLevels, VoltageRange


def voltage(low, high):
    return dict(minimum_v=low, maximum_v=high, reference="Synthetic test envelope")


def levels(low, high):
    return dict(low_max_v=low, high_min_v=high, reference="Synthetic test guarantees")


def project_data(digital=False):
    raw = dict(
        name="electrical_test",
        assembly=dict(total_mass_kg=1, wheel_radius_m=0.03, track_width_m=0.2),
        components=[
            dict(
                id="source",
                type="test",
                category="mcu",
                pins=[
                    dict(
                        id="OUT",
                        kind="bidirectional" if digital else "power_out",
                        capabilities=["DIGITAL_OUT", "PWM"] if digital else ["POWER"],
                        driven_voltage=voltage(0, 3.3) if digital else voltage(6, 8.4),
                        output_levels=levels(0.4, 2.8) if digital else None,
                    )
                ],
            ),
            dict(
                id="sink",
                type="test",
                category="motor_driver",
                pins=[
                    dict(
                        id="IN",
                        kind="input" if digital else "power_in",
                        capabilities=["DIGITAL_IN"] if digital else ["POWER"],
                        accepted_voltage=voltage(0, 3.6) if digital else voltage(5, 9),
                        input_levels=levels(0.8, 2.0) if digital else None,
                    )
                ],
            ),
        ],
        nets=[
            dict(
                id="wire",
                endpoints=[dict(component="source", pin="OUT"), dict(component="sink", pin="IN")],
            )
        ],
        assignments=[dict(endpoint=dict(component="source", pin="OUT"), function="PWM")]
        if digital
        else [],
    )
    for component in raw["components"]:
        component["pins"][0]["ground_reference"] = "GND"
        component["pins"].append(dict(id="GND", kind="ground", capabilities=["GROUND"]))
    raw["nets"].append(
        dict(id="ground", endpoints=[dict(component=c["id"], pin="GND") for c in raw["components"]])
    )
    return raw


def codes(raw):
    return {d.code for d in check_project(RobotProject.model_validate(raw))}


@pytest.mark.parametrize("digital", [False, True])
def test_declared_compatible_envelopes_do_not_imply_readiness(digital):
    assert codes(project_data(digital)) == {"engineering_checks_pending"}


@pytest.mark.parametrize("low,high", [(4.9, 8.4), (6, 9.1)])
def test_supply_extremes_not_just_nominal_are_checked(low, high):
    raw = project_data()
    raw["components"][0]["pins"][0]["driven_voltage"] = voltage(low, high)
    assert "voltage_range_mismatch" in codes(raw)


def test_equal_envelope_boundaries_are_accepted():
    raw = project_data()
    raw["components"][0]["pins"][0]["driven_voltage"] = voltage(5, 9)
    assert codes(raw) == {"engineering_checks_pending"}


@pytest.mark.parametrize("low,high", [(0.9, 2.8), (0.4, 1.9)])
def test_logic_high_and_low_each_need_guarantees(low, high):
    raw = project_data(True)
    raw["components"][0]["pins"][0]["output_levels"] = levels(low, high)
    assert "logic_level_mismatch" in codes(raw)


def test_missing_data_is_unknown_not_zero_or_pass():
    raw = project_data(True)
    raw["components"][0]["pins"][0]["output_levels"] = None
    raw["components"][1]["pins"][0]["accepted_voltage"] = None
    assert {"voltage_limits_unknown", "logic_levels_unknown"} <= codes(raw)


def test_pwm_on_input_only_pin_is_rejected():
    raw = project_data(True)
    pin = raw["components"][0]["pins"][0]
    pin.update(kind="input", capabilities=["DIGITAL_IN", "INTERRUPT"])
    assert "pin_function_unsupported" in codes(raw)


def test_assigned_output_conflicts_with_explicit_output():
    raw = project_data(True)
    raw["components"][1]["pins"][0].update(kind="output", capabilities=["DIGITAL_OUT"])
    assert "net_source_conflict" in codes(raw)


def test_floating_power_input_is_not_powered_by_another_input():
    raw = project_data()
    raw["components"][0]["pins"][0]["kind"] = "power_in"
    assert "power_source_missing" in codes(raw)


def test_assignments_require_unique_existing_mcu_pins():
    raw = project_data(True)
    raw["assignments"].append(raw["assignments"][0])
    with pytest.raises(ValidationError, match="duplicate MCU pin assignment"):
        RobotProject.model_validate(raw)
    raw["assignments"] = [dict(endpoint=dict(component="sink", pin="IN"), function="DIGITAL_IN")]
    with pytest.raises(ValidationError, match="existing MCU pin"):
        RobotProject.model_validate(raw)


@pytest.mark.parametrize("low,high", [(3, 2), (-1, 2), (float("nan"), 3), (True, 3)])
def test_invalid_voltage_data_rejected(low, high):
    with pytest.raises(ValidationError):
        VoltageRange.model_validate(voltage(low, high))


def test_overlapping_logic_thresholds_rejected():
    with pytest.raises(ValidationError):
        LogicLevels.model_validate(levels(2, 2))


def test_guarantees_cannot_exceed_their_own_voltage_envelope():
    raw = project_data(True)
    raw["components"][0]["pins"][0]["output_levels"] = levels(0.4, 5)
    with pytest.raises(ValidationError, match="inside their declared voltage envelope"):
        RobotProject.model_validate(raw)


def test_cli_reports_voltage_failure_and_retains_unknown_readiness(tmp_path, capsys):
    raw = project_data()
    raw["components"][0]["pins"][0]["driven_voltage"] = voltage(6, 10)
    path = tmp_path / "voltage.json"
    path.write_text(json.dumps(raw), encoding="utf-8")
    assert main(["inspect-project", str(path)]) == 3
    report = json.loads(capsys.readouterr().out)
    assert report["engineering_readiness"] == "not_assessed"
    problem = next(d for d in report["diagnostics"] if d["code"] == "voltage_range_mismatch")
    assert problem["context"] == ["wire", "source.OUT", "sink.IN"]


def test_unassigned_signal_direction_stays_unknown():
    raw = project_data(True)
    raw["assignments"] = []
    assert "signal_drive_unresolved" in codes(raw)


def test_disconnected_ground_reference_is_an_error():
    raw = project_data()
    raw["nets"].pop()
    assert "ground_reference_unconnected" in codes(raw)


def test_unknown_ground_is_not_inferred_from_component_ground_pin():
    raw = project_data()
    del raw["components"][0]["pins"][0]["ground_reference"]
    assert "ground_reference_unknown" in codes(raw)


def test_separate_ground_domains_are_not_implicitly_shorted():
    raw = project_data(True)
    raw["nets"].pop()
    for component in raw["components"]:
        component["pins"].append(dict(id="RETURN", kind="ground", capabilities=["GROUND"]))
        raw["nets"].append(
            dict(
                id=component["id"] + "_ground",
                endpoints=[dict(component=component["id"], pin=p) for p in ("GND", "RETURN")],
            )
        )
    assert "ground_reference_mismatch" in codes(raw)


@pytest.mark.parametrize("reference", ["MISSING", "OUT"])
def test_ground_reference_must_name_a_real_ground_pin(reference):
    raw = project_data()
    raw["components"][0]["pins"][0]["ground_reference"] = reference
    with pytest.raises(ValidationError, match="local ground pin"):
        RobotProject.model_validate(raw)
