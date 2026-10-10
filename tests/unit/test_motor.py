import json
import math
from pathlib import Path

import pytest
from pydantic import ValidationError

from roboforge.cli import main
from roboforge.hardware import load_project
from roboforge.hardware.motor import DCMotor, MotorDatasheet

REFERENCE = Path(__file__).parents[2] / "examples/esp32_diff_drive/robot.yaml"


def datasheet():
    return MotorDatasheet(
        nominal_voltage_v=6,
        output_no_load_rpm=300,
        no_load_current_a=0.1,
        stall_current_a=1.8,
        output_stall_torque_nm=0.25,
        gear_ratio=100,
    )


def test_zero_voltage_at_rest():
    point = datasheet().derive().operating_point(0, 0)
    assert point.current_a == point.output_torque_nm == point.electrical_power_w == 0


def test_stall_recovers_datasheet_values():
    point = datasheet().derive().operating_point(6, 0)
    assert point.current_a == pytest.approx(1.8)
    assert point.output_torque_nm == pytest.approx(0.25)
    assert point.output_power_w == 0
    assert point.copper_loss_w == pytest.approx(10.8)


def test_no_load_recovers_speed_current_and_zero_available_torque():
    point = datasheet().derive().operating_point(6, 300 * math.tau / 60)
    assert point.current_a == pytest.approx(0.1)
    assert point.output_torque_nm == pytest.approx(0, abs=1e-12)
    assert point.rotor_rpm == pytest.approx(30_000)


def test_half_speed_and_load_torque():
    point = datasheet().derive().operating_point(6, 150 * math.tau / 60, 0.1)
    assert point.current_a == pytest.approx(0.95)
    assert point.output_torque_nm == pytest.approx(0.125)
    assert point.net_output_torque_nm == pytest.approx(0.025)


def test_reverse_is_symmetric():
    model = datasheet().derive()
    forward = model.operating_point(6, 10, 0.1)
    reverse = model.operating_point(-6, -10, -0.1)
    assert reverse.current_a == pytest.approx(-forward.current_a)
    assert reverse.output_torque_nm == pytest.approx(-forward.output_torque_nm)
    assert reverse.net_output_torque_nm == pytest.approx(-forward.net_output_torque_nm)
    assert reverse.electrical_power_w == pytest.approx(forward.electrical_power_w)


@pytest.mark.parametrize("voltage,speed", [(6, 0), (6, 10), (6, 40), (0, 10), (-6, 10), (-6, -40)])
def test_signed_power_balance_in_driving_braking_and_generation(voltage, speed):
    point = datasheet().derive().operating_point(voltage, speed)
    assert point.electrical_power_w == pytest.approx(
        point.output_power_w + point.copper_loss_w + point.friction_loss_w + point.gearbox_loss_w,
        abs=1e-12,
    )


def test_zero_voltage_spinning_is_braking_not_coasting():
    point = datasheet().derive().operating_point(0, 10)
    assert point.current_a < 0 and point.output_torque_nm < 0
    assert point.electrical_power_w == 0 and point.copper_loss_w > 0


def test_gearing_scales_speed_and_torque_without_creating_energy():
    direct = DCMotor(winding_resistance_ohm=2, motor_constant_si=0.1)
    geared = DCMotor(
        winding_resistance_ohm=2, motor_constant_si=0.1, gear_ratio=10, gear_efficiency=0.8
    )
    a, b = direct.operating_point(6, 20), geared.operating_point(6, 2)
    assert a.current_a == b.current_a
    assert b.output_torque_nm == pytest.approx(a.output_torque_nm * 8)
    assert b.output_power_w == pytest.approx(a.output_power_w * 0.8)


def test_inconsistent_datasheet_is_rejected():
    bad = datasheet().model_copy(update={"output_stall_torque_nm": 1})
    with pytest.raises(ValueError, match="efficiency above one"):
        bad.derive()
    with pytest.raises(ValidationError, match="no-load current"):
        MotorDatasheet.model_validate({**datasheet().model_dump(), "no_load_current_a": 2})


@pytest.mark.parametrize("value", [True, "6", float("inf"), float("nan")])
def test_nonfinite_or_implicit_inputs_rejected(value):
    with pytest.raises(ValueError):
        datasheet().derive().operating_point(value, 0)


def test_component_units_are_enforced():
    component = next(c for c in load_project(REFERENCE).components if c.id == "left_motor")
    assert MotorDatasheet.from_component(component) == datasheet()
    raw = component.model_dump(mode="json")
    raw["parameters"][0]["unit"] = "A"
    with pytest.raises(ValueError, match="unit V"):
        MotorDatasheet.from_component(type(component).model_validate(raw))


def test_cli_preserves_assumptions_and_reports_estimate(capsys):
    args = [
        "motor-point",
        str(REFERENCE),
        "--component",
        "left_motor",
        "--voltage",
        "6",
        "--rpm",
        "0",
    ]
    assert main(args) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["result_type"] == "estimate"
    assert report["point"]["output_torque_nm"] == pytest.approx(0.25)
    assert all(p["evidence"] == "assumed" for p in report["inputs"])
    assert main([*args[:-1], "nan"]) == 2
    assert not capsys.readouterr().out
