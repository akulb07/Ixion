import json

import pytest

from roboforge.cli import main
from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel, BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.motor import DCMotor
from roboforge.hardware.powertrain import MotorChannel, Powertrain
from roboforge.hardware.rolling import RollingBody
from roboforge.hardware.rolling_runtime import RollingRuntime
from roboforge.hardware.rover_sizing import load_rover_sizing, size_rover
from roboforge.hardware.starter import create_starter
from tests.unit.test_hardware_project import REFERENCE


def inputs():
    return load_project(REFERENCE), load_rover_sizing(REFERENCE.with_name("rover_sizing.yaml"))


def test_reference_sizing_retains_design_provenance_and_replays_mechanical_acceleration():
    project, config = inputs()
    # The runtime currently has no auxiliary-load argument. Use zero for this
    # exact end-to-end replay against the same battery and motor models.
    config = config.model_copy(update={"auxiliary_current_a": 0})
    report = size_rover(project, config)
    assert report["status"] == "checks_passed"
    assert report["engineering_readiness"] == "not_assessed"
    assert report["project"]["snapshot"] == project.model_dump(mode="json")
    assert len(report["project"]["sha256"]) == 64
    assert report["connections"][0]["driver"] == "driver"
    assert report["wheels"] == ["left_wheel", "right_wheel"]
    assert report["project"]["snapshot"]["components"][0]["pins"]
    models, result = report["models"], report["result"]
    body = RollingBody.model_validate(models["body"])
    motor = DCMotor.model_validate(models["motor"])
    system = Powertrain(
        battery=BatteryModel.model_validate(models["battery"]),
        channels=(
            MotorChannel(
                motor=motor,
                inputs=DriverInputs.model_validate(result["command"]),
                output_speed_rad_s=config.speed_m_s / body.wheel_radius_m,
            ),
        )
        * 2,
    )
    runtime = RollingRuntime(system, BatteryState(soc=config.soc), body)
    assert runtime.sample.acceleration_m_s2 == pytest.approx(config.acceleration_m_s2)
    assert runtime.sample.shaft.point.model_dump(mode="json") == result["point"]


def test_more_payload_and_lower_charge_change_the_required_duty():
    project, config = inputs()
    baseline = size_rover(project, config)["result"]
    heavier = project.model_copy(
        update={"assembly": project.assembly.model_copy(update={"total_mass_kg": 4.6})}
    )
    payload = size_rover(heavier, config)["result"]
    empty = size_rover(project, config.model_copy(update={"soc": 0.1}))["result"]
    assert (
        payload["required_motor_current_per_channel_a"]
        > baseline["required_motor_current_per_channel_a"]
    )
    assert payload["required_pwm_duty"] > baseline["required_pwm_duty"]
    assert empty["required_pwm_duty"] > baseline["required_pwm_duty"]
    assert (
        empty["required_motor_current_per_channel_a"]
        == baseline["required_motor_current_per_channel_a"]
    )


def test_cli_saves_success_and_failure_evidence_without_overwriting(tmp_path, capsys):
    source = REFERENCE.with_name("rover_sizing.yaml")
    out = tmp_path / "sizing.json"
    args = ["rover-size", str(REFERENCE), str(source), "--output", str(out)]
    assert main(args) == 0
    assert "readiness not assessed" in capsys.readouterr().out
    payload = out.read_bytes()
    assert json.loads(payload)["artifact_type"] == "ixion_rover_sizing"
    assert main(args) == 2
    assert out.read_bytes() == payload
    _, config = inputs()
    bad = tmp_path / "too-fast.json"
    bad.write_text(config.model_copy(update={"speed_m_s": 2}).model_dump_json())
    failed = tmp_path / "failed.json"
    assert main(["rover-size", str(REFERENCE), str(bad), "--output", str(failed)]) == 3
    report = json.loads(failed.read_text())
    assert report["status"] == "checks_failed"
    assert report["result"]["required_pwm_duty"] > 1


@pytest.mark.parametrize(
    "content",
    [
        "speed_m_s: 1\nspeed_m_s: 2",
        "unknown: 1",
        "x" * 65537,
        "!!python/object/apply:os.system ['echo nope']",
        "[1, 2]",
    ],
    ids=["duplicate-key", "unknown-field", "oversized", "unsafe-tag", "nonmapping"],
)
def test_invalid_sizing_input_does_not_create_a_report(tmp_path, content):
    source, output = tmp_path / "bad.yaml", tmp_path / "result.json"
    source.write_text(content)
    assert main(["rover-size", str(REFERENCE), str(source), "--output", str(output)]) == 2
    assert not output.exists()


def test_selected_battery_must_be_the_pack_wired_to_the_driver():
    project, config = inputs()
    battery = next(c for c in project.components if c.id == "battery")
    # Unconnected optional pins let the topology stay valid; a second numerical
    # battery model must still not silently replace the wired supply.
    spare = battery.model_copy(update={"id": "spare", "pins": ()})
    project = project.model_copy(update={"components": (*project.components, spare)})
    with pytest.raises(ValueError, match="directly supply"):
        size_rover(project, config.model_copy(update={"battery": "spare"}))


def test_mismatched_motor_models_are_not_silently_averaged():
    project, config = inputs()
    parts = list(project.components)
    index = next(i for i, c in enumerate(parts) if c.id == "right_motor")
    motor = parts[index]
    parts[index] = motor.model_copy(
        update={
            "parameters": tuple(
                p.model_copy(update={"value": 200}) if p.name == "output_no_load_speed" else p
                for p in motor.parameters
            )
        }
    )
    with pytest.raises(ValueError, match="two identical motor models"):
        size_rover(project.model_copy(update={"components": tuple(parts)}), config)


def test_missing_file_is_a_bounded_load_error(tmp_path):
    with pytest.raises(ValueError, match="Invalid rover sizing input"):
        load_rover_sizing(tmp_path / "missing.yaml")


def test_packaged_starter_can_run_its_sizing_input(tmp_path):
    root = tmp_path / "starter"
    create_starter(root)
    output = root / "sizing.json"
    assert (
        main(
            [
                "rover-size",
                str(root / "robot.yaml"),
                str(root / "rover_sizing.yaml"),
                "--output",
                str(output),
            ]
        )
        == 0
    )
    report = json.loads(output.read_text())
    assert report["result"]["required_pwm_duty"] == pytest.approx(0.4495015868795172)
