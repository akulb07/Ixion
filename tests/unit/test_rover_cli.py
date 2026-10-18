import json

import pytest

from roboforge.cli import main
from roboforge.hardware.rover_brake import RoverBrakeConfig, load_rover_scenario
from tests.unit.test_hardware_project import REFERENCE


@pytest.mark.parametrize(
    "scenario,status,code",
    [
        ("rover_wall.yaml", "stopped_before_wall", 0),
        ("rover_late_brake.yaml", "wall_contact", 3),
        ("rover_wall_imu.yaml", "stopped_before_wall", 0),
        ("rover_wall_debug.yaml", "stopped_before_wall", 0),
    ],
)
def test_cli_saves_complete_report_and_preserves_existing_file(
    tmp_path, capsys, scenario, status, code
):
    output = tmp_path / "rover.json"
    args = [
        "rover-brake",
        str(REFERENCE),
        str(REFERENCE.with_name(scenario)),
        "--output",
        str(output),
    ]
    assert main(args) == code
    assert status in capsys.readouterr().out
    payload = output.read_bytes()
    report = json.loads(payload)
    assert report["status"] == status
    assert report["experiment"] == "straight_line_wall_brake"
    assert report["project"]["name"] == "esp32_diff_drive_reference"
    assert report["limitations"] and report["power_diagnostics"]
    assert report["samples"] and report["control_decisions"] and report["readings"]
    assert main(args) == 2
    assert output.read_bytes() == payload


@pytest.mark.parametrize(
    "content",
    [
        "wall_distance_m: 1\nwall_distance_m: 2",
        "unknown: 1",
        "x" * 65537,
        "!!python/object/apply:os.system ['echo nope']",
    ],
    ids=["duplicate-key", "unknown-field", "oversized", "unsafe-tag"],
)
def test_invalid_scenario_does_not_create_report(tmp_path, content):
    source = tmp_path / "scenario.yaml"
    source.write_text(content)
    output = tmp_path / "report.json"
    assert main(["rover-brake", str(REFERENCE), str(source), "--output", str(output)]) == 2
    assert not output.exists()


def test_failed_physics_does_not_create_report_and_json_scenarios_work(tmp_path):
    config = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml"))
    source = tmp_path / "scenario.json"
    source.write_text(config.model_copy(update={"friction_coefficient": 0.001}).model_dump_json())
    output = tmp_path / "report.json"
    assert main(["rover-brake", str(REFERENCE), str(source), "--output", str(output)]) == 2
    assert not output.exists()


@pytest.mark.parametrize(
    "changes,message",
    [
        ({"duration_s": 20}, "step budget"),
        ({"sensor_offset_m": 0.2}, "front boundary"),
        ({"wall_distance_m": 0.1}, "front boundary"),
        ({"poll_every_steps": 10}, "poll period"),
        ({"right_motor": "left_motor"}, "distinct rover motors"),
        ({"duration_s": True}, "real number"),
        ({"brake_threshold_m": float("nan")}, "finite"),
    ],
    ids=[
        "step-budget",
        "sensor-offset",
        "initial-contact",
        "poll-period",
        "same-motor",
        "boolean",
        "nan",
    ],
)
def test_invalid_fixture_is_rejected(changes, message):
    data = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml")).model_dump()
    data.update(changes)
    with pytest.raises(ValueError, match=message):
        RoverBrakeConfig.model_validate(data)


def test_missing_scenario_is_a_validation_error(tmp_path):
    with pytest.raises(ValueError, match="Invalid rover scenario"):
        load_rover_scenario(tmp_path / "missing.yaml")


@pytest.mark.parametrize(
    "faults",
    [
        [{"channel": "left", "start_time_s": 0}] * 2,
        [{"channel": "left", "start_time_s": 5}],
        [{"channel": "right", "start_time_s": -1}],
        [{"channel": "left", "start_time_s": 0, "kind": "offset", "offset_counts": 0}],
    ],
)
def test_invalid_encoder_faults_are_rejected_before_running(faults):
    data = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml")).model_dump()
    with pytest.raises(ValueError):
        RoverBrakeConfig.model_validate({**data, "encoder_faults": faults})


def test_cli_loads_counter_fault_from_scenario_and_saves_both_count_layers(tmp_path):
    config = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml"))
    data = {
        **config.model_dump(),
        "duration_s": 0.1,
        "encoder_faults": [{"channel": "left", "start_time_s": 0.05}],
    }
    source, output = tmp_path / "fault.json", tmp_path / "result.json"
    source.write_text(json.dumps(data))
    assert main(["rover-brake", str(REFERENCE), str(source), "--output", str(output)]) == 3
    result = json.loads(output.read_text())
    assert result["schema_version"] == 7
    assert result["encoder_feedback"]["activations"][0]["time_s"] == 0.05
    final = result["samples"][-1]
    assert final["encoder_counts"]["left"] < final["decoded_encoder_counts"]["left"]
