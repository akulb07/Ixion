import copy
import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from roboforge.cli import main
from roboforge.hardware import RobotProject, check_project, load_project

REFERENCE = Path(__file__).parents[2] / "examples/esp32_diff_drive/robot.yaml"


def data():
    return load_project(REFERENCE).model_dump(mode="json")


def test_reference_roundtrip_and_honest_readiness(capsys):
    project = load_project(REFERENCE)
    assert len(project.components) == 15
    assert len(project.nets) == 24
    assert RobotProject.model_validate_json(project.model_dump_json()) == project
    diagnostics = check_project(project)
    assert all(d.severity == "WARNING" for d in diagnostics)
    assert "engineering_checks_pending" in {d.code for d in diagnostics}
    assert main(["inspect-project", str(REFERENCE)]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["engineering_readiness"] == "not_assessed"


@pytest.mark.parametrize(
    "issue",
    [
        "component",
        "pin",
        "net",
        "missing_pin",
        "reused_pin",
        "mechanical",
        "bus_nets",
        "bus_controller",
        "unit",
        "nan",
        "bool",
        "extra",
    ],
)
def test_schema_rejects_ambiguous_or_invalid_graphs(issue):
    raw = data()
    if issue == "component":
        raw["components"].append(copy.deepcopy(raw["components"][0]))
    elif issue == "pin":
        raw["components"][0]["pins"].append(copy.deepcopy(raw["components"][0]["pins"][0]))
    elif issue == "net":
        raw["nets"][1]["id"] = raw["nets"][0]["id"]
    elif issue == "missing_pin":
        raw["nets"][0]["endpoints"][0]["pin"] = "TYPO"
    elif issue == "reused_pin":
        raw["nets"][1]["endpoints"].append(raw["nets"][0]["endpoints"][0])
    elif issue == "mechanical":
        raw["assembly"]["links"][0]["target"] = "missing"
    elif issue == "bus_nets":
        raw["i2c_buses"][0]["scl_net"] = "i2c_sda"
    elif issue == "bus_controller":
        raw["i2c_buses"][0]["controller"] = "battery"
    elif issue == "unit":
        raw["components"][2]["parameters"][0]["unit"] = "bananas"
    elif issue == "nan":
        raw["assembly"]["total_mass_kg"] = float("nan")
    elif issue == "bool":
        raw["assembly"]["total_mass_kg"] = True
    else:
        raw["assembly"]["mass"] = 2
    with pytest.raises(ValidationError):
        RobotProject.model_validate(raw)


def test_missing_required_connection_is_a_diagnostic(tmp_path, capsys):
    raw = data()
    raw["nets"] = [n for n in raw["nets"] if n["id"] != "signal_STBY"]
    project = RobotProject.model_validate(raw)
    assert any(
        d.component == "driver" and d.code == "required_pin_unconnected"
        for d in check_project(project)
    )
    path = tmp_path / "broken.json"
    path.write_text(project.model_dump_json())
    assert main(["inspect-project", str(path)]) == 3
    assert "required_pin_unconnected" in capsys.readouterr().out


def test_output_to_ground_is_detected():
    raw = data()
    echo = next(n for n in raw["nets"] if n["id"] == "echo_high")
    raw["nets"][0]["endpoints"].extend(echo["endpoints"])
    raw["nets"].remove(echo)
    assert "net_source_conflict" in {
        d.code for d in check_project(RobotProject.model_validate(raw))
    }


def test_i2c_duplicate_address_and_incompatible_assignment():
    raw = data()
    extra = copy.deepcopy(next(c for c in raw["components"] if c["id"] == "imu"))
    extra["id"] = "imu2"
    raw["components"].append(extra)
    raw["i2c_buses"][0]["devices"].append({"component": "imu2", "address": 104})
    codes = {d.code for d in check_project(RobotProject.model_validate(raw))}
    assert {"i2c_duplicate_address", "i2c_line_membership", "required_pin_unconnected"} <= codes


def test_duplicate_yaml_and_input_budget(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("name: a\nname: b\n")
    with pytest.raises(ValueError, match="duplicate configuration key"):
        load_project(path)
    path.write_bytes(b" " * (1024 * 1024 + 1))
    with pytest.raises(ValueError, match="exceeds 1 MiB"):
        load_project(path)
