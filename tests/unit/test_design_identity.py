import pytest

from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.design_identity import canonical_sha256, design_changes, project_identity
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


def test_design_fingerprint_is_independent_of_graph_and_parameter_list_order(model):
    data = model.model_dump()
    for key in ("components", "nets", "i2c_buses", "assignments"):
        data[key] = tuple(reversed(data[key]))
    data["assembly"]["links"] = tuple(reversed(data["assembly"]["links"]))
    for component in data["components"]:
        component["pins"] = tuple(reversed(component["pins"]))
        component["parameters"] = tuple(reversed(component["parameters"]))
        for pin in component["pins"]:
            pin["capabilities"] = tuple(reversed(pin["capabilities"]))
    for net in data["nets"]:
        net["endpoints"] = tuple(reversed(net["endpoints"]))
    for bus in data["i2c_buses"]:
        bus["devices"] = tuple(reversed(bus["devices"]))
    reordered = RobotProject.model_validate(data)
    assert project_identity(model) == project_identity(reordered)
    assert canonical_sha256(project_identity(model)) == canonical_sha256(
        project_identity(reordered)
    )
    assert design_changes(model, reordered)["total"] == 0


def test_changes_retain_parameter_value_unit_and_evidence_provenance(model):
    data = model.model_dump()
    motor = next(component for component in data["components"] if component["id"] == "left_motor")
    parameter = motor["parameters"][0]
    parameter.update(
        value=parameter["value"] + 1, evidence="measured", reference="bench measurement"
    )
    result = design_changes(model, RobotProject.model_validate(data))
    assert result["total"] == 3 and not result["truncated"]
    prefix = f"/components/left_motor/parameters/{parameter['name']}/"
    assert {change["path"] for change in result["changes"]} == {
        prefix + key for key in ("value", "evidence", "reference")
    }


def test_added_objects_and_truncated_change_lists_are_explicit(model):
    data = model.model_dump()
    data["components"] += ({"id": "extra_wheel", "type": "wheel", "category": "wheel"},)
    data["assembly"]["total_mass_kg"] += 1
    candidate = RobotProject.model_validate(data)
    result = design_changes(model, candidate, limit=1)
    assert result["total"] == 2 and result["shown"] == 1 and result["truncated"]
    full = design_changes(model, candidate)
    added = next(
        change for change in full["changes"] if change["path"] == "/components/extra_wheel"
    )
    assert added["baseline"] is None and not added["baseline_present"]
    assert added["candidate_present"] and added["candidate"]["category"] == "wheel"


def test_fingerprint_rejects_nonfinite_values():
    with pytest.raises(ValueError):
        canonical_sha256({"value": float("nan")})
