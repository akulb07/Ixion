import hashlib
import json
from importlib.resources import files

import pytest

from roboforge.cli import main
from roboforge.hardware import load_project, starter
from roboforge.hardware.starter import TEMPLATE_FILES, create_starter
from tests.unit.test_hardware_project import REFERENCE


def test_starter_copies_assumptions_and_records_exact_created_bytes(tmp_path):
    root = tmp_path / "my rover"
    manifest = create_starter(root)
    assert manifest == json.loads((root / "starter-manifest.json").read_text())
    assert set(manifest["files"]) == set(TEMPLATE_FILES)
    assert manifest["component_instances"] == 15
    assert manifest["engineering_readiness"] == "not_assessed"
    assert manifest["demo"] is None and manifest["diagnostics"]["warning"] > 0
    for name, record in manifest["files"].items():
        data = (root / name).read_bytes()
        assert record == {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
        assert b"\r\n" not in data
    assert load_project(root / "robot.yaml") == load_project(REFERENCE)
    assert not (root / "results").exists()


def test_packaged_reference_inputs_match_the_maintained_examples():
    resources = files("roboforge").joinpath("hardware/templates/esp32_rover")
    for name in ("robot.yaml", "rover_wall.yaml", "rover_feedback_suite.yaml"):
        assert resources.joinpath(name).read_text(encoding="utf-8") == REFERENCE.with_name(
            name
        ).read_text(encoding="utf-8")


def test_existing_directory_and_files_are_never_overwritten(tmp_path):
    root = tmp_path / "existing"
    root.mkdir()
    marker = root / "robot.yaml"
    marker.write_text("my work")
    with pytest.raises(ValueError, match="already exists"):
        create_starter(root, run_demo=True)
    assert list(root.iterdir()) == [marker] and marker.read_text() == "my work"
    with pytest.raises(ValueError, match="already exists"):
        create_starter(marker)


def test_unsupported_template_or_failed_demo_does_not_create_a_partial_project(
    tmp_path, monkeypatch
):
    destination = tmp_path / "unwritten"
    with pytest.raises(ValueError, match="unsupported"):
        create_starter(destination, template="unknown")

    def failed_run(*args):
        raise ValueError("fixture initialization failed")

    monkeypatch.setattr(starter, "run_rover_suite", failed_run)
    with pytest.raises(ValueError, match="initialization"):
        create_starter(destination, run_demo=True)
    assert not destination.exists()


def test_cli_creates_a_new_project_and_leaves_existing_work_alone(tmp_path, capsys):
    root = tmp_path / "created"
    assert main(["new-project", str(root)]) == 0
    assert "engineering readiness is unassessed" in capsys.readouterr().out
    saved = (root / "robot.yaml").read_bytes()
    assert main(["new-project", str(root)]) == 2
    assert (root / "robot.yaml").read_bytes() == saved
