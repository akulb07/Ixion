import csv
import hashlib
import io
import json
import stat
import zipfile

import pytest

from roboforge.cli import main
from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.build_reference import (
    BASE_FILES,
    CASE_FILES,
    build_reference_files,
    verify_build_reference,
    write_build_reference,
)
from roboforge.hardware.rover_suite import load_rover_suite
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


def rows(data):
    return list(csv.DictReader(io.StringIO(data.decode("utf-8"))))


def test_export_preserves_design_parameter_evidence_and_pin_connections(model):
    files = build_reference_files(model)
    assert set(files) == BASE_FILES | {"manifest.json"}
    assert RobotProject.model_validate_json(files["project.json"]) == model
    assert len(rows(files["parts.csv"])) == len(model.components) == 15
    assert len(rows(files["parameters.csv"])) == sum(len(c.parameters) for c in model.components)
    pins = rows(files["pins.csv"])
    for assignment in model.assignments:
        row = next(
            p
            for p in pins
            if p["mcu"] == assignment.endpoint.component and p["pin"] == assignment.endpoint.pin
        )
        assert row["assignment"] == assignment.function
        assert row["net"]
    manifest = json.loads(files["manifest.json"])
    assert manifest["engineering_readiness_assessed"] is False
    assert len(manifest["incomplete_component_metadata"]) == 15
    for name, record in manifest["files"].items():
        assert record == {
            "bytes": len(files[name]),
            "sha256": hashlib.sha256(files[name]).hexdigest(),
        }
    assert all(
        row["measured_value"] == "" and row["uncertainty"] == ""
        for row in rows(files["bench-checklist.csv"])
    )


def test_csv_formula_like_text_is_escaped_without_changing_the_source_design(model):
    data = model.model_dump()
    data["components"][0].update(manufacturer='=HYPERLINK("bad")', part_number="  +formula")
    parameter_component = next(c for c in data["components"] if c["parameters"])
    parameter_component["parameters"][0]["reference"] = "\t@formula"
    changed = RobotProject.model_validate(data)
    files = build_reference_files(changed)
    part = next(
        row for row in rows(files["parts.csv"]) if row["component"] == data["components"][0]["id"]
    )
    assert part["manufacturer"].startswith("'")
    original = json.loads(files["project.json"])
    assert original["components"][0]["manufacturer"] == '=HYPERLINK("bad")'
    assert any(row["reference"] == "'\t@formula" for row in rows(files["parameters.csv"]))


def test_selected_suite_case_exports_host_configuration_without_running_it(model):
    suite = load_rover_suite(REFERENCE.with_name("rover_feedback_suite.yaml"))
    files = build_reference_files(model, suite, "normal")
    assert set(files) == BASE_FILES | CASE_FILES | {"manifest.json"}
    firmware = json.loads(files["firmware-config.json"])
    assert firmware["execution"] == "host_python"
    assert firmware["parameters"]["encoder"]["counts_per_wheel_revolution"] == [4800, 4800]
    assert json.loads(files["case.json"])["name"] == "normal"
    assert "samples" not in firmware
    with pytest.raises(ValueError, match="select one named case"):
        build_reference_files(model, suite)
    with pytest.raises(ValueError, match="not present"):
        build_reference_files(model, suite, "missing")
    with pytest.raises(ValueError, match="requires a suite"):
        build_reference_files(model, case_name="normal")


def test_archive_bytes_are_deterministic_and_existing_files_are_preserved(tmp_path, model):
    first, second = tmp_path / "first.zip", tmp_path / "second.zip"
    manifest = write_build_reference(first, model)
    write_build_reference(second, model)
    assert first.read_bytes() == second.read_bytes()
    assert verify_build_reference(first) == manifest
    before = first.read_bytes()
    with pytest.raises(OSError):
        write_build_reference(first, model)
    assert first.read_bytes() == before


@pytest.mark.parametrize(
    "change",
    [
        "contents",
        "rehash",
        "missing",
        "extra",
        "duplicate",
        "symlink",
        "unsafe_name",
        "compression",
    ],
)
def test_verifier_rejects_inconsistent_or_unsupported_archives(tmp_path, model, change):
    files = build_reference_files(model)
    if change in ("contents", "rehash"):
        files["parts.csv"] += b"changed\n"
        if change == "rehash":
            manifest = json.loads(files["manifest.json"])
            manifest["files"]["parts.csv"] = {
                "bytes": len(files["parts.csv"]),
                "sha256": hashlib.sha256(files["parts.csv"]).hexdigest(),
            }
            files["manifest.json"] = json.dumps(manifest).encode()
    elif change == "missing":
        del files["pins.csv"]
    elif change in ("extra", "unsafe_name"):
        files["extra.txt" if change == "extra" else "../escape.txt"] = b"bad"
    archive = tmp_path / "bad.zip"
    with zipfile.ZipFile(archive, "w") as output:
        for name, content in files.items():
            info = zipfile.ZipInfo(name)
            if change == "compression":
                info.compress_type = zipfile.ZIP_BZIP2
            if change == "symlink" and name == "project.json":
                info.external_attr = (stat.S_IFLNK | 0o777) << 16
            output.writestr(info, content)
        if change == "duplicate":
            with pytest.warns(UserWarning, match="Duplicate name"):
                output.writestr("project.json", files["project.json"])
    with pytest.raises(ValueError, match="Invalid build reference"):
        verify_build_reference(archive)


def test_cli_exports_errors_as_evidence_and_preserves_existing_output(tmp_path, model):
    data = model.model_dump(mode="json")
    data["nets"] = [
        net
        for net in data["nets"]
        if not any(e["component"] == "driver" and e["pin"] == "STBY" for e in net["endpoints"])
    ]
    source, output = tmp_path / "project.json", tmp_path / "reference.zip"
    source.write_text(json.dumps(data))
    args = ["build-reference", str(source), "--output", str(output)]
    assert main(args) == 3
    manifest = verify_build_reference(output)
    assert manifest["design_errors"] > 0
    assert main(["verify-build-reference", str(output)]) == 0
    before = output.read_bytes()
    assert main(args) == 2 and output.read_bytes() == before


def test_file_limits_and_missing_archives_are_validation_errors(tmp_path, monkeypatch, model):
    from roboforge.hardware import build_reference as module

    monkeypatch.setattr(module, "MAX_ENTRY_BYTES", 10)
    with pytest.raises(ValueError, match="size limits"):
        build_reference_files(model)
    assert main(["verify-build-reference", str(tmp_path / "missing.zip")]) == 2


def test_build_reference_decompression_uses_bounded_member_reads(tmp_path, monkeypatch, model):
    source = tmp_path / "reference.zip"
    write_build_reference(source, model)
    original_read = zipfile.ZipExtFile.read
    reads = []

    def bounded_read(member, size=-1):
        assert 0 <= size <= 2 * 1024 * 1024 + 1
        reads.append(member.name)
        return original_read(member, size)

    monkeypatch.setattr(zipfile.ZipExtFile, "read", bounded_read)
    verify_build_reference(source)
    assert len(reads) == len(BASE_FILES) + 1


def test_level_exports_keep_legacy_encoding_and_grade_exports_declare_version_two(tmp_path, model):
    level = load_rover_suite(REFERENCE.with_name("rover_feedback_suite.yaml"))
    files = build_reference_files(model, level, "normal")
    assert json.loads(files["manifest.json"])["schema_version"] == 1
    assert "grade_rad" not in json.loads(files["case.json"])["scenario"]
    graded = load_rover_suite(REFERENCE.with_name("rover_grade_suite.yaml"))
    files = build_reference_files(model, graded, "uphill_5_degrees")
    assert json.loads(files["manifest.json"])["schema_version"] == 2
    assert json.loads(files["case.json"])["scenario"]["grade_rad"] > 0
    archive = tmp_path / "grade.zip"
    write_build_reference(archive, model, graded, "uphill_5_degrees")
    assert verify_build_reference(archive)["schema_version"] == 2
