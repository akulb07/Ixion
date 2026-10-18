import json
from xml.etree import ElementTree as ET

import pytest

from roboforge.ci_reports import rover_suite_junit
from roboforge.cli import main
from tests.unit.test_hardware_project import REFERENCE
from tests.unit.test_rover_suite import suite_data


def test_rover_junit_separates_check_failures_from_model_errors_and_escapes_text():
    checks = [{"name": "clearance < limit", "actual": 0.1, "expected": 0.2, "passed": False}]
    report = {
        "schema_version": 1,
        "experiment": "rover_acceptance_suite",
        "name": "suite <a> & b\x00",
        "cases": [
            {"name": "normal", "status": "passed", "checks": [{"name": "status", "passed": True}]},
            {"name": "fault", "status": "failed", "checks": checks},
            {
                "name": "model_error\x01",
                "status": "error",
                "checks": [],
                "error": "traction < demand & load",
            },
        ],
    }
    root = ET.fromstring(rover_suite_junit(report))
    assert root.attrib == {
        "name": "suite <a> & b\ufffd",
        "tests": "3",
        "failures": "1",
        "errors": "1",
        "skipped": "0",
    }
    cases = root.findall("testcase")
    assert cases[0].find("failure") is None and cases[0].find("error") is None
    assert json.loads(cases[1].find("failure").text) == checks
    assert cases[2].find("error").attrib["type"] == "RoverModelError"
    assert "time" not in cases[0].attrib


@pytest.mark.parametrize(
    "case",
    [
        {"status": "passed", "checks": []},
        {"status": "passed", "checks": [{"passed": "yes"}]},
        {"status": "failed", "checks": [{"passed": True}]},
        {"status": "error", "checks": [], "error": ""},
        {"status": "unknown", "checks": []},
    ],
)
def test_junit_cannot_turn_missing_or_contradictory_evidence_into_a_pass(case):
    report = {
        "schema_version": 1,
        "experiment": "rover_acceptance_suite",
        "name": "bad",
        "cases": [{"name": "case", **case}],
    }
    with pytest.raises(ValueError):
        rover_suite_junit(report)


def test_cli_saves_json_and_junit_for_a_failed_suite(tmp_path):
    source = tmp_path / "suite.json"
    source.write_text(json.dumps(suite_data()))
    output, junit = tmp_path / "result.json", tmp_path / "result.xml"
    args = [
        "rover-suite",
        str(REFERENCE),
        str(source),
        "--output",
        str(output),
        "--junit",
        str(junit),
    ]
    assert main(args) == 3
    assert json.loads(output.read_text())["summary"]["failed"] == 1
    root = ET.parse(junit).getroot()
    assert root.attrib["failures"] == "1" and root.attrib["errors"] == "0"
    saved = junit.read_bytes()
    args[args.index("--output") + 1] = str(tmp_path / "fresh.json")
    assert main(args) == 2 and not (tmp_path / "fresh.json").exists()
    assert junit.read_bytes() == saved


def test_cli_rejects_colliding_outputs_before_running(tmp_path):
    source = tmp_path / "suite.json"
    source.write_text(json.dumps(suite_data()))
    output = tmp_path / "same.json"
    args = [
        "rover-suite",
        str(REFERENCE),
        str(source),
        "--output",
        str(output),
        "--junit",
        str(output),
    ]
    assert main(args) == 2 and not output.exists()
