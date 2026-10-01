import json
from xml.etree import ElementTree as ET

import pytest

from roboforge.ci_reports import junit_report
from roboforge.cli import main
from tests.integration.test_bundles import make_bundle
from tests.integration.test_regression import policy


def test_junit_counts_failures_and_missing_evidence_without_hiding_controls():
    report = {
        "policy": {"name": 'robot <check> & "test"\x00'},
        "status": "fail",
        "software_version": "test",
        "policy_sha256": "digest",
        "comparison": {
            "baseline_id": "baseline",
            "runs": [{"id": "baseline"}, {"id": "candidate"}],
        },
        "checks": [
            {"name": "same", "status": "pass", "reason": "within bounds", "measured": 0},
            {"name": "same", "status": "fail", "reason": "error < -1 & > 1", "measured": -2},
            {"name": "missing\x01", "status": "inconclusive", "reason": "measurement unavailable"},
        ],
    }
    root = ET.fromstring(junit_report(report))
    assert root.attrib["tests"] == "3" and root.attrib["failures"] == "1"
    assert root.attrib["errors"] == "1" and root.attrib["skipped"] == "0"
    assert len({case.attrib["name"] for case in root.findall("testcase")}) == 3
    assert root.find("testcase/failure").attrib["message"] == "error < -1 & > 1"
    assert root.find("testcase/error").attrib["type"] == "InconclusiveEvidence"
    assert json.loads(root.findall("testcase")[1].find("system-out").text)["measured"] == -2


@pytest.mark.parametrize(
    "metric,maximum,exit_code,failures,errors",
    [
        ("duration_s", 1, 0, 0, 0),
        ("duration_s", -1, 3, 1, 0),
        ("unknown", 0, 4, 0, 1),
    ],
)
def test_cli_and_bundle_junit_match_evaluated_results(
    tmp_path, metric, maximum, exit_code, failures, errors
):
    design = policy(rules=[{"name": "limit", "metric": metric, "maximum": maximum}])
    store, spec, archive, report = make_bundle(tmp_path, design)
    policy_file = tmp_path / "policy.json"
    policy_file.write_text(design.model_dump_json())
    output, junit = tmp_path / "report.json", tmp_path / "report.xml"
    args = [
        "check",
        str(policy_file),
        "--store",
        str(store),
        "--baseline",
        spec.baseline_id,
        "--candidate",
        spec.candidate_id,
        "--output",
        str(output),
        "--junit",
        str(junit),
    ]
    assert main(args) == exit_code
    root = ET.parse(junit).getroot()
    assert int(root.attrib["failures"]) == failures and int(root.attrib["errors"]) == errors
    assert len(root.findall("testcase")) == len(report["checks"])
    bundle_xml = tmp_path / "bundle.xml"
    assert main(["bundle-check", str(archive), "--junit", str(bundle_xml)]) == exit_code
    assert ET.tostring(ET.parse(bundle_xml).getroot()) == ET.tostring(root)
    # An existing XML file must be refused before creating a fresh JSON report.
    args[args.index("--output") + 1] = str(tmp_path / "new.json")
    assert main(args) == 2 and not (tmp_path / "new.json").exists()
    args[-1] = str(store / "forbidden.xml")
    assert main(args) == 2 and not (store / "forbidden.xml").exists()
