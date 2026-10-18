import hashlib
import json
from xml.etree import ElementTree as ET

from roboforge.cli import main


def test_starter_demo_runs_packaged_project_and_saves_linked_offline_evidence(tmp_path):
    root = tmp_path / "first rover"
    assert main(["new-project", str(root), "--demo"]) == 0
    manifest = json.loads((root / "starter-manifest.json").read_text())
    assert manifest["demo"] == {
        "status": "passed",
        "total": 4,
        "passed": 4,
        "failed": 0,
        "error": 0,
    }
    report = json.loads((root / "results/rover-suite.json").read_text())
    assert [case["name"] for case in report["cases"]] == [
        "normal",
        "left_counter_freeze",
        "both_counters_freeze",
        "left_counter_jump",
    ]
    assert ET.parse(root / "results/rover-suite.xml").getroot().get("failures") == "0"
    html = (root / "results/rover-suite.html").read_text(encoding="utf-8")
    fingerprint = hashlib.sha256((root / "results/rover-suite.json").read_bytes()).hexdigest()
    assert fingerprint in html and "default-src 'none'" in html
    for name, expected in manifest["files"].items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected["sha256"]
