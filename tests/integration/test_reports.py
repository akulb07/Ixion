import copy
import csv
import hashlib
import io
import json
from html.parser import HTMLParser

from fastapi.testclient import TestClient

from roboforge.api import create_app
from roboforge.benchmarks import BenchmarkConfig, run_benchmarks
from roboforge.experiments import ExperimentConfig, run_experiment
from roboforge.reports import report_html, trial_csv
from tests.integration.test_batches import batch_finished, specification
from tests.integration.test_experiments import base


class ReportParser(HTMLParser):
    def __init__(self, text):
        super().__init__()
        self.tags = []
        self.tables = []
        self.row = None
        self.cell = None
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        self.tags.append(tag)
        if tag == "table":
            self.tables.append([])
        if tag == "tr":
            self.row = []
        if tag in ("td", "th"):
            self.cell = ""

    def handle_data(self, data):
        if self.cell is not None:
            self.cell += data

    def handle_endtag(self, tag):
        if tag in ("td", "th"):
            self.row.append(self.cell)
            self.cell = None
        if tag == "tr":
            self.tables[-1].append(self.row)


def test_report_preserves_missing_zero_negative_and_escapes_recorded_text():
    report = {
        "name": '</title><script>alert("bad")</script>',
        "status": "running",
        "expected_trials": 3,
        "finished_trials": 2,
        "trials": [
            {"group": "g", "seed": 1, "status": "completed", "metrics": {"error": -2, "zero": 0}},
            {
                "group": "g",
                "seed": 2,
                "status": "failed",
                "metrics": {},
                "error": '=HYPERLINK("bad")\n<img src=x onerror=bad>',
            },
            {"group": "g", "seed": 3, "status": "pending", "metrics": {}},
        ],
        "groups": {},
    }
    before = copy.deepcopy(report)
    rows = list(csv.DictReader(io.StringIO(trial_csv(report))))
    assert [r["status"] for r in rows] == ["completed", "failed", "pending"]
    assert rows[0]["metric.error"] == "-2" and rows[0]["metric.zero"] == "0"
    assert rows[1]["metric.error"] == rows[2]["metric.zero"] == ""
    assert rows[1]["error"] == "'" + report["trials"][1]["error"]
    html = report_html(report, design={"name": report["name"]})
    parsed = ReportParser(html)
    assert "script" not in parsed.tags and "img" not in parsed.tags
    assert "&lt;script&gt;" in html and "saved snapshot" in html
    assert report == before


def test_browser_exports_match_saved_trials_and_leave_store_unchanged(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        spec = specification(axes=[{"path": "robot.wheel_radius", "values": [-1, 0.05]}])
        result = client.post("/api/experiments", json=spec.model_dump(mode="json"))
        batch_id = result.json()["id"]
        snapshot = batch_finished(client.app.state.batches, batch_id)
        before = {p: p.read_bytes() for p in tmp_path.rglob("*.json")}
        url = f"/api/experiments/{batch_id}/report"
        assert client.get(url).json() == snapshot
        response = client.get(url + "?format=csv")
        assert response.status_code == 200 and "text/csv" in response.headers["content-type"]
        assert response.headers["content-disposition"].endswith('-report.csv"')
        rows = list(csv.DictReader(io.StringIO(response.text)))
        for row, trial in zip(rows, snapshot["trials"], strict=True):
            assert row["status"] == trial["status"]
            assert int(row["seed"]) == trial["seed"]
            assert json.loads(row["parameters"]) == trial["parameters"]
            for metric, value in trial["metrics"].items():
                assert float(row[f"metric.{metric}"]) == value
        assert rows[0]["metric.duration_s"] == "" and rows[0]["error"]
        html = client.get(url + "?format=html")
        assert html.status_code == 200 and "text/html" in html.headers["content-type"]
        assert "default-src 'none'" in html.headers["content-security-policy"]
        assert snapshot["specification_sha256"] in html.text
        parsed = ReportParser(html.text)
        # The summary retains the invalid group with a 100% failure rate.
        assert ["group-0000", "2", "2", "1.0"] == parsed.tables[1][1][:4]
        assert client.get(url + "?format=pdf").status_code == 422
        assert client.get("/api/experiments/missing/report?format=html").status_code == 404
        assert {p: p.read_bytes() for p in before} == before


def test_benchmark_portable_exports_include_failures_geometry_and_verified_hashes(tmp_path):
    spec = BenchmarkConfig(cases=("empty_room",), algorithms=("astar", "rrt"), iterations=1)
    root = run_benchmarks(spec, tmp_path)
    report = json.loads((root / "report.json").read_text())
    rows = list(csv.DictReader(io.StringIO((root / "trials.csv").read_text())))
    assert report["status"] == "completed" and report["finished_trials"] == 2
    assert rows[0]["status"] == "success"
    assert float(rows[0]["path_length_m"]) == report["trials"][0]["path_length_m"]
    assert rows[1]["status"] == "budget_exceeded" and rows[1]["path_length_m"] == ""
    html = (root / "report.html").read_text(encoding="utf-8")
    assert "Exact benchmark geometry" in html and "rewire_radius" in html
    assert report["specification_sha256"] in html
    manifest = json.loads((root / "manifest.json").read_text())["files"]
    for name in ("report.html", "trials.csv", "report.json", "cases.json"):
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == manifest[name]


def test_cli_experiment_exports_recorded_results_and_manifest(tmp_path):
    root = run_experiment(ExperimentConfig(base=base()), tmp_path)
    report = json.loads((root / "report.json").read_text())
    rows = list(csv.DictReader(io.StringIO((root / "trials.csv").read_text())))
    assert float(rows[0]["metric.duration_s"]) == report["trials"][0]["metrics"]["duration_s"]
    assert report["software_version"] and report["finished_utc"]
    html = (root / "report.html").read_text(encoding="utf-8")
    assert report["trials"][0]["config_sha256"] in html
    assert "Design and budgets" in html
    manifest = json.loads((root / "manifest.json").read_text())["files"]
    assert (
        hashlib.sha256((root / "report.html").read_bytes()).hexdigest() == manifest["report.html"]
    )
