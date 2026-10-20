import json
import threading
import time

import pytest

from roboforge.hardware import workspace
from roboforge.hardware.workspace import HardwareRunRequest, HardwareService, hardware_template
from roboforge.service import ServiceError


def specification():
    data = hardware_template("feedback")
    case = data["suite"]["cases"][0]
    case.update(controller="sonar", acceptance={"expected_status": "completed_without_brake"})
    case["scenario"].update(duration_s=0.002)
    data["suite"]["cases"] = [case]
    return HardwareRunRequest.model_validate(data)


def finished(service, job_id):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        job = service.get(job_id)
        if job["status"] not in {"queued", "running"}:
            return job
        time.sleep(0.01)
    raise AssertionError("hardware run did not finish")


def test_saved_run_retains_its_inputs_and_recovers_after_restart(tmp_path):
    service = HardwareService(tmp_path)
    try:
        original = specification()
        submitted = service.submit(original)
        job = finished(service, submitted["id"])
        assert job["status"] == "completed" and job["outcome"] == "passed"
        assert job["summary"] == {"total": 1, "passed": 1, "failed": 0, "error": 0}
        report, fingerprint = service.report(job["id"])
        assert report["project"] == original.project.model_dump(mode="json")
        assert service.specification(job["id"]) == original.model_dump(mode="json")
        assert fingerprint == job["report_sha256"]
        assert service.list()["items"][0]["id"] == job["id"]
    finally:
        service.close()
    resumed = HardwareService(tmp_path)
    try:
        assert resumed.get(job["id"]) == job
        assert resumed.report(job["id"])[0] == report
        assert not resumed.recovery_warnings
    finally:
        resumed.close()


def test_queue_is_bounded_and_pending_reports_are_unavailable(tmp_path, monkeypatch):
    release = threading.Event()
    original = workspace.run_rover_suite

    def held(*args):
        assert release.wait(10)
        return original(*args)

    monkeypatch.setattr(workspace, "run_rover_suite", held)
    service = HardwareService(tmp_path)
    try:
        jobs = [service.submit(specification()) for _ in range(4)]
        with pytest.raises(ServiceError, match="queue is full"):
            service.submit(specification())
        with pytest.raises(ServiceError, match="no completed report"):
            service.report(jobs[0]["id"])
        release.set()
        for job in jobs:
            assert finished(service, job["id"])["status"] == "completed"
    finally:
        release.set()
        service.close()


def test_changed_report_is_rejected_and_interrupted_run_is_failed_on_recovery(tmp_path):
    service = HardwareService(tmp_path)
    try:
        job = finished(service, service.submit(specification())["id"])
    finally:
        service.close()
    folder = tmp_path / job["id"]
    job["status"] = "running"
    (folder / "job.json").write_text(json.dumps(job))
    resumed = HardwareService(tmp_path)
    try:
        assert resumed.get(job["id"])["status"] == "failed"
        assert "Service stopped" in resumed.get(job["id"])["error"]
        new_job = finished(resumed, resumed.submit(specification())["id"])
        (tmp_path / new_job["id"] / "report.json").write_text("{}")
        with pytest.raises(ServiceError, match="integrity"):
            resumed.report(new_job["id"])
    finally:
        resumed.close()


def test_execution_error_is_saved_and_unknown_ids_are_rejected(tmp_path, monkeypatch):
    def fail(*args):
        raise ValueError("fixture unavailable")

    monkeypatch.setattr(workspace, "run_rover_suite", fail)
    service = HardwareService(tmp_path)
    try:
        job = finished(service, service.submit(specification())["id"])
        assert job["status"] == "failed" and job["error"] == "fixture unavailable"
        for value in ("../job", "hardware-" + "0" * 32):
            with pytest.raises(ServiceError) as exc:
                service.get(value)
            assert exc.value.status == 404
    finally:
        service.close()


def test_hardware_http_workflow_and_shared_origin_guards(tmp_path):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from roboforge.api import create_app

    with TestClient(create_app(tmp_path)) as client:
        assert len(client.get("/api/hardware/templates").json()) == 5
        assert client.get("/api/hardware/templates/missing").status_code == 404
        request = specification().model_dump(mode="json")
        checked = client.post("/api/hardware/specification/validate", json=request)
        assert checked.status_code == 200 and checked.json()["diagnostics"]
        assert (
            client.post("/api/hardware/runs", json={**request, "code": "unsupported"}).status_code
            == 422
        )
        assert (
            client.post(
                "/api/hardware/runs", json=request, headers={"Origin": "https://example.com"}
            ).status_code
            == 403
        )
        submitted = client.post("/api/hardware/runs", json=request)
        assert submitted.status_code == 202
        job = finished(client.app.state.hardware, submitted.json()["id"])
        prefix = f"/api/hardware/runs/{job['id']}"
        assert client.get(prefix).json()["status"] == "completed"
        assert client.get(prefix + "/specification").json() == request
        assert client.get(prefix + "/report").json()["status"] == "passed"
        view = client.get(prefix + "/view")
        assert view.status_code == 200
        assert view.json()["project"] == request["project"]
        projection = view.json()["evidence"]
        assert projection["source_sha256"] == job["report_sha256"]
        samples = projection["cases"][0]["run"]["samples"]
        original_samples = client.get(prefix + "/report").json()["cases"][0]["report"]["samples"]
        assert [s["time_s"] for s in samples] == [s["time_s"] for s in original_samples]
        assert samples[-1]["distance_m"] == original_samples[-1]["distance_m"]
        assert samples[-1]["driver_modes"] == original_samples[-1]["driver_modes"]
        html = client.get(prefix + "/report?format=html")
        assert (
            html.status_code == 200
            and "sandbox allow-scripts" in html.headers["content-security-policy"]
        )
        assert "ROVER_EVIDENCE" in html.text
        assert "testsuite" in client.get(prefix + "/report?format=xml").text
        assert client.get(prefix + "/report?format=invalid").status_code == 422


def test_sizing_api_preserves_failed_demands_and_missing_assessments(tmp_path):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from roboforge.api import create_app

    with TestClient(create_app(tmp_path)) as client:
        config = client.get("/api/hardware/sizing/defaults").json()
        project = specification().project.model_dump(mode="json")
        response = client.post("/api/hardware/sizing", json={"project": project, "config": config})
        assert response.status_code == 200
        report = response.json()
        assert report["status"] == "checks_passed"
        assert report["project"]["snapshot"] == project
        assert report["engineering_readiness"] == "not_assessed"
        assert any(check["status"] == "not_assessed" for check in report["result"]["checks"])
        config["speed_m_s"] = 2
        failed = client.post("/api/hardware/sizing", json={"project": project, "config": config})
        assert failed.status_code == 200
        assert failed.json()["status"] == "checks_failed"
        assert failed.json()["result"]["required_pwm_duty"] > 1
        config["speed_m_s"] = -1
        assert (
            client.post(
                "/api/hardware/sizing", json={"project": project, "config": config}
            ).status_code
            == 422
        )


def test_wiring_transaction_changes_the_runtime_pin_map_and_keeps_diagnostics(tmp_path):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from roboforge.api import create_app

    with TestClient(create_app(tmp_path)) as client:
        original = specification()
        project = original.project.model_dump(mode="json")
        nets = {net["id"]: net for net in project["nets"]}
        nets["signal_AIN1"]["endpoints"][0]["pin"] = "GPIO26"
        nets["signal_AIN2"]["endpoints"][0]["pin"] = "GPIO25"
        checked = client.post("/api/hardware/validate", json=project)
        assert checked.status_code == 200
        assert not any(row["severity"] == "ERROR" for row in checked.json()["diagnostics"])
        job = client.app.state.hardware.submit(
            HardwareRunRequest.model_validate(
                {
                    "project": checked.json()["project"],
                    "suite": original.suite.model_dump(mode="json"),
                }
            )
        )
        assert finished(client.app.state.hardware, job["id"])["outcome"] == "passed"
        report = client.app.state.hardware.report(job["id"])[0]["cases"][0]["report"]
        assert report["output_pin_map"]["GPIO26"] == "AIN1"
        assert report["output_pin_map"]["GPIO25"] == "AIN2"
        nets["signal_AIN1"]["endpoints"].pop()
        assert client.post("/api/hardware/validate", json=project).status_code == 422
        bad_mode = original.project.model_dump(mode="json")
        next(row for row in bad_mode["assignments"] if row["endpoint"]["pin"] == "GPIO34")[
            "function"
        ] = "PWM"
        modes = client.post("/api/hardware/validate", json=bad_mode)
        assert modes.status_code == 200
        assert any(row["severity"] == "ERROR" for row in modes.json()["diagnostics"])


def test_saved_design_comparison_exports_sources_and_rejects_changed_conditions(tmp_path):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from roboforge.api import create_app
    from roboforge.hardware.rover_report import project_rover_evidence

    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.hardware
        base = specification().model_dump(mode="json")
        candidate = json.loads(json.dumps(base))
        candidate["project"]["assembly"]["total_mass_kg"] += 1
        before = finished(service, service.submit(HardwareRunRequest.model_validate(base))["id"])
        after = finished(
            service, service.submit(HardwareRunRequest.model_validate(candidate))["id"]
        )
        path = f"/api/hardware/comparisons?baseline={before['id']}&candidate={after['id']}"
        summary = client.get(path)
        assert summary.status_code == 200
        data = summary.json()
        assert "runs" not in data
        assert data["sources"]["baseline"]["sha256"] == before["report_sha256"]
        assert data["sources"]["candidate"]["sha256"] == after["report_sha256"]
        assert data["sources"]["baseline"]["hash_scope"] == "source_file_bytes"
        assert data["design_changes"]["changes"][0]["path"] == "/assembly/total_mass_kg"
        assert data["run_metadata"]["candidate"]["mass_kg"] == 3.3
        metrics = data["cases"][0]["metrics"]
        assert metrics["terminal_energy_j"]["delta"] != 0
        assert metrics["stop_time_s"]["delta"] is None
        exported = client.get(path + "&format=json")
        assert "attachment" in exported.headers["content-disposition"]
        assert exported.json()["runs"]["baseline"]["project"] == base["project"]
        assert exported.json()["runs"]["candidate"]["project"] == candidate["project"]
        assert len(project_rover_evidence(exported.json())["cases"]) == 2
        html = client.get(path + "&format=html")
        assert html.status_code == 200 and "ROVER_EVIDENCE" in html.text
        assert "sandbox allow-scripts" in html.headers["content-security-policy"]
        assert "attachment" in html.headers["content-disposition"]
        assert client.get(path + "&format=xml").status_code == 422

        candidate["suite"]["cases"][0]["scenario"]["duration_s"] = 0.003
        changed = finished(
            service, service.submit(HardwareRunRequest.model_validate(candidate))["id"]
        )
        mismatch = client.get(path.replace(after["id"], changed["id"]))
        assert mismatch.status_code == 422
        assert "different case conditions" in mismatch.json()["detail"]

        missing = json.loads(json.dumps(base))
        for component in missing["project"]["components"]:
            if component["category"] == "motor":
                component["parameters"] = [
                    p for p in component["parameters"] if p["name"] != "stall_current"
                ]
        failed = finished(service, service.submit(HardwareRunRequest.model_validate(missing))["id"])
        assert failed["summary"]["error"] == 1
        incomplete = client.get(path.replace(after["id"], failed["id"]))
        assert incomplete.status_code == 200
        assert all(
            m["candidate"] is None and m["delta"] is None
            for m in incomplete.json()["cases"][0]["metrics"].values()
        )
        assert service.report(before["id"])[1] == before["report_sha256"]
        assert service.report(after["id"])[1] == after["report_sha256"]
