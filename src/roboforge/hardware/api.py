"""Local HTTP adapter for hardware experiments; only built-in controllers run."""

import hashlib
import json
from importlib.resources import files

import yaml
from fastapi import APIRouter, Query, Request
from fastapi.responses import JSONResponse, Response

from roboforge.ci_reports import rover_suite_junit
from roboforge.config import Schema

from .project import RobotProject
from .rover_comparison import compare_rover_suites
from .rover_report import project_rover_evidence, render_rover_evidence
from .rover_sizing import RoverSizingConfig, size_rover
from .validation import check_project
from .workspace import MAX_REPORT_BYTES, TEMPLATES, HardwareRunRequest, hardware_template


class SizingRequest(Schema):
    project: RobotProject
    config: RoverSizingConfig


def hardware_router():
    router = APIRouter(prefix="/api/hardware", tags=["hardware"])

    @router.get("/templates")
    def templates():
        return [{"id": key, "title": value[0]} for key, value in TEMPLATES.items()]

    @router.get("/templates/{template_id}")
    def template(template_id: str):
        return hardware_template(template_id)

    @router.post("/validate")
    def validate(project: RobotProject):
        diagnostics = [d.model_dump(mode="json") for d in check_project(project)]
        return {"project": project.model_dump(mode="json"), "diagnostics": diagnostics}

    @router.post("/specification/validate")
    def validate_specification(specification: HardwareRunRequest):
        return {
            **specification.model_dump(mode="json"),
            "diagnostics": [
                d.model_dump(mode="json") for d in check_project(specification.project)
            ],
        }

    @router.post("/sizing")
    def sizing(specification: SizingRequest):
        try:
            return size_rover(specification.project, specification.config)
        except ValueError as exc:
            return JSONResponse({"detail": str(exc)}, status_code=422)

    @router.get("/sizing/defaults")
    def sizing_defaults():
        text = (
            files("roboforge")
            .joinpath("hardware/templates/esp32_rover/rover_sizing.yaml")
            .read_text(encoding="utf-8")
        )
        return RoverSizingConfig.model_validate(yaml.safe_load(text)).model_dump(mode="json")

    @router.get("/comparisons")
    def comparison(
        request: Request,
        baseline: str = Query(pattern=r"^hardware-[0-9a-f]{32}$"),
        candidate: str = Query(pattern=r"^hardware-[0-9a-f]{32}$"),
        format: str = Query("summary", pattern="^(summary|json|html)$"),
    ):
        service = request.app.state.hardware
        before, before_hash = service.report(baseline)
        after, after_hash = service.report(candidate)
        try:
            result = compare_rover_suites(
                before, after, baseline_sha256=before_hash, candidate_sha256=after_hash
            )
            result["workspace_runs"] = {"baseline": baseline, "candidate": candidate}
            if format == "summary":
                return {
                    **{key: value for key, value in result.items() if key != "runs"},
                    "run_metadata": {
                        label: {
                            **service.get(job_id),
                            "mass_kg": report["project"]["assembly"]["total_mass_kg"],
                        }
                        for label, job_id, report in (
                            ("baseline", baseline, before),
                            ("candidate", candidate, after),
                        )
                    },
                }
            raw = (json.dumps(result, indent=2, allow_nan=False) + "\n").encode("utf-8")
            if len(raw) > MAX_REPORT_BYTES * 2:
                raise ValueError("combined comparison export exceeds 128 MiB")
            headers = {
                "X-Content-Type-Options": "nosniff",
                "Content-Disposition": f'attachment; filename="ixion-design-comparison.{format}"',
            }
            if format == "json":
                return Response(raw, media_type="application/json", headers=headers)
            headers["Content-Security-Policy"] = "sandbox allow-scripts; frame-ancestors 'self'"
            return Response(
                render_rover_evidence(result, hashlib.sha256(raw).hexdigest()),
                media_type="text/html",
                headers=headers,
            )
        except ValueError as exc:
            return JSONResponse({"detail": str(exc)}, status_code=422)

    @router.post("/runs", status_code=202)
    def submit(specification: HardwareRunRequest, request: Request):
        return request.app.state.hardware.submit(specification)

    @router.get("/runs")
    def runs(request: Request, offset: int = Query(0, ge=0), limit: int = Query(20, ge=1, le=100)):
        return request.app.state.hardware.list(offset, limit)

    @router.get("/runs/{job_id}")
    def run(job_id: str, request: Request):
        return request.app.state.hardware.get(job_id)

    @router.get("/runs/{job_id}/specification")
    def specification(job_id: str, request: Request):
        return request.app.state.hardware.specification(job_id)

    @router.get("/runs/{job_id}/view")
    def view(job_id: str, request: Request):
        evidence, fingerprint = request.app.state.hardware.report(job_id)
        try:
            return {
                "job_id": job_id,
                "project": RobotProject.model_validate(evidence["project"]).model_dump(mode="json"),
                "evidence": project_rover_evidence(evidence, fingerprint),
            }
        except (KeyError, ValueError) as exc:
            return JSONResponse({"detail": str(exc)}, status_code=422)

    @router.get("/runs/{job_id}/report")
    def report(
        job_id: str, request: Request, format: str = Query("json", pattern="^(json|html|xml)$")
    ):
        evidence, fingerprint = request.app.state.hardware.report(job_id)
        headers = {"X-Content-Type-Options": "nosniff"}
        if format == "html":
            headers["Content-Security-Policy"] = "sandbox allow-scripts; frame-ancestors 'self'"
            return Response(
                render_rover_evidence(evidence, fingerprint),
                media_type="text/html",
                headers=headers,
            )
        headers["Content-Disposition"] = f'attachment; filename="{job_id}.{format}"'
        if format == "xml":
            return Response(
                rover_suite_junit(evidence), media_type="application/xml", headers=headers
            )
        return JSONResponse(evidence, headers=headers)

    return router
