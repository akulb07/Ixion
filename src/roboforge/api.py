"""Optional FastAPI transport for the local RoboForge run service."""

from contextlib import asynccontextmanager
from dataclasses import asdict
from pathlib import Path
from urllib.parse import urlsplit

from fastapi import FastAPI, Query, Request
from fastapi.responses import FileResponse, JSONResponse
from starlette.middleware.trustedhost import TrustedHostMiddleware
from starlette.staticfiles import StaticFiles

from roboforge import __version__
from roboforge.batch_service import BatchRequest, BatchService, prepare_batch
from roboforge.comparison import ComparisonRequest, compare_runs
from roboforge.config import RunConfig
from roboforge.experiments import paired_differences
from roboforge.odometry_analysis import analyze_odometry
from roboforge.planning_service import PlanningRequest, PlanningService
from roboforge.service import RunService, ServiceError, resource_estimate


class BodyLimitMiddleware:
    """Bound bytes received, including chunked bodies without Content-Length."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http" or scope["method"] not in {"POST", "PUT", "PATCH", "DELETE"}:
            return await self.app(scope, receive, send)
        body = bytearray()
        while True:
            message = await receive()
            if message["type"] == "http.disconnect":
                return
            body.extend(message.get("body", b""))
            if len(body) > 1_000_000:
                response = JSONResponse({"detail": "request exceeds 1 MB limit"}, status_code=413)
                return await response(scope, receive, send)
            if not message.get("more_body", False):
                break
        delivered = False

        async def buffered_receive():
            nonlocal delivered
            if not delivered:
                delivered = True
                return {"type": "http.request", "body": bytes(body), "more_body": False}
            return await receive()

        await self.app(scope, buffered_receive, send)


def presets():
    base = {
        "name": "sensor_lab",
        "seed": 42,
        "robot": {"initial_pose": {"x": 2, "y": 2}},
        "environment": {
            "width": 8,
            "height": 8,
            "obstacles": [
                {"type": "rectangle", "x": 5, "y": 2, "width": 1, "height": 3},
                {"type": "circle", "x": 1.5, "y": 5, "radius": 0.6},
            ],
        },
        "simulation": {"dt": 0.02, "collision": {"mode": "stop"}},
        "commands": [{"left": 4, "right": 7, "steps": 500}],
        "sensors": [
            {"type": "encoder", "rate_hz": 50},
            {"type": "imu", "rate_hz": 50},
            {"type": "lidar", "rate_hz": 5, "rays": 90},
        ],
    }
    normal = RunConfig.model_validate(base)
    faulted = RunConfig.model_validate(
        {
            **base,
            "name": "slip_lab",
            "faults": [
                {
                    "name": "left_slip",
                    "kind": "wheel_slip",
                    "target": "left",
                    "start": 2,
                    "end": 4,
                    "magnitude": 0.5,
                }
            ],
        }
    )
    return [
        {"id": "sensors", "title": "Sensor laboratory", "config": normal.model_dump(mode="json")},
        {"id": "slip", "title": "Wheel slip laboratory", "config": faulted.model_dump(mode="json")},
    ]


def create_app(directory: str | Path = "results/service") -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        app.state.service = RunService(directory)
        app.state.planner = PlanningService()
        try:
            app.state.batches = BatchService(app.state.service)
            try:
                yield
            finally:
                app.state.batches.close()
        finally:
            app.state.service.close()

    app = FastAPI(
        title="RoboForge Local API",
        version=__version__,
        lifespan=lifespan,
        description="Validated simulation runs and recorded replay. Local single-user service.",
    )
    app.add_middleware(
        TrustedHostMiddleware, allowed_hosts=["localhost", "127.0.0.1", "[::1]", "testserver"]
    )
    app.add_middleware(BodyLimitMiddleware)

    @app.middleware("http")
    async def local_requests(request: Request, call_next):
        if request.method in {"POST", "PUT", "PATCH", "DELETE"}:
            origin = request.headers.get("origin")
            if origin:
                try:
                    parsed = urlsplit(origin)
                    same = (
                        parsed.scheme,
                        parsed.hostname,
                        parsed.port or (443 if parsed.scheme == "https" else 80),
                    ) == (
                        request.url.scheme,
                        request.url.hostname,
                        request.url.port or (443 if request.url.scheme == "https" else 80),
                    )
                except ValueError:
                    same = False
                if not same:
                    return JSONResponse(
                        {"detail": "cross-origin writes are disabled"}, status_code=403
                    )
            try:
                length = int(request.headers.get("content-length", "0"))
            except ValueError:
                return JSONResponse({"detail": "invalid content length"}, status_code=400)
            if length < 0:
                return JSONResponse({"detail": "invalid content length"}, status_code=400)
            if length > 1_000_000:
                return JSONResponse({"detail": "request exceeds 1 MB limit"}, status_code=413)
        return await call_next(request)

    @app.exception_handler(ServiceError)
    async def service_error(request, error):
        return JSONResponse({"detail": str(error)}, status_code=error.status)

    @app.get("/api/health")
    def health(request: Request):
        return {
            "status": "ok",
            "version": __version__,
            "recovery_warnings": request.app.state.service.recovery_warnings
            + request.app.state.batches.recovery_warnings,
        }

    @app.get("/api/config/schema")
    def schema():
        return RunConfig.model_json_schema()

    @app.get("/api/presets")
    def get_presets():
        return presets()

    @app.post("/api/config/validate")
    def validate(config: RunConfig):
        return {"config": config.model_dump(mode="json"), "resources": resource_estimate(config)}

    @app.post("/api/runs", status_code=202)
    def submit(config: RunConfig, request: Request):
        return request.app.state.service.submit(config)

    @app.post("/api/plans")
    def plan(specification: PlanningRequest, request: Request):
        return request.app.state.planner.plan(specification)

    @app.post("/api/comparisons")
    def compare(specification: ComparisonRequest, request: Request):
        return compare_runs(request.app.state.service, specification)

    @app.post("/api/experiments/preview")
    def preview_batch(specification: BatchRequest):
        return prepare_batch(specification)[0]

    @app.post("/api/experiments", status_code=202)
    def submit_batch(specification: BatchRequest, request: Request):
        return request.app.state.batches.submit(specification)

    @app.get("/api/experiments")
    def list_batches(
        request: Request, offset: int = Query(0, ge=0), limit: int = Query(20, ge=1, le=100)
    ):
        return request.app.state.batches.list(offset, limit)

    @app.get("/api/experiments/{batch_id}")
    def get_batch(batch_id: str, request: Request):
        return request.app.state.batches.get(batch_id)

    @app.post("/api/experiments/{batch_id}/cancel")
    def cancel_batch(batch_id: str, request: Request):
        return request.app.state.batches.cancel(batch_id)

    @app.get("/api/experiments/{batch_id}/report")
    def batch_report(batch_id: str, request: Request):
        return JSONResponse(
            request.app.state.batches.get(batch_id),
            headers={"Content-Disposition": f'attachment; filename="{batch_id}-report.json"'},
        )

    @app.get("/api/experiments/{batch_id}/artifacts/{filename}")
    def batch_artifact(batch_id: str, filename: str, request: Request):
        return FileResponse(
            request.app.state.batches.artifact(batch_id, filename), filename=filename
        )

    @app.get("/api/experiments/{batch_id}/paired")
    def paired_batch(
        batch_id: str,
        request: Request,
        baseline: str = Query(min_length=1, max_length=64),
        challenger: str = Query(min_length=1, max_length=64),
        metric: str = Query(min_length=1, max_length=128),
    ):
        batch = request.app.state.batches.get(batch_id)
        try:
            result = paired_differences(batch, baseline, challenger, metric)
        except ValueError as exc:
            raise ServiceError(str(exc), 422) from exc
        return JSONResponse(
            {"format_version": 1, "batch_id": batch_id, "batch_status": batch["status"], **result},
            headers={"Content-Disposition": f'attachment; filename="{batch_id}-paired.json"'},
        )

    @app.get("/api/runs")
    def list_runs(
        request: Request, offset: int = Query(0, ge=0), limit: int = Query(50, ge=1, le=100)
    ):
        return request.app.state.service.list(offset, limit)

    @app.get("/api/runs/{run_id}")
    def get_run(run_id: str, request: Request):
        return request.app.state.service.get(run_id)

    @app.get("/api/runs/{run_id}/config")
    def get_config(run_id: str, request: Request):
        return request.app.state.service.config(run_id).model_dump(mode="json")

    @app.post("/api/runs/{run_id}/cancel")
    def cancel(run_id: str, request: Request):
        return request.app.state.service.cancel(run_id)

    @app.get("/api/runs/{run_id}/frame")
    def frame(run_id: str, request: Request, time: float = Query(ge=0, allow_inf_nan=False)):
        service = request.app.state.service
        replay = service.replay(run_id)
        try:
            snapshot = replay.at(time)
        except ValueError as exc:
            raise ServiceError(str(exc), 422) from exc
        latest = {}
        for reading in snapshot.readings:
            latest[reading.sensor] = reading.model_dump(mode="json")
        active = [
            fault.model_dump(mode="json")
            for fault in service.config(run_id).faults
            if fault.start <= time and (fault.end is None or time < fault.end)
        ]
        return {
            "state": asdict(snapshot.state),
            "sensors": list(latest.values()),
            "sensor_capture_poses": {
                name: asdict(replay.state_at(reading["capture_time"]).pose)
                for name, reading in latest.items()
            },
            "control": snapshot.control,
            "actuator": snapshot.actuator,
            "navigation": snapshot.navigation,
            "active_faults": active,
        }

    @app.get("/api/runs/{run_id}/trajectory")
    def trajectory(run_id: str, request: Request, max_points: int = Query(1000, ge=2, le=10000)):
        states = request.app.state.service.replay(run_id).states
        count = min(len(states), max_points)
        indices = (
            [round(i * (len(states) - 1) / (count - 1)) for i in range(count)] if count > 1 else [0]
        )
        return {
            "total_states": len(states),
            "sampled": count < len(states),
            "states": [asdict(states[i]) for i in indices],
        }

    @app.get("/api/runs/{run_id}/artifacts/{filename}")
    def artifact(run_id: str, filename: str, request: Request):
        return FileResponse(request.app.state.service.artifact(run_id, filename), filename=filename)

    @app.get("/api/runs/{run_id}/odometry")
    def odometry(
        run_id: str,
        request: Request,
        sensor: str = Query(min_length=1, max_length=128),
        max_points: int = Query(1000, ge=2, le=2000),
    ):
        result = analyze_odometry(request.app.state.service.replay(run_id), sensor, max_points)
        return JSONResponse(
            {"run_id": run_id, **result},
            headers={"Content-Disposition": f'attachment; filename="{run_id}-odometry.json"'},
        )

    web = Path(__file__).parent / "web"
    if web.is_dir():
        app.mount("/assets", StaticFiles(directory=web / "assets"), name="workspace-assets")

        @app.get("/", include_in_schema=False)
        def workspace():
            return FileResponse(web / "index.html", headers={"Cache-Control": "no-cache"})

    return app
