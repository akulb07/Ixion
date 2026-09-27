"""Bounded, reproducible planning requests for the local workspace."""

import hashlib
import json
import math
import threading
from dataclasses import asdict
from typing import Annotated, Literal

from pydantic import Field, model_validator

from roboforge import __version__
from roboforge.config import Environment, Real, Schema
from roboforge.geometry import Vector2
from roboforge.planning import grid_plan, sampling_plan
from roboforge.service import ServiceError


class PlanningPoint(Schema):
    x: Real
    y: Real

    def vector(self):
        return Vector2(self.x, self.y)


class PlanningRequest(Schema):
    environment: Environment
    start: PlanningPoint
    goal: PlanningPoint
    footprint_radius: Annotated[Real, Field(ge=0.01, le=1000)]
    clearance: Annotated[Real, Field(ge=0, le=100)] = 0.1
    algorithm: Literal["astar", "dijkstra", "rrt", "rrt_star"] = "astar"
    seed: Annotated[int, Field(strict=True, ge=0, le=9007199254740991)] = 42
    budget: Annotated[int, Field(strict=True, ge=1, le=1000)] = 500
    resolution: Annotated[Real, Field(ge=0.05, le=1000)] = 0.25
    step_size: Annotated[Real, Field(ge=0.01, le=1000)] = 0.5
    goal_bias: Annotated[Real, Field(ge=0, le=1)] = 0.1
    rewire_radius: Annotated[Real, Field(ge=0.01, le=1000)] = 1.0

    @model_validator(mode="after")
    def bounded_world(self):
        world = self.environment
        if not 0.01 <= world.width <= 1000 or not 0.01 <= world.height <= 1000:
            raise ValueError("planning world dimensions must be between 0.01 and 1000 m")
        if len(world.obstacles) > 100 or self.budget * (len(world.obstacles) + 4) > 100000:
            raise ValueError("planning world/search workload exceeds the local API budget")
        for point in (self.start, self.goal):
            if not 0 <= point.x <= world.width or not 0 <= point.y <= world.height:
                raise ValueError("planning endpoints must lie inside world bounds")
        if self.algorithm in {"astar", "dijkstra"}:
            cells = math.floor(world.width / self.resolution) * math.floor(
                world.height / self.resolution
            )
            if cells > 40000:
                raise ValueError("planning grid exceeds 40,000 cells; increase resolution")
        return self


class PlanningService:
    """One active search per app; concurrent requests fail immediately with 429."""

    def __init__(self):
        self._lock = threading.Lock()

    def plan(self, request: PlanningRequest):
        if not self._lock.acquire(blocking=False):
            raise ServiceError(
                "a planning search is already running; try again when it finishes", 429
            )
        try:
            radius = request.footprint_radius + request.clearance
            if request.algorithm in {"astar", "dijkstra"}:
                result = grid_plan(
                    request.environment,
                    request.start.vector(),
                    request.goal.vector(),
                    radius,
                    request.resolution,
                    request.algorithm,
                    request.budget,
                )
            else:
                result = sampling_plan(
                    request.environment,
                    request.start.vector(),
                    request.goal.vector(),
                    radius,
                    request.algorithm,
                    request.seed,
                    request.budget,
                    request.step_size,
                    request.goal_bias,
                    request.rewire_radius,
                )
            document = request.model_dump(mode="json")
            digest = hashlib.sha256(
                json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            return {
                "format_version": 1,
                "software_version": __version__,
                "request_sha256": digest,
                "request": document,
                "result": asdict(result),
                "planning_radius_m": radius,
            }
        finally:
            self._lock.release()
