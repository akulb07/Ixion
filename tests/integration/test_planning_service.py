import hashlib
import json

import pytest
from pydantic import ValidationError

from roboforge.geometry import Vector2
from roboforge.planning import PlanningWorld
from roboforge.planning_service import PlanningRequest, PlanningService
from roboforge.service import ServiceError


def specification(**changes):
    return PlanningRequest.model_validate(
        {
            "environment": {"width": 3, "height": 3},
            "start": {"x": 0.5, "y": 0.5},
            "goal": {"x": 2.5, "y": 2.5},
            "footprint_radius": 0.1,
            "budget": 150,
            **changes,
        }
    )


@pytest.mark.parametrize("algorithm", ["astar", "dijkstra", "rrt", "rrt_star"])
def test_plans_reproduce_and_every_segment_is_collision_checked(algorithm):
    request = specification(algorithm=algorithm)
    service = PlanningService()
    document = service.plan(request)
    assert document == service.plan(request)
    assert document["result"]["status"] == "success"
    assert document["planning_radius_m"] == 0.2
    path = tuple(Vector2(**point) for point in document["result"]["path"])
    world = PlanningWorld(request.environment, document["planning_radius_m"])
    assert all(world.segment_free(a, b) for a, b in zip(path, path[1:]))
    assert path[0] == request.start.vector() and path[-1] == request.goal.vector()
    digest = hashlib.sha256(
        json.dumps(document["request"], sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    assert digest == document["request_sha256"]


def test_failed_and_trivial_searches_are_retained():
    service = PlanningService()
    failed = service.plan(specification(algorithm="rrt", budget=1, goal_bias=0))
    assert failed["result"]["status"] == "budget_exceeded"
    assert failed["result"]["path"] == ()
    invalid = service.plan(specification(goal={"x": 0.05, "y": 0.05}))
    assert invalid["result"]["status"] == "invalid_goal"
    same = service.plan(specification(goal={"x": 0.5, "y": 0.5}))
    assert same["result"]["status"] == "success"
    assert same["result"]["length"] == 0 and len(same["result"]["path"]) == 1


@pytest.mark.parametrize(
    "changes",
    [
        {"budget": 1001},
        {"budget": True},
        {"goal": {"x": 4, "y": 2}},
        {"environment": {"width": 1001, "height": 3}},
        {"environment": {"width": 100, "height": 100}, "resolution": 0.05},
        {"footprint_radius": -1},
        {"seed": 9007199254740992},
        {"algorithm": "invented"},
    ],
)
def test_unbounded_or_invalid_requests_are_rejected(changes):
    with pytest.raises(ValidationError):
        specification(**changes)


def test_single_active_search_and_lock_released_after_failure(monkeypatch):
    import roboforge.planning_service as module

    service = PlanningService()
    with service._lock:
        with pytest.raises(ServiceError) as error:
            service.plan(specification())
        assert error.value.status == 429

    def fail(*args, **kwargs):
        raise RuntimeError("test search failure")

    with monkeypatch.context() as patch:
        patch.setattr(module, "grid_plan", fail)
        with pytest.raises(RuntimeError):
            service.plan(specification())
    assert service.plan(specification())["result"]["status"] == "success"


def test_planning_endpoint_does_not_submit_simulation(tmp_path):
    pytest.importorskip("fastapi")
    from fastapi.testclient import TestClient

    from roboforge.api import create_app

    with TestClient(create_app(tmp_path)) as client:
        response = client.post("/api/plans", json=specification().model_dump(mode="json"))
        assert response.status_code == 200
        assert response.json()["result"]["status"] == "success"
        assert client.get("/api/runs").json()["total"] == 0
        bad = specification().model_dump(mode="json")
        bad["budget"] = 1001
        assert client.post("/api/plans", json=bad).status_code == 422
        assert (
            client.post(
                "/api/plans", json={}, headers={"Origin": "https://example.com"}
            ).status_code
            == 403
        )
