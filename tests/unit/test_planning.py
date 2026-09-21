import pytest

from roboforge.config import Environment, Rectangle
from roboforge.geometry import Vector2
from roboforge.planning import PlanningWorld, grid_plan, path_length, sampling_plan


def room():
    return Environment(width=5, height=5, obstacles=(Rectangle(x=2, y=0, width=0.1, height=3.5),))


def assert_valid(result, env, start, goal):
    assert result.status == "success"
    assert result.path[0] == start and result.path[-1] == goal
    oracle = PlanningWorld(env, 0.15)
    assert all(oracle.segment_free(a, b) for a, b in zip(result.path, result.path[1:]))
    assert result.length == pytest.approx(path_length(result.path))


def test_astar_matches_dijkstra_and_avoids_wall():
    env, start, goal = room(), Vector2(1, 1), Vector2(4, 1)
    astar = grid_plan(env, start, goal, 0.15, resolution=0.5)
    dijkstra = grid_plan(env, start, goal, 0.15, resolution=0.5, algorithm="dijkstra")
    assert_valid(astar, env, start, goal)
    assert_valid(dijkstra, env, start, goal)
    assert astar.length == pytest.approx(dijkstra.length)
    assert astar.expanded <= dijkstra.expanded
    assert max(p.y for p in astar.path) > 3.5


def test_thin_wall_sweep_blocks_clear_endpoints():
    world = PlanningWorld(room(), 0.15)
    assert world.point_free(Vector2(1, 1)) and world.point_free(Vector2(4, 1))
    assert not world.segment_free(Vector2(1, 1), Vector2(4, 1))


def test_no_path_and_budget_distinct():
    env = Environment(width=5, height=5, obstacles=(Rectangle(x=2, y=0, width=0.1, height=5),))
    assert grid_plan(env, Vector2(1, 1), Vector2(4, 1), 0.15, 0.5).status == "no_path"
    assert (
        grid_plan(room(), Vector2(1, 1), Vector2(4, 1), 0.15, 0.5, max_expansions=1).status
        == "budget_exceeded"
    )


@pytest.mark.parametrize("planner", [grid_plan, sampling_plan])
def test_invalid_endpoints_and_identity(planner):
    env, valid = room(), Vector2(1, 1)
    assert planner(env, Vector2(0, 0), valid, 0.15).status == "invalid_start"
    assert planner(env, valid, Vector2(2, 1), 0.15).status == "invalid_goal"
    result = planner(env, valid, valid, 0.15)
    assert result.status == "success" and result.path == (valid,) and result.length == 0


@pytest.mark.parametrize("algorithm", ["rrt", "rrt_star"])
def test_sampling_deterministic_and_safe(algorithm):
    env, start, goal = room(), Vector2(1, 1), Vector2(4, 1)
    result = sampling_plan(env, start, goal, 0.15, algorithm=algorithm, iterations=500, seed=17)
    assert_valid(result, env, start, goal)
    assert result == sampling_plan(
        env, start, goal, 0.15, algorithm=algorithm, iterations=500, seed=17
    )


def test_rrt_star_budget_extension_does_not_worsen_path():
    env, start, goal = Environment(width=5, height=5), Vector2(1, 1), Vector2(4, 4)
    a = sampling_plan(env, start, goal, 0.15, algorithm="rrt_star", iterations=100)
    b = sampling_plan(env, start, goal, 0.15, algorithm="rrt_star", iterations=250)
    assert_valid(a, env, start, goal)
    assert_valid(b, env, start, goal)
    assert b.length <= a.length + 1e-10


def test_sampling_exhaustion_is_not_infeasibility():
    result = sampling_plan(room(), Vector2(1, 1), Vector2(4, 1), 0.15, iterations=10, goal_bias=1)
    assert result.status == "budget_exceeded" and not result.path


@pytest.mark.parametrize(
    "kwargs",
    [
        {"iterations": 0},
        {"step_size": 0},
        {"seed": True},
        {"goal_bias": 1.1},
        {"algorithm": "fake"},
    ],
)
def test_sampling_validation(kwargs):
    with pytest.raises(ValueError):
        sampling_plan(room(), Vector2(1, 1), Vector2(4, 1), 0.15, **kwargs)
