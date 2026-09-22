import json

import pytest

from roboforge.benchmarks import BenchmarkConfig, run_benchmarks, standard_cases
from roboforge.planning import PlanningWorld, grid_plan


@pytest.mark.parametrize("name", ["empty_room", "corridor", "maze", "clutter"])
def test_standard_case_has_valid_endpoints_and_grid_route(name):
    case = standard_cases()[name]
    world = PlanningWorld(case.environment, 0.2)
    assert world.point_free(case.start) and world.point_free(case.goal)
    plan = grid_plan(case.environment, case.start, case.goal, 0.2, 0.5)
    assert plan.status == "success"
    assert all(world.segment_free(a, b) for a, b in zip(plan.path, plan.path[1:]))


def test_benchmark_comparison_artifacts_and_budget_failures(tmp_path):
    config = BenchmarkConfig(
        cases=("empty_room",), algorithms=("astar", "dijkstra", "rrt"), iterations=1
    )
    root = run_benchmarks(config, tmp_path)
    report = json.loads((root / "report.json").read_text())
    records = report["trials"]
    assert len(records) == 3
    assert records[0]["path_length_m"] == pytest.approx(records[1]["path_length_m"])
    assert records[0]["verified_collision_free"] and records[1]["verified_collision_free"]
    assert records[2]["status"] == "budget_exceeded" and records[2]["path_length_m"] is None
    assert records[0]["planner_runtime_s"] >= 0
    assert (root / "manifest.json").exists() and (root / "summary.csv").exists()


def test_duplicate_design_rejected():
    with pytest.raises(ValueError):
        BenchmarkConfig(seeds=(1, 1))
