"""Compare four actual collision-checked planners on one known map."""

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.config import Environment, Rectangle
from roboforge.geometry import Vector2
from roboforge.planning import PlanningWorld, grid_plan, sampling_plan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_7")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    env = Environment(width=5, height=5, obstacles=(Rectangle(x=2, y=0, width=0.1, height=3.5),))
    start, goal = Vector2(1, 1), Vector2(4, 1)
    results = [
        grid_plan(env, start, goal, 0.15, 0.25, algorithm=name) for name in ("dijkstra", "astar")
    ]
    results += [
        sampling_plan(env, start, goal, 0.15, algorithm=name, iterations=500, seed=17)
        for name in ("rrt", "rrt_star")
    ]
    world = PlanningWorld(env, 0.15)
    for result in results:
        assert result.status == "success"
        assert all(world.segment_free(a, b) for a, b in zip(result.path, result.path[1:]))
    (args.output / "plans.json").write_text(
        json.dumps([asdict(r) for r in results], indent=2), encoding="utf-8"
    )
    (args.output / "environment.json").write_text(env.model_dump_json(indent=2), encoding="utf-8")
    (args.output / "config.json").write_text(
        json.dumps(
            {
                "start": asdict(start),
                "goal": asdict(goal),
                "radius": 0.15,
                "resolution": 0.25,
                "seed": 17,
                "iterations": 500,
                "step_size": 0.5,
                "goal_bias": 0.1,
                "rewire_radius": 1.0,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle as Patch

    fig, axes = plt.subplots(2, 2, figsize=(10, 9))
    for ax, result in zip(axes.flat, results):
        ax.add_patch(Patch((2, 0), 0.1, 3.5, color="#536278"))
        ax.plot(
            [p.x for p in result.path], [p.y for p in result.path], color="#0099aa", linewidth=2
        )
        ax.scatter([start.x, goal.x], [start.y, goal.y], c=["green", "red"], zorder=3)
        ax.set(
            xlim=(0, 5),
            ylim=(0, 5),
            xlabel="x [m]",
            ylabel="y [m]",
            title=f"{result.algorithm} · {result.length:.3f} m · {result.expanded} expansions",
        )
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
    fig.suptitle("RoboForge · known-map planning · 0.15 m robot radius")
    fig.tight_layout()
    fig.savefig(args.output / "planning.png", dpi=150)
    plt.close(fig)
    print(
        json.dumps({r.algorithm: {"length_m": r.length, "expanded": r.expanded} for r in results})
    )


if __name__ == "__main__":
    main()
