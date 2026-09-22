"""Geometrically verified loop factors correct a biased odometry graph and map."""

import argparse
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.config import RunConfig
from roboforge.geometry import Pose2
from roboforge.io import save_result
from roboforge.loop_closure import LoopConfig, ScanKeyframe, optimize_scan_graph
from roboforge.mapping import GridConfig, OccupancyGrid
from roboforge.odometry import EncoderOdometry
from roboforge.sensors.readings import EncoderReading, LidarReading
from roboforge.simulation import Simulator


def run():
    config = RunConfig.model_validate(
        {
            "name": "loop_graph",
            "robot": {"initial_pose": {"x": 3, "y": 2}},
            "environment": {
                "width": 8,
                "height": 8,
                "obstacles": [
                    {"type": "rectangle", "x": 6, "y": 2, "width": 1, "height": 3},
                    {"type": "circle", "x": 1.5, "y": 5, "radius": 0.6},
                ],
            },
            "simulation": {"dt": 0.02, "collision": {"mode": "stop"}},
            "commands": [{"left": 4, "right": 7, "steps": 630}],
            "sensors": [
                {"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 65536},
                {"type": "lidar", "rate_hz": 5, "rays": 180, "max_range": 12},
            ],
        }
    )
    result = Simulator(config).run()
    assert result.status == "completed"
    odometry = EncoderOdometry(0.05, 0.33, config.robot.initial_pose.to_pose())
    estimates = {}
    for reading in result.readings:
        if isinstance(reading, EncoderReading):
            estimates[reading.capture_time] = odometry.update(reading)
    keyframes = tuple(
        ScanKeyframe(scan, estimates[scan.capture_time].pose)
        for scan in result.readings
        if isinstance(scan, LidarReading)
    )
    grid_config = GridConfig(columns=170, rows=170, resolution=0.05, origin_x=-0.25, origin_y=-0.25)
    mount = Pose2(0.1, 0)
    loop_config = LoopConfig(minimum_separation=50, search_radius=0.7)
    optimized, rebuilt = optimize_scan_graph(keyframes, grid_config, loop_config, mount=mount)
    original = OccupancyGrid(grid_config)
    for keyframe in keyframes:
        original.update(
            keyframe.scan, keyframe.pose, pose_time=keyframe.scan.capture_time, mount=mount
        )
    truth = result.states[-1].pose

    def error(pose):
        return math.hypot(pose.x - truth.x, pose.y - truth.y)

    summary = {
        "accepted_loops": sum(p.accepted for p in optimized.loops),
        "tested_candidates": len(optimized.loops),
        "optimizer_status": optimized.graph.status,
        "initial_cost": optimized.graph.costs[0],
        "final_cost": optimized.graph.costs[-1],
        "odometry_endpoint_error_m": error(keyframes[-1].pose),
        "optimized_endpoint_error_m": error(optimized.graph.poses[-1]),
    }
    assert summary["accepted_loops"] > 0
    assert summary["final_cost"] < summary["initial_cost"]
    assert summary["optimized_endpoint_error_m"] < summary["odometry_endpoint_error_m"] * 0.5
    return result, keyframes, optimized, original, rebuilt, loop_config, summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_12")
    args = parser.parse_args()
    result, frames, optimized, original, rebuilt, loop_config, summary = run()
    save_result(result, args.output)
    original.save(args.output / "before.npz")
    rebuilt.save(args.output / "after.npz")
    (args.output / "graph.json").write_text(
        json.dumps(asdict(optimized), indent=2), encoding="utf-8"
    )
    (args.output / "loop_config.json").write_text(
        loop_config.model_dump_json(indent=2), encoding="utf-8"
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    for ax, grid, poses, title in (
        (axes[0], original, [f.pose for f in frames], "Before loop correction"),
        (axes[1], rebuilt, optimized.graph.poses, "Rebuilt after graph optimization"),
    ):
        ax.imshow(
            grid.probabilities,
            origin="lower",
            extent=(-0.25, 8.25, -0.25, 8.25),
            cmap="gray_r",
            vmin=0,
            vmax=1,
        )
        ax.plot([p.x for p in poses], [p.y for p in poses], color="#00a3b3")
        ax.set(xlabel="x [m]", ylabel="y [m]", title=title)
        ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(args.output / "loop-closure.png", dpi=150)
    plt.close(fig)
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
