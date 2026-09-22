"""Incremental scan-to-map alignment under biased odometry calibration."""

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
from roboforge.mapping import GridConfig
from roboforge.odometry import EncoderOdometry
from roboforge.sensors.readings import EncoderReading, LidarReading
from roboforge.simulation import Simulator
from roboforge.slam import IncrementalSlam, SlamConfig


def run():
    config = RunConfig.model_validate(
        {
            "name": "incremental_scan_slam",
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
            "commands": [{"left": 4, "right": 7, "steps": 600}],
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
    slam = IncrementalSlam(
        GridConfig(columns=170, rows=170, resolution=0.05, origin_x=-0.25, origin_y=-0.25),
        SlamConfig(),
        mount=Pose2(0.1, 0),
    )
    trajectory = []
    for scan in result.readings:
        if isinstance(scan, LidarReading):
            prior = estimates[scan.capture_time]
            trajectory.append(slam.update(scan, prior.pose, prior_time=prior.time))
    truth = result.states[-1].pose

    def error(pose):
        return math.hypot(pose.x - truth.x, pose.y - truth.y)

    summary = {
        "matched_scans": sum(e.status == "matched" for e in trajectory),
        "rejected_scans": sum(e.status == "rejected" for e in trajectory),
        "odometry_endpoint_error_m": error(estimates[result.states[-1].time].pose),
        "slam_endpoint_error_m": error(trajectory[-1].pose),
        "point_map_size": len(slam.points),
    }
    assert summary["matched_scans"] >= 50
    assert summary["slam_endpoint_error_m"] < summary["odometry_endpoint_error_m"] * 0.5
    return result, list(estimates.values()), slam, trajectory, summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_11")
    args = parser.parse_args()
    result, odometry, slam, trajectory, summary = run()
    repeated = run()
    assert trajectory == repeated[3] and summary == repeated[4]
    save_result(result, args.output)
    slam.grid.save(args.output / "occupancy.npz")
    (args.output / "slam.json").write_text(
        json.dumps([asdict(e) for e in trajectory], indent=2), encoding="utf-8"
    )
    (args.output / "slam_config.json").write_text(
        slam.config.model_dump_json(indent=2), encoding="utf-8"
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    axes[0].plot(
        [s.pose.x for s in result.states],
        [s.pose.y for s in result.states],
        color="black",
        label="Truth",
    )
    axes[0].plot(
        [e.pose.x for e in odometry], [e.pose.y for e in odometry], label="Biased odometry"
    )
    axes[0].plot(
        [e.pose.x for e in trajectory], [e.pose.y for e in trajectory], label="Scan-to-map SLAM"
    )
    axes[0].set_title("Estimated trajectory · no loop closure")
    axes[0].legend()
    axes[1].imshow(
        slam.grid.probabilities,
        origin="lower",
        extent=(-0.25, 8.25, -0.25, 8.25),
        cmap="gray_r",
        vmin=0,
        vmax=1,
    )
    axes[1].plot([e.pose.x for e in trajectory], [e.pose.y for e in trajectory], color="#00a3b3")
    axes[1].set_title("Incremental occupancy reconstruction")
    for ax in axes:
        ax.set(xlabel="x [m]", ylabel="y [m]")
        ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(args.output / "slam.png", dpi=150)
    plt.close(fig)
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
