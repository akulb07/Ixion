"""Build occupancy from LiDAR and encoder/gyro estimates, never truth poses."""

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np

from roboforge.config import RunConfig
from roboforge.geometry import Pose2
from roboforge.io import save_result
from roboforge.localization import EncoderImuEKF
from roboforge.mapping import GridConfig, OccupancyGrid
from roboforge.sensors.readings import EncoderReading, ImuReading, LidarReading
from roboforge.simulation import Simulator


def run():
    config = RunConfig.model_validate(
        {
            "name": "lidar_occupancy",
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
            "commands": [{"left": 4, "right": 7, "steps": 500}],
            "sensors": [
                {"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 65536},
                {"type": "imu", "rate_hz": 50, "gyro_noise": {"stddev": 0.005}},
                {"type": "lidar", "rate_hz": 5, "rays": 120, "max_range": 12},
            ],
        }
    )
    result = Simulator(config).run()
    assert result.status == "completed"
    ekf = EncoderImuEKF(
        0.05, 0.3, config.robot.initial_pose.to_pose(), np.eye(3) * 1e-5, gyro_stddev=0.005
    )
    imu = {r.capture_time: r for r in result.readings if isinstance(r, ImuReading)}
    estimates = {}
    for reading in result.readings:
        if isinstance(reading, EncoderReading):
            estimates[reading.capture_time] = ekf.update(reading, imu[reading.capture_time])
    grid = OccupancyGrid(
        GridConfig(columns=170, rows=170, resolution=0.05, origin_x=-0.25, origin_y=-0.25)
    )
    updates = []
    for scan in result.readings:
        if isinstance(scan, LidarReading):
            estimate = estimates[scan.capture_time]
            updates.append(
                grid.update(scan, estimate.pose, pose_time=estimate.time, mount=Pose2(0.1, 0))
            )
    states = grid.states()
    summary = {
        "scans": len(updates),
        "free_cells": int((states == 0).sum()),
        "occupied_cells": int((states == 100).sum()),
        "unknown_cells": int((states == -1).sum()),
    }
    assert summary["occupied_cells"] > 200 and summary["free_cells"] > 1000
    return result, grid, list(estimates.values()), updates, summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_10")
    args = parser.parse_args()
    result, grid, estimates, updates, summary = run()
    repeated = run()
    np.testing.assert_array_equal(grid.log_odds, repeated[1].log_odds)
    assert summary == repeated[-1]
    save_result(result, args.output)
    grid.save(args.output / "occupancy.npz")
    (args.output / "updates.json").write_text(
        json.dumps([asdict(u) for u in updates], indent=2), encoding="utf-8"
    )
    (args.output / "estimates.json").write_text(
        json.dumps([asdict(e) for e in estimates], indent=2), encoding="utf-8"
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Circle, Rectangle

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    axes[0].add_patch(Rectangle((0, 0), 8, 8, fill=False, linewidth=2))
    axes[0].add_patch(Rectangle((6, 2), 1, 3, color="#293542"))
    axes[0].add_patch(Circle((1.5, 5), 0.6, color="#293542"))
    axes[0].set_title("Known world · evaluation only")
    display = np.ones(grid.states().shape)
    display[grid.states() == 0] = 2
    display[grid.states() == 100] = 0
    axes[1].imshow(
        display,
        origin="lower",
        extent=(-0.25, 8.25, -0.25, 8.25),
        cmap=ListedColormap(["#293542", "#a5afba", "#f5f7fa"]),
        vmin=0,
        vmax=2,
    )
    axes[1].set_title("Estimated map · dark occupied / gray unknown")
    for ax in axes:
        ax.plot(
            [e.pose.x for e in estimates],
            [e.pose.y for e in estimates],
            color="#00a3b3",
            label="EKF path",
        )
        ax.set(xlim=(-0.25, 8.25), ylim=(-0.25, 8.25), xlabel="x [m]", ylabel="y [m]")
        ax.set_aspect("equal")
        ax.legend()
    fig.tight_layout()
    fig.savefig(args.output / "mapping.png", dpi=150)
    plt.close(fig)
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
