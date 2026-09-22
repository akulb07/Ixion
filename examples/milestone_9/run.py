"""Encoder/gyro EKF versus odometry under wheelbase calibration error."""

import argparse
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np

from roboforge.config import RunConfig
from roboforge.io import save_result
from roboforge.localization import EncoderImuEKF
from roboforge.odometry import EncoderOdometry
from roboforge.sensors.readings import EncoderReading, ImuReading
from roboforge.simulation import Simulator


def run():
    config = RunConfig.model_validate(
        {
            "name": "encoder_gyro_ekf",
            "seed": 42,
            "robot": {"initial_pose": {"x": 2, "y": 2}},
            "commands": [{"left": 5, "right": 8, "steps": 1200}],
            "sensors": [
                {"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 65536},
                {"type": "imu", "rate_hz": 50, "gyro_noise": {"stddev": 0.01}},
            ],
        }
    )
    result = Simulator(config).run()
    prior = config.robot.initial_pose.to_pose()
    ekf = EncoderImuEKF(0.05, 0.33, prior, np.diag([0.001, 0.001, 0.0001]), gyro_stddev=0.01)
    odometry = EncoderOdometry(0.05, 0.33, prior)
    imu = {r.capture_time: r for r in result.readings if isinstance(r, ImuReading)}
    fused, dead = [], []
    for reading in result.readings:
        if isinstance(reading, EncoderReading):
            fused.append(ekf.update(reading, imu[reading.capture_time]))
            dead.append(odometry.update(reading))
    truth = result.states[-1].pose

    def error(pose):
        return math.hypot(pose.x - truth.x, pose.y - truth.y)

    summary = {
        "odometry_endpoint_error_m": error(dead[-1].pose),
        "ekf_endpoint_error_m": error(fused[-1].pose),
        "gyro_updates": len(ekf.innovations),
        "rejected_gyro_updates": sum(not i.accepted for i in ekf.innovations),
    }
    assert summary["ekf_endpoint_error_m"] < 0.05
    assert summary["ekf_endpoint_error_m"] < summary["odometry_endpoint_error_m"] * 0.1
    return result, fused, dead, ekf.innovations, summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_9")
    args = parser.parse_args()
    result, fused, dead, innovations, summary = run()
    assert (result, fused, dead, innovations, summary) == run()
    save_result(result, args.output)
    for name, values in (("ekf", fused), ("odometry", dead), ("innovations", innovations)):
        (args.output / f"{name}.json").write_text(
            json.dumps([asdict(v) for v in values], indent=2), encoding="utf-8"
        )
    (args.output / "estimator.json").write_text(
        json.dumps(
            {
                "wheel_radius": 0.05,
                "assumed_wheel_separation": 0.33,
                "wheel_variance_per_m": 0.001,
                "gyro_stddev": 0.01,
                "prior_covariance": [[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.0001]],
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(
        [s.pose.x for s in result.states],
        [s.pose.y for s in result.states],
        color="black",
        label="Truth",
    )
    for series, name in ((dead, "Encoder odometry"), (fused, "Encoder + gyro EKF")):
        ax.plot([s.pose.x for s in series], [s.pose.y for s in series], label=name)
    for estimate in fused[::100]:
        eigenvalues, eigenvectors = np.linalg.eigh(np.array(estimate.covariance)[:2, :2])
        radii = np.sqrt(5.991 * np.maximum(eigenvalues, 0))
        angle = math.degrees(math.atan2(eigenvectors[1, 1], eigenvectors[0, 1]))
        ax.add_patch(
            Ellipse(
                (estimate.pose.x, estimate.pose.y),
                2 * radii[1],
                2 * radii[0],
                angle=angle,
                alpha=0.15,
                color="green",
            )
        )
    ax.set(
        xlabel="World x [m]",
        ylabel="World y [m]",
        title="EKF · wheelbase calibration error · 95% position ellipses",
    )
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output / "localization.png", dpi=150)
    plt.close(fig)
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
