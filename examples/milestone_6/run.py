"""Compare measured encoder odometry against truth for evaluation only."""

import argparse
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.config import RunConfig
from roboforge.io import save_result
from roboforge.odometry import EncoderOdometry
from roboforge.sensors.readings import EncoderReading
from roboforge.simulation import Simulator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_6")
    args = parser.parse_args()
    config = RunConfig.model_validate(
        {
            "name": "odometry_calibration_drift",
            "robot": {"initial_pose": {"x": 2, "y": 2}},
            "commands": [{"left": 5, "right": 8, "steps": 800}],
            "sensors": [{"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 8192}],
        }
    )
    result = Simulator(config).run()
    save_result(result, args.output)
    estimators = {
        "Calibrated": EncoderOdometry(0.05, 0.3, config.robot.initial_pose.to_pose()),
        "Radius +5%": EncoderOdometry(0.0525, 0.3, config.robot.initial_pose.to_pose()),
    }
    series = {name: [] for name in estimators}
    for measurement in result.readings:
        if isinstance(measurement, EncoderReading):
            for name, estimator in estimators.items():
                estimate = estimator.update(measurement)
                if estimate is not None:
                    series[name].append(estimate)
    (args.output / "odometry.json").write_text(
        json.dumps(
            {name: [asdict(e) for e in estimates] for name, estimates in series.items()},
            indent=2,
            allow_nan=False,
        ),
        encoding="utf-8",
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(
        [s.pose.x for s in result.states],
        [s.pose.y for s in result.states],
        color="black",
        linewidth=3,
        label="Ground truth (evaluation)",
    )
    for name, estimates in series.items():
        ax.plot([e.pose.x for e in estimates], [e.pose.y for e in estimates], "--", label=name)
    ax.set(xlabel="World x [m]", ylabel="World y [m]", title="Encoder odometry · calibration drift")
    ax.set_aspect("equal")
    ax.legend()
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(args.output / "odometry.png", dpi=150)
    plt.close(fig)
    truth = result.states[-1].pose
    errors = {
        name: math.hypot(values[-1].pose.x - truth.x, values[-1].pose.y - truth.y)
        for name, values in series.items()
    }
    assert errors["Calibrated"] < 0.001
    assert errors["Radius +5%"] > 0.1
    (args.output / "validation.json").write_text(json.dumps(errors, indent=2), encoding="utf-8")
    print(json.dumps(errors))


if __name__ == "__main__":
    main()
