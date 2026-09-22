"""Combined timed faults and a measured encoder-scale sensitivity sweep."""

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.config import RunConfig
from roboforge.experiments import ExperimentConfig, SweepAxis, run_experiment
from roboforge.io import save_result
from roboforge.odometry import EncoderOdometry
from roboforge.sensors.readings import EncoderReading
from roboforge.simulation import Simulator


def odometry(result):
    estimator = EncoderOdometry(
        result.config.robot.wheel_radius,
        result.config.robot.wheel_separation,
        result.config.robot.initial_pose.to_pose(),
    )
    return [
        estimate
        for r in result.readings
        if isinstance(r, EncoderReading)
        if (estimate := estimator.update(r)) is not None
    ]


def error(result):
    estimate, truth = odometry(result)[-1].pose, result.states[-1].pose
    return math.hypot(estimate.x - truth.x, estimate.y - truth.y)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_14")
    args = parser.parse_args()
    document = {
        "name": "timed_faults",
        "robot": {"initial_pose": {"x": 2, "y": 2}},
        "simulation": {"dt": 0.02},
        "commands": [{"left": 6, "right": 6, "steps": 300}],
        "sensors": [{"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 65536}],
        "faults": [
            {
                "name": "left_slip",
                "kind": "wheel_slip",
                "target": "left",
                "start": 2,
                "end": 4,
                "magnitude": 0.5,
            },
            {
                "name": "encoder_gap",
                "kind": "sensor_dropout",
                "target": "encoders",
                "start": 3,
                "end": 3.5,
                "magnitude": 1,
            },
        ],
    }
    config = RunConfig.model_validate(document)
    result = Simulator(config).run()
    assert result == Simulator(config).run()
    estimates = odometry(result)
    save_result(result, args.output / "combined")
    document["faults"] = [
        {"name": "scale", "kind": "encoder_scale", "target": "encoders", "magnitude": 0}
    ]
    spec = ExperimentConfig(
        name="encoder_scale_sensitivity",
        base=RunConfig.model_validate(document),
        axes=(SweepAxis(path="faults.0.magnitude", values=(0, 0.01, 0.02, 0.05, 0.1)),),
    )
    root = run_experiment(spec, args.output, metric_functions={"odometry_endpoint_error_m": error})
    report = json.loads((root / "report.json").read_text())
    errors = [r["metrics"]["odometry_endpoint_error_m"] for r in report["trials"]]
    assert all(b > a for a, b in zip(errors, errors[1:]))
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(
        [s.pose.x for s in result.states],
        [s.pose.y for s in result.states],
        label="Actual ground path",
    )
    axes[0].plot(
        [e.pose.x for e in estimates], [e.pose.y for e in estimates], "--", label="Encoder odometry"
    )
    axes[0].set(xlabel="x [m]", ylabel="y [m]", title="Left slip 2–4 s; encoder dropout 3–3.5 s")
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].legend()
    axes[1].plot([0, 0.01, 0.02, 0.05, 0.1], errors, "o-", color="#0099aa")
    axes[1].set(
        xlabel="Encoder readout scale error",
        ylabel="Final odometry error [m]",
        title="Measured fault sensitivity · seed 42",
    )
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(args.output / "faults.png", dpi=150)
    plt.close(fig)
    summary = {
        "combined_odometry_error_m": error(result),
        "scale_sweep_errors_m": errors,
        "fault_events": len(result.fault_events),
        "experiment_directory": root.name,
    }
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
