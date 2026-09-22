"""Paired-seed PID parameter sweep with persisted trials and replay."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.config import RunConfig
from roboforge.experiments import ExperimentConfig, SweepAxis, paired_differences, run_experiment
from roboforge.replay import ReplayLog


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_13")
    args = parser.parse_args()
    base = RunConfig.model_validate(
        {
            "name": "pid_sweep",
            "robot": {"initial_pose": {"x": 2, "y": 2}},
            "simulation": {"dt": 0.02},
            "commands": [{"left": 6, "right": 6, "steps": 250}],
            "sensors": [
                {
                    "type": "encoder",
                    "rate_hz": 50,
                    "ticks_per_revolution": 65536,
                    "noise": {"stddev": 0.0002},
                }
            ],
            "actuators": {
                "left": {"gain": 0.7, "time_constant": 0.15},
                "right": {"gain": 0.8, "time_constant": 0.2},
            },
            "wheel_controller": {"left": {"kp": 0.6}, "right": {"kp": 0.6}},
        }
    )
    spec = ExperimentConfig(
        name="paired_pid_integral_sweep",
        base=base,
        seeds=(11, 22, 33),
        axes=(
            SweepAxis(path="wheel_controller.left.ki", values=(0, 3)),
            SweepAxis(path="wheel_controller.right.ki", values=(0, 3)),
        ),
    )
    directory = run_experiment(spec, args.output)
    report = json.loads((directory / "report.json").read_text())
    comparison = paired_differences(report, "group-0000", "group-0003", "wheel_tracking_rmse_rad_s")
    assert report["finished_trials"] == 12 and all(
        r["status"] == "completed" for r in report["trials"]
    )
    assert comparison["mean_difference"] < 0
    replay = ReplayLog(directory / report["trials"][0]["trial_id"])
    assert replay.at(2.5).state.time == 2.5
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "comparison.json").write_text(json.dumps(comparison, indent=2), encoding="utf-8")
    (args.output / "latest.json").write_text(
        json.dumps({"directory": directory.name}, indent=2), encoding="utf-8"
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 5))
    for i, (group, summary) in enumerate(report["groups"].items()):
        values = [
            r["metrics"]["wheel_tracking_rmse_rad_s"]
            for r in report["trials"]
            if r["group"] == group
        ]
        stats = summary["metrics"]["wheel_tracking_rmse_rad_s"]
        ax.scatter([i] * len(values), values, color="#0099aa", alpha=0.7)
        ax.plot([i - 0.15, i + 0.15], [stats["mean"]] * 2, color="black")
    ax.set_xticks(range(4), ["Ki=(0,0)", "Ki=(0,3)", "Ki=(3,0)", "Ki=(3,3)"])
    ax.set(
        ylabel="Measured wheel tracking RMSE [rad/s]",
        title="Paired PID sweep · 3 seeds per setting · marks show means",
    )
    ax.grid(axis="y", alpha=0.2)
    fig.tight_layout()
    fig.savefig(args.output / "experiments.png", dpi=150)
    plt.close(fig)
    print(
        json.dumps(
            {
                "directory": str(directory),
                "trials": 12,
                "paired_mean_difference": comparison["mean_difference"],
            }
        )
    )


if __name__ == "__main__":
    main()
