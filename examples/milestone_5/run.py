"""Sensor-only PID feedback corrects mismatched wheel gains."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.config import RunConfig
from roboforge.io import save_result
from roboforge.simulation import Simulator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_5")
    args = parser.parse_args()
    config = RunConfig.model_validate(
        {
            "name": "encoder_pid",
            "robot": {"initial_pose": {"x": 2, "y": 2}},
            "commands": [{"left": 6, "right": 6, "steps": 800}],
            "sensors": [{"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 65536}],
            "actuators": {
                "left": {"gain": 0.7, "time_constant": 0.15},
                "right": {"gain": 0.8, "time_constant": 0.2},
            },
            "wheel_controller": {"left": {"kp": 0.6, "ki": 3}, "right": {"kp": 0.6, "ki": 3}},
        }
    )
    closed = Simulator(config).run()
    opened = Simulator(config.model_copy(update={"wheel_controller": None})).run()
    assert closed == Simulator(config).run()
    save_result(closed, args.output / "closed")
    save_result(opened, args.output / "open")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    for result, name in ((opened, "Open loop"), (closed, "Encoder PID")):
        axes[0].plot(
            [s.time for s in result.states], [s.wheels.left for s in result.states], label=name
        )
        axes[1].plot(
            [s.pose.x for s in result.states], [s.pose.y for s in result.states], label=name
        )
    axes[0].axhline(6, color="gray", linestyle="--", label="Target")
    axes[0].set(xlabel="Time [s]", ylabel="Left wheel [rad/s]", title="Gain mismatch recovery")
    axes[1].set(xlabel="World x [m]", ylabel="World y [m]", title="Resulting ground-truth path")
    axes[1].set_aspect("equal", adjustable="datalim")
    for ax in axes:
        ax.grid(alpha=0.2)
        ax.legend()
    fig.tight_layout()
    fig.savefig(args.output / "control.png", dpi=150)
    plt.close(fig)
    summary = {
        "control_updates": len(closed.control_samples),
        "deterministic_repeat": True,
        "left_final_error_rad_s": abs(closed.states[-1].wheels.left - 6),
        "right_final_error_rad_s": abs(closed.states[-1].wheels.right - 6),
    }
    assert max(summary["left_final_error_rad_s"], summary["right_final_error_rad_s"]) < 0.01
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
