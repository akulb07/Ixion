"""Compare ideal and limited asymmetric wheel response with actual telemetry."""

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
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_4")
    args = parser.parse_args()
    config = RunConfig.model_validate(
        {
            "name": "actuator_step_reversal",
            "robot": {"initial_pose": {"x": 2, "y": 2}},
            "commands": [{"left": w, "right": w, "steps": 150} for w in (12, -8, 0)],
            "actuators": {
                "delay_steps": 8,
                "left": {"max_speed": 9, "max_acceleration": 15, "time_constant": 0.15},
                "right": {
                    "max_speed": 9,
                    "max_acceleration": 12,
                    "time_constant": 0.2,
                    "gain": 0.9,
                },
            },
        }
    )
    result = Simulator(config).run()
    assert result == Simulator(config).run()
    save_result(result, args.output)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 5))
    samples = result.actuator_samples
    times = [s.time for s in samples]
    ax.step(
        times, [s.requested.left for s in samples], where="post", label="Requested", color="gray"
    )
    for side in ("left", "right"):
        ax.step(
            times, [getattr(s.applied, side) for s in samples], where="post", label=side.title()
        )
    ax.set(xlabel="Time [s]", ylabel="Wheel rate [rad/s]", title="RoboForge · actuator response")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output / "actuators.png", dpi=150)
    plt.close(fig)
    summary = {
        "deterministic_repeat": True,
        "samples": len(samples),
        "speed_limited_ticks": sum(any(s.speed_limited) for s in samples),
        "acceleration_limited_ticks": sum(any(s.acceleration_limited) for s in samples),
    }
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
