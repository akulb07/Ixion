"""Run the configured foundation demo, validate endpoints, and save a plot."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np

from roboforge.config import load_config
from roboforge.core import wrap_angle
from roboforge.io import save_result
from roboforge.simulation import Simulator
from roboforge.visualization import plot_trajectory


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "foundation")
    args = parser.parse_args()
    config = load_config(ROOT / "configs" / "foundation.yaml")
    result = Simulator(config).run()
    # Independent analytical references for this fixed demonstration config.
    expected_straight = np.array([1.5, 1.0, 0.0])
    expected_spin = np.array([1.5, 1.0, wrap_angle(10.0 / 3.0)])
    actual_straight = result.states[1000].pose.as_array()
    actual_spin = result.states[-1].pose.as_array()
    np.testing.assert_allclose(actual_straight, expected_straight, atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual_spin, expected_spin, atol=1e-12, rtol=0)
    assert result == Simulator(config).run(), "Repeated simulation must produce identical states"
    save_result(result, args.output)
    plot_trajectory(result, args.output / "trajectory.png")
    validation = {
        "passed": True,
        "absolute_tolerance": 1e-12,
        "steps": len(result.states) - 1,
        "straight": {"expected": expected_straight.tolist(), "actual": actual_straight.tolist()},
        "spin": {"expected": expected_spin.tolist(), "actual": actual_spin.tolist()},
        "max_absolute_component_error": float(
            max(
                np.max(np.abs(actual_straight - expected_straight)),
                np.max(np.abs(actual_spin - expected_spin)),
            )
        ),
        "deterministic_repeat": True,
    }
    (args.output / "validation.json").write_text(json.dumps(validation, indent=2), encoding="utf-8")
    print(json.dumps(validation, indent=2))
    print(f"Artifacts: {args.output.resolve()}")


if __name__ == "__main__":
    main()
