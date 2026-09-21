"""Run and export a deterministic noisy-sensor laboratory."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.config import load_config
from roboforge.io import save_result
from roboforge.sensors.visualization import plot_sensors
from roboforge.simulation import Simulator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_3")
    args = parser.parse_args()
    config = load_config(ROOT / "configs" / "sensors.yaml")
    result = Simulator(config).run()
    if result != Simulator(config).run():
        raise AssertionError("Seeded sensor measurements must repeat exactly")
    if result.status != "completed":
        raise AssertionError("Sensor demonstration must complete its motion")
    save_result(result, args.output)
    plot_sensors(result.readings, args.output / "sensors.png")
    summary = {
        "seed": config.seed,
        "readings": len(result.readings),
        "counts": {
            kind: sum(r.kind == kind for r in result.readings)
            for kind in ("encoder", "imu", "lidar")
        },
        "deterministic_repeat": True,
        "ground_truth_fields_in_readings": False,
    }
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
