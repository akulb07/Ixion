"""Exercise trace import with deliberately biased synthetic data, not hardware measurements."""

import argparse
import csv
import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.bench_trace import compare_bench_trace, load_bench_trace
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, required=True, help="new directory for synthetic example files"
    )
    args = parser.parse_args()
    try:
        if args.output.exists():
            raise ValueError("synthetic example directory already exists; choose a new path")
        root = Path(__file__).parent
        report = run_rover_brake(
            load_project(root / "robot.yaml"), load_rover_scenario(root / "rover_wall.yaml")
        )
        args.output.mkdir(parents=True)
        with (args.output / "run.json").open("x", encoding="utf-8", newline="\n") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
        with (args.output / "trace.csv").open("x", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(["time_s", "speed_m_s", "pack_voltage_v", "pack_current_a"])
            for sample in report["samples"][::20]:
                writer.writerow(
                    [
                        sample["time_s"],
                        sample["speed_m_s"] + 0.01,
                        sample["pack_voltage_v"] - 0.05,
                        sample["pack_current_a"] + 0.02,
                    ]
                )
        metadata = {
            "source_kind": "synthetic",
            "reference": "Generated from the saved reference run with fixed illustrative biases",
            "instrument": "None; generated example",
            "conditions": "Speed +0.01 m/s, voltage -0.05 V, current +0.02 A; no physical measurements",
            "speed_reference": "wheel",
        }
        with (args.output / "metadata.json").open("x", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2)
        trace = load_bench_trace(args.output / "trace.csv", args.output / "metadata.json")
        comparison = compare_bench_trace(report, trace, time_offset_s=0, maximum_sample_age_s=0)
        with (args.output / "comparison.json").open("x", encoding="utf-8") as stream:
            json.dump(comparison, stream, indent=2, allow_nan=False)
    except (OSError, ValueError) as exc:
        parser.exit(2, f"{exc}\n")
    print(f"Synthetic example saved in {args.output}; no physical accuracy claim")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
