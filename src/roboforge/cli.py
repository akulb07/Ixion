"""Foundation CLI: configuration validation and prescribed-motion simulation."""

import argparse
import sys
from pathlib import Path

from roboforge.config import load_config
from roboforge.io import save_result
from roboforge.simulation import Simulator


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="roboforge")
    subparsers = parser.add_subparsers(dest="command", required=True)
    validate = subparsers.add_parser("validate", help="Validate a foundation run YAML/JSON")
    validate.add_argument("config", type=Path)
    simulate = subparsers.add_parser("simulate", help="Run ideal prescribed wheel motion")
    simulate.add_argument("config", type=Path)
    simulate.add_argument("--output", type=Path, required=True)
    simulate.add_argument("--plot", action="store_true")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.command == "validate":
            print(f"Valid: {config.name}")
            return 0
        result = Simulator(config).run()
        save_result(result, args.output)
        if args.plot:
            from roboforge.visualization import plot_trajectory

            plot_trajectory(result, args.output / "trajectory.png")
        final = result.states[-1]
        print(
            f"{len(result.states) - 1} steps, t={final.time:g} s; "
            f"pose=({final.pose.x:.9f}, {final.pose.y:.9f}, {final.pose.theta:.9f})"
        )
        print(f"Saved to {args.output.resolve()}; collision disabled")
        return 0
    except (ValueError, OSError, OverflowError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
