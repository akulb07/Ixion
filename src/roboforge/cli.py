"""Foundation CLI: configuration validation and prescribed-motion simulation."""

import argparse
import sys
from pathlib import Path

from roboforge.config import load_config
from roboforge.io import save_result
from roboforge.physics import CollisionWorld
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
    experiment = subparsers.add_parser("experiment", help="Run a bounded parameter and seed sweep")
    experiment.add_argument("config", type=Path)
    experiment.add_argument("--output", type=Path, required=True)
    experiment.add_argument("--plot", action="store_true")
    replay = subparsers.add_parser(
        "replay", help="Inspect a recorded time without rerunning simulation"
    )
    replay.add_argument("directory", type=Path)
    replay.add_argument("--time", type=float, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "experiment":
            import json

            from roboforge.experiments import load_experiment, run_experiment

            directory = run_experiment(load_experiment(args.config), args.output, plot=args.plot)
            report = json.loads((directory / "report.json").read_text(encoding="utf-8"))
            print(f"Saved {report['finished_trials']} trials to {directory.resolve()}")
            return 0 if all(r["status"] == "completed" for r in report["trials"]) else 3
        if args.command == "replay":
            import json
            from dataclasses import asdict

            from roboforge.replay import ReplayLog

            frame = ReplayLog(args.directory).at(args.time)
            print(
                json.dumps(
                    {
                        "state": asdict(frame.state),
                        "delivered_readings": [r.model_dump() for r in frame.readings],
                        "control": frame.control,
                        "actuator": frame.actuator,
                    },
                    allow_nan=False,
                )
            )
            return 0
        config = load_config(args.config)
        if args.command == "validate":
            if config.simulation.collision.mode == "stop":
                report = CollisionWorld(config.environment).query(
                    config.robot.initial_pose.to_pose().position, config.robot.footprint_radius
                )
                if report.collision:
                    raise ValueError(
                        f"initial footprint contacts {report.nearest.object_id}; "
                        f"clearance={report.clearance:g} m"
                    )
            print(f"Valid: {config.name}")
            return 0
        result = Simulator(config).run()
        save_result(result, args.output)
        if args.plot:
            from roboforge.visualization import plot_trajectory

            plot_trajectory(result, args.output / "trajectory.png")
        final = result.states[-1]
        print(
            f"{result.status}: {len(result.states) - 1} state updates, t={final.time:g} s; "
            f"pose=({final.pose.x:.9f}, {final.pose.y:.9f}, {final.pose.theta:.9f})"
        )
        if result.collisions:
            event = result.collisions[0]
            print(
                f"Stopped: {event.reason} with {event.report.nearest.object_id}; "
                f"candidate clearance={event.report.clearance:.9g} m"
            )
        print(
            f"Saved to {args.output.resolve()}; collision mode={config.simulation.collision.mode}"
        )
        return 3 if result.status == "collision" else 0
    except (ValueError, OSError, OverflowError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
