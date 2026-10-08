"""Foundation CLI: configuration validation and prescribed-motion simulation."""

import argparse
import sys
from pathlib import Path

from roboforge.config import load_config
from roboforge.io import save_result
from roboforge.physics import CollisionWorld
from roboforge.simulation import Simulator


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="ixion")
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
    benchmark = subparsers.add_parser("benchmark", help="Compare planners on versioned static maps")
    benchmark.add_argument("--output", type=Path, required=True)
    benchmark.add_argument("--seed", type=int, default=42)
    benchmark.add_argument("--iterations", type=int, default=500)
    serve = subparsers.add_parser("serve", help="Start the optional local API (install ixion[api])")
    serve.add_argument("--output", type=Path, default=Path("results/service"))
    serve.add_argument("--port", type=int, default=8765)
    check = subparsers.add_parser("check", help="Check saved runs against an acceptance policy")
    check.add_argument("policy", type=Path)
    check.add_argument("--store", type=Path, required=True)
    check.add_argument("--baseline", required=True)
    check.add_argument("--candidate", required=True)
    check.add_argument("--output", type=Path, required=True)
    check.add_argument("--junit", type=Path, help="Also write JUnit XML for CI")
    package = subparsers.add_parser(
        "bundle", help="Package two saved runs and their acceptance check"
    )
    package.add_argument("policy", type=Path)
    package.add_argument("--store", type=Path, required=True)
    package.add_argument("--baseline", required=True)
    package.add_argument("--candidate", required=True)
    package.add_argument("--output", type=Path, required=True)
    verify_bundle = subparsers.add_parser(
        "bundle-check", help="Verify a portable recorded-check bundle"
    )
    verify_bundle.add_argument("archive", type=Path)
    verify_bundle.add_argument("--junit", type=Path, help="Write JUnit XML for CI")
    inspect = subparsers.add_parser(
        "inspect-recording", help="Inventory an MCAP without decoding payloads"
    )
    inspect.add_argument("recording", type=Path)
    inspect.add_argument("--output", type=Path, required=True)
    inspect.add_argument("--max-messages", type=int, default=1_000_000)
    odometry = subparsers.add_parser(
        "extract-odometry", help="Decode a recorded ROS 2 odometry topic"
    )
    odometry.add_argument("recording", type=Path)
    odometry.add_argument("--topic", required=True)
    odometry.add_argument("--output", type=Path, required=True)
    odometry.add_argument("--max-samples", type=int, default=100_000)
    imu = subparsers.add_parser("extract-imu", help="Decode a recorded ROS 2 IMU topic")
    imu.add_argument("recording", type=Path)
    imu.add_argument("--topic", required=True)
    imu.add_argument("--output", type=Path, required=True)
    imu.add_argument("--max-samples", type=int, default=100_000)
    hardware = subparsers.add_parser(
        "inspect-project", help="Inspect hardware topology (not engineering readiness)"
    )
    hardware.add_argument("project", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "inspect-project":
            import json

            from roboforge.hardware import check_project, load_project

            project = load_project(args.project)
            diagnostics = check_project(project)
            print(
                json.dumps(
                    {
                        "kind": "hardware_topology_check",
                        "project": project.name,
                        "components": len(project.components),
                        "nets": len(project.nets),
                        "engineering_readiness": "not_assessed",
                        "diagnostics": [d.model_dump(mode="json") for d in diagnostics],
                    },
                    indent=2,
                    allow_nan=False,
                )
            )
            return 3 if any(d.severity == "ERROR" for d in diagnostics) else 0
        if args.command == "extract-imu":
            import json

            from roboforge.ci_reports import validate_output_paths
            from roboforge.recorded_imu import extract_imu

            validate_output_paths([args.output], protected=[args.recording])
            report = extract_imu(args.recording, args.topic, max_samples=args.max_samples)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(json.dumps(report, indent=2, allow_nan=False))
            print(
                f"Extracted {report['sample_count']} IMU samples; report: {args.output.resolve()}"
            )
            return 0
        if args.command == "extract-odometry":
            import json

            from roboforge.ci_reports import validate_output_paths
            from roboforge.recorded_odometry import extract_odometry

            validate_output_paths([args.output], protected=[args.recording])
            report = extract_odometry(args.recording, args.topic, max_samples=args.max_samples)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(json.dumps(report, indent=2, allow_nan=False))
            print(
                f"Extracted {report['sample_count']} odometry estimates; report: {args.output.resolve()}"
            )
            return 0
        if args.command == "inspect-recording":
            import json

            from roboforge.ci_reports import validate_output_paths
            from roboforge.recordings import inspect_mcap

            validate_output_paths([args.output], protected=[args.recording])
            report = inspect_mcap(args.recording, max_messages=args.max_messages)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(json.dumps(report, indent=2, allow_nan=False))
            print(
                f"Inspected {report['messages']} messages on {len(report['channels'])} channels; report: {args.output.resolve()}"
            )
            return 0
        if args.command in {"bundle", "bundle-check"}:
            from roboforge.bundles import check_bundle, export_bundle
            from roboforge.regression import RegressionPolicy, RegressionRequest

            if args.command == "bundle":
                policy = RegressionPolicy.model_validate_json(
                    args.policy.read_text(encoding="utf-8")
                )
                result = export_bundle(
                    args.store,
                    RegressionRequest(
                        baseline_id=args.baseline, candidate_id=args.candidate, policy=policy
                    ),
                    args.output,
                )
                print(
                    f"Saved check bundle: {args.output.resolve()}; acceptance: {result['status']}"
                )
                return 0
            from roboforge.ci_reports import validate_output_paths, write_junit

            validate_output_paths([args.junit], protected=[args.archive])
            result = check_bundle(args.archive)
            if args.junit:
                write_junit(args.junit, result)
            print(f"Verified bundle; acceptance: {result['status']}")
            for item in result["checks"]:
                print(f"  {item['status']}: {item['name']} — {item['reason']}")
            return {"pass": 0, "fail": 3, "inconclusive": 4}[result["status"]]
        if args.command == "check":
            import json

            from roboforge.ci_reports import validate_output_paths, write_junit
            from roboforge.regression import RegressionPolicy, RegressionRequest, check_saved_runs

            validate_output_paths(
                [args.output, args.junit], protected=[args.policy], store=args.store
            )
            policy = RegressionPolicy.model_validate_json(args.policy.read_text(encoding="utf-8"))
            report = check_saved_runs(
                args.store,
                RegressionRequest(
                    baseline_id=args.baseline, candidate_id=args.candidate, policy=policy
                ),
            )
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(json.dumps(report, indent=2, allow_nan=False))
            if args.junit:
                write_junit(args.junit, report)
            print(
                f"{report['status']}: {len(report['checks'])} checks; report: {args.output.resolve()}"
            )
            return {"pass": 0, "fail": 3, "inconclusive": 4}[report["status"]]
        if args.command == "serve":
            if not 1 <= args.port <= 65535:
                raise ValueError("port must be between 1 and 65535")
            try:
                import uvicorn

                from roboforge.api import create_app
            except ImportError as exc:
                raise ValueError('Install API dependencies with: pip install "ixion[api]"') from exc
            uvicorn.run(create_app(args.output), host="127.0.0.1", port=args.port, workers=1)
            return 0
        if args.command == "benchmark":
            from roboforge.benchmarks import BenchmarkConfig, run_benchmarks

            directory = run_benchmarks(
                BenchmarkConfig(seeds=(args.seed,), iterations=args.iterations), args.output
            )
            print(f"Saved benchmark results to {directory.resolve()}")
            return 0
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
                        "navigation": frame.navigation,
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
        return 0 if result.status == "completed" else 3
    except (ValueError, OSError, OverflowError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
