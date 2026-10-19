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
    motor = subparsers.add_parser("motor-point", help="Estimate a DC gearmotor operating point")
    motor.add_argument("project", type=Path)
    motor.add_argument("--component", required=True)
    motor.add_argument("--voltage", type=float, required=True)
    motor.add_argument("--rpm", type=float, required=True, help="Signed gearbox output RPM")
    motor.add_argument(
        "--load-torque", type=float, default=0, help="Signed opposing output torque in N m"
    )
    battery = subparsers.add_parser(
        "battery-step", help="Estimate constant-current battery discharge"
    )
    battery.add_argument("project", type=Path)
    battery.add_argument("--component", required=True)
    battery.add_argument("--current", type=float, required=True)
    battery.add_argument("--seconds", type=float, required=True)
    battery.add_argument("--soc", type=float, default=1)
    feedback = subparsers.add_parser(
        "encoder-feedback", help="Run a project-wired shaft braking experiment"
    )
    feedback.add_argument("project", type=Path)
    feedback.add_argument("--motor", default="left_motor")
    feedback.add_argument("--battery", default="battery")
    feedback.add_argument("--mcu", default="esp32")
    feedback.add_argument("--inertia", type=float, required=True)
    feedback.add_argument("--target-count", type=int, required=True)
    feedback.add_argument("--seconds", type=float, default=2)
    feedback.add_argument("--step", type=float, default=0.005)
    feedback.add_argument(
        "--control-every", type=int, default=1, help="Run controller every N physics steps"
    )
    feedback.add_argument("--output", type=Path, required=True)
    ultrasonic = subparsers.add_parser(
        "ultrasonic-feedback", help="Run a scripted ultrasonic shaft experiment"
    )
    ultrasonic.add_argument("project", type=Path)
    ultrasonic.add_argument("scenario", type=Path)
    ultrasonic.add_argument("--output", type=Path, required=True)
    rover = subparsers.add_parser(
        "rover-brake", help="Estimate a moving rover's wall approach and braking"
    )
    rover.add_argument("project", type=Path)
    rover.add_argument("scenario", type=Path)
    rover.add_argument("--output", type=Path, required=True)
    rover_size = subparsers.add_parser(
        "rover-size",
        help="Screen a rover speed/acceleration demand against torque, supply and traction",
    )
    rover_size.add_argument("project", type=Path)
    rover_size.add_argument("demand", type=Path)
    rover_size.add_argument("--output", type=Path, required=True)
    rover_suite = subparsers.add_parser(
        "rover-suite", help="Run repeatable rover cases against declared acceptance checks"
    )
    rover_suite.add_argument("project", type=Path)
    rover_suite.add_argument("suite", type=Path)
    rover_suite.add_argument("--output", type=Path, required=True)
    rover_suite.add_argument("--junit", type=Path, help="Also write JUnit XML for CI")
    rover_report = subparsers.add_parser(
        "rover-report", help="Create an offline timeline viewer from saved rover JSON"
    )
    rover_report.add_argument("report", type=Path)
    rover_report.add_argument("--output", type=Path, required=True)
    rover_compare = subparsers.add_parser(
        "rover-compare", help="Compare saved hardware suites under identical declared conditions"
    )
    rover_compare.add_argument("baseline", type=Path)
    rover_compare.add_argument("candidate", type=Path)
    rover_compare.add_argument("--output", type=Path, required=True)
    build_reference = subparsers.add_parser(
        "build-reference", help="Export parts, pins and bench references from a hardware project"
    )
    build_reference.add_argument("project", type=Path)
    build_reference.add_argument("--suite", type=Path)
    build_reference.add_argument("--case")
    build_reference.add_argument("--output", type=Path, required=True)
    verify_reference = subparsers.add_parser(
        "verify-build-reference", help="Verify a portable hardware build-reference packet"
    )
    verify_reference.add_argument("archive", type=Path)
    rover_bench = subparsers.add_parser(
        "rover-bench", help="Compare a declared bench CSV trace with saved rover evidence"
    )
    rover_bench.add_argument("report", type=Path)
    rover_bench.add_argument("trace", type=Path)
    rover_bench.add_argument("metadata", type=Path)
    rover_bench.add_argument("--case")
    rover_bench.add_argument("--time-offset-s", type=float, required=True)
    rover_bench.add_argument("--max-sample-age-s", type=float, required=True)
    rover_bench.add_argument("--output", type=Path, required=True)
    bench_check = subparsers.add_parser(
        "rover-bench-check",
        help="Check saved bench evidence against explicit error and coverage limits",
    )
    bench_check.add_argument("comparison", type=Path)
    bench_check.add_argument("policy", type=Path)
    bench_check.add_argument("--output", type=Path, required=True)
    bench_check.add_argument("--junit", type=Path)
    starter = subparsers.add_parser(
        "new-project", help="Create the bundled rover starter with assumed part values"
    )
    starter.add_argument("destination", type=Path)
    starter.add_argument("--template", choices=["esp32-rover"], default="esp32-rover")
    starter.add_argument(
        "--demo",
        action="store_true",
        help="Also run the reference suite and save JSON, JUnit and HTML",
    )
    bench_bundle = subparsers.add_parser(
        "rover-bench-bundle",
        help="Package original bench inputs and declared acceptance in a portable ZIP",
    )
    bench_bundle.add_argument("report", type=Path)
    bench_bundle.add_argument("trace", type=Path)
    bench_bundle.add_argument("metadata", type=Path)
    bench_bundle.add_argument("policy", type=Path)
    bench_bundle.add_argument("--case")
    bench_bundle.add_argument("--time-offset-s", type=float, required=True)
    bench_bundle.add_argument("--max-sample-age-s", type=float, required=True)
    bench_bundle.add_argument("--output", type=Path, required=True)
    verify_bench = subparsers.add_parser(
        "verify-bench-bundle",
        help="Verify and recompute a portable bench check without extracting files",
    )
    verify_bench.add_argument("archive", type=Path)
    verify_bench.add_argument(
        "--output", type=Path, help="Save the recomputed full bench acceptance JSON"
    )
    verify_bench.add_argument("--junit", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "rover-bench-bundle":
            from roboforge.ci_reports import validate_output_paths
            from roboforge.hardware.bench_bundle import write_bench_bundle

            validate_output_paths(
                [args.output], protected=[args.report, args.trace, args.metadata, args.policy]
            )
            result = write_bench_bundle(
                args.output,
                args.report,
                args.trace,
                args.metadata,
                args.policy,
                time_offset_s=args.time_offset_s,
                maximum_sample_age_s=args.max_sample_age_s,
                case_name=args.case,
            )
            print(
                f"Saved original bench inputs: {args.output}; declared {result['source_kind']} acceptance: {result['status']}"
            )
            return {"pass": 0, "fail": 3, "inconclusive": 4}[result["status"]]
        if args.command == "verify-bench-bundle":
            import json

            from roboforge.ci_reports import bench_acceptance_junit, validate_output_paths
            from roboforge.hardware.bench_bundle import verify_bench_bundle

            validate_output_paths([args.output, args.junit], protected=[args.archive])
            result = verify_bench_bundle(args.archive)
            payload = json.dumps(result, indent=2, allow_nan=False)
            junit = None if args.junit is None else bench_acceptance_junit(result)
            if args.output is not None:
                with args.output.open("x", encoding="utf-8") as stream:
                    stream.write(payload + "\n")
            if args.junit is not None:
                with args.junit.open("x", encoding="utf-8") as stream:
                    stream.write(junit)
            print(
                f"Verified archive consistency; declared {result['source_kind']} acceptance: {result['status']}"
            )
            return {"pass": 0, "fail": 3, "inconclusive": 4}[result["status"]]
        if args.command == "new-project":
            from roboforge.hardware.starter import create_starter

            manifest = create_starter(args.destination, template=args.template, run_demo=args.demo)
            print(
                f"Created {manifest['template']} in {args.destination.resolve()}; {manifest['component_instances']} components; engineering readiness is unassessed"
            )
            print(
                f'Inspect the assumed parts: ixion inspect-project "{args.destination / "robot.yaml"}"'
            )
            if manifest["demo"] is not None:
                print(
                    f"Demo {manifest['demo']['status']}: {manifest['demo']['passed']}/{manifest['demo']['total']} cases passed; open {args.destination / 'results/rover-suite.html'}"
                )
                return 0 if manifest["demo"]["status"] == "passed" else 3
            return 0
        if args.command == "rover-bench-check":
            import json

            from roboforge.ci_reports import bench_acceptance_junit, validate_output_paths
            from roboforge.hardware.bench_acceptance import (
                check_bench_comparison,
                load_bench_policy,
            )
            from roboforge.hardware.rover_report import load_rover_evidence

            validate_output_paths(
                [args.output, args.junit], protected=[args.comparison, args.policy]
            )
            comparison, comparison_digest = load_rover_evidence(args.comparison)
            policy, policy_digest = load_bench_policy(args.policy)
            result = check_bench_comparison(
                comparison,
                policy,
                comparison_sha256=comparison_digest,
                policy_source_sha256=policy_digest,
            )
            payload = json.dumps(result, indent=2, allow_nan=False)
            junit = None if args.junit is None else bench_acceptance_junit(result)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
            if args.junit is not None:
                with args.junit.open("x", encoding="utf-8") as stream:
                    stream.write(junit)
            print(
                f"{result['status']}: {result['source_kind']} bench evidence against {policy.name}; saved to {args.output}"
            )
            return {"pass": 0, "fail": 3, "inconclusive": 4}[result["status"]]
        if args.command == "rover-bench":
            import json

            from roboforge.ci_reports import validate_output_paths
            from roboforge.hardware.bench_trace import compare_bench_trace, load_bench_trace
            from roboforge.hardware.rover_report import load_rover_evidence

            validate_output_paths([args.output], protected=[args.report, args.trace, args.metadata])
            report, digest = load_rover_evidence(args.report)
            trace = load_bench_trace(args.trace, args.metadata)
            result = compare_bench_trace(
                report,
                trace,
                time_offset_s=args.time_offset_s,
                maximum_sample_age_s=args.max_sample_age_s,
                case_name=args.case,
                report_sha256=digest,
            )
            payload = json.dumps(result, indent=2, allow_nan=False)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
            print(
                f"{result['status']}: {result['source_kind']} trace; {result['counts']['paired_rows']}/{result['counts']['total']} rows paired; saved to {args.output}"
            )
            return 0 if result["status"] == "compared" else 3
        if args.command == "verify-build-reference":
            from roboforge.hardware.build_reference import verify_build_reference

            result = verify_build_reference(args.archive)
            print(
                f"Verified build reference for {result['project_name']}; {result['component_instances']} component instances; engineering readiness is unassessed"
            )
            return 0
        if args.command == "build-reference":
            from roboforge.ci_reports import validate_output_paths
            from roboforge.hardware import load_project
            from roboforge.hardware.build_reference import write_build_reference
            from roboforge.hardware.rover_suite import load_rover_suite

            validate_output_paths(
                [args.output],
                protected=[args.project, *([] if args.suite is None else [args.suite])],
            )
            suite = None if args.suite is None else load_rover_suite(args.suite)
            result = write_build_reference(
                args.output, load_project(args.project), suite, args.case
            )
            print(
                f"Exported {result['component_instances']} component instances; {result['design_errors']} design errors; {len(result['incomplete_component_metadata'])} parts have incomplete metadata; saved to {args.output}"
            )
            return 3 if result["design_errors"] else 0
        if args.command == "rover-compare":
            import json

            from roboforge.ci_reports import validate_output_paths
            from roboforge.hardware.rover_comparison import compare_rover_suites
            from roboforge.hardware.rover_report import load_rover_evidence

            validate_output_paths([args.output], protected=[args.baseline, args.candidate])
            baseline, base_digest = load_rover_evidence(args.baseline)
            candidate, next_digest = load_rover_evidence(args.candidate)
            result = compare_rover_suites(
                baseline, candidate, baseline_sha256=base_digest, candidate_sha256=next_digest
            )
            payload = json.dumps(result, indent=2, allow_nan=False)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
            print(
                f"{result['status']}: candidate acceptance; {result['design_changes']['total']} design changes; saved to {args.output}"
            )
            return 0 if result["status"] == "passed" else 3
        if args.command == "rover-report":
            from roboforge.hardware.rover_report import load_rover_evidence, render_rover_evidence

            if args.output.exists():
                raise ValueError("rover report output already exists; choose a new path")
            report, digest = load_rover_evidence(args.report)
            html = render_rover_evidence(report, digest)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(html)
            print(f"Saved offline rover viewer to {args.output}")
            return 0
        if args.command == "rover-suite":
            import json

            from roboforge.ci_reports import rover_suite_junit, validate_output_paths
            from roboforge.hardware import load_project
            from roboforge.hardware.rover_suite import load_rover_suite, run_rover_suite

            validate_output_paths([args.output, args.junit], protected=[args.project, args.suite])
            result = run_rover_suite(load_project(args.project), load_rover_suite(args.suite))
            payload = json.dumps(result, indent=2, allow_nan=False)
            junit = None if args.junit is None else rover_suite_junit(result)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
            if args.junit is not None:
                with args.junit.open("x", encoding="utf-8") as stream:
                    stream.write(junit)
            summary = result["summary"]
            print(
                f"{result['status']}: {summary['passed']}/{summary['total']} cases passed; "
                f"{summary['failed']} failed, {summary['error']} errors; saved to {args.output}"
            )
            return 0 if result["status"] == "passed" else 3
        if args.command == "rover-size":
            import json

            from roboforge.ci_reports import validate_output_paths
            from roboforge.hardware import load_project
            from roboforge.hardware.rover_sizing import load_rover_sizing, size_rover

            validate_output_paths([args.output], protected=[args.project, args.demand])
            result = size_rover(load_project(args.project), load_rover_sizing(args.demand))
            payload = json.dumps(result, indent=2, allow_nan=False)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
            failed = [c["code"] for c in result["result"]["checks"] if c["status"] == "failed"]
            unassessed = sum(c["status"] == "not_assessed" for c in result["result"]["checks"])
            sizing = result["result"]
            duty = sizing["required_pwm_duty"]
            point = sizing["point"]
            print(
                ("Required PWM unavailable" if duty is None else f"Required PWM={duty:.1%}")
                + (
                    "; electrical point unavailable"
                    if point is None
                    else f"; pack={point['battery']['terminal_voltage_v']:.4g} V, {point['battery']['current_a']:.4g} A"
                )
                + f"; traction margin={sizing['traction_margin_per_wheel_n']:.4g} N/wheel"
            )
            print(
                f"{result['status']}: {', '.join(failed) if failed else 'no known bound exceeded'}; "
                f"{unassessed} checks unassessed; engineering readiness not assessed; saved to {args.output}"
            )
            return 0 if result["status"] == "checks_passed" else 3
        if args.command == "rover-brake":
            import json

            from roboforge.hardware import load_project
            from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake

            if args.output.exists():
                raise ValueError("rover output already exists; choose a new path")
            result = run_rover_brake(load_project(args.project), load_rover_scenario(args.scenario))
            payload = json.dumps(result, indent=2, allow_nan=False)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
            print(
                f"{result['status']}: clearance={result['metrics']['final_front_clearance_m']:.4g} m; "
                f"saved to {args.output}"
            )
            return 0 if result["status"] == "stopped_before_wall" else 3
        if args.command == "ultrasonic-feedback":
            import json

            from roboforge.hardware import load_project
            from roboforge.hardware.ultrasonic_feedback import (
                load_ultrasonic_scenario,
                run_ultrasonic_feedback,
            )

            if args.output.exists():
                raise ValueError("feedback output already exists; choose a new path")
            result = run_ultrasonic_feedback(
                load_project(args.project), load_ultrasonic_scenario(args.scenario)
            )
            payload = json.dumps(result, indent=2, allow_nan=False)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
            print(f"{result['status']}: reason={result['brake_reason']}; saved to {args.output}")
            return 0 if result["status"] == "brake_triggered" else 3
        if args.command == "encoder-feedback":
            import json

            from roboforge.hardware import load_project
            from roboforge.hardware.feedback import FeedbackConfig, run_encoder_feedback

            if args.output.exists():
                raise ValueError("feedback output already exists; choose a new path")
            result = run_encoder_feedback(
                load_project(args.project),
                FeedbackConfig(
                    motor=args.motor,
                    battery=args.battery,
                    mcu=args.mcu,
                    inertia_kg_m2=args.inertia,
                    target_count=args.target_count,
                    duration_s=args.seconds,
                    step_s=args.step,
                    control_every_steps=args.control_every,
                ),
            )
            payload = json.dumps(result, indent=2, allow_nan=False)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload + "\n")
            print(
                f"{result['status']}: {result['samples'][-1]['encoder_count']} counts; saved to {args.output}"
            )
            return 0 if result["status"] == "brake_triggered" else 3
        if args.command == "battery-step":
            import json

            from roboforge.hardware import load_project
            from roboforge.hardware.battery import BatteryModel, BatteryState

            project = load_project(args.project)
            component = next((c for c in project.components if c.id == args.component), None)
            if component is None:
                raise ValueError("battery component not found in project")
            model = BatteryModel.from_component(component)
            result = model.discharge(BatteryState(soc=args.soc), args.current, args.seconds)
            print(
                json.dumps(
                    {
                        "kind": "battery_discharge_step",
                        "result_type": "estimate",
                        "component": component.id,
                        "model": model.model_dump(mode="json"),
                        "inputs": [p.model_dump(mode="json") for p in component.parameters],
                        "result": result.model_dump(mode="json"),
                        "limitations": [
                            "Two-point linear OCV approximation using explicit pack inputs.",
                            "Charge-limited runtime excludes cutoff voltage, temperature, aging and changing load.",
                            "Current ratings produce warnings, not protection or burst-duration enforcement.",
                            "No charging, BMS, motor/driver coupling or cell imbalance model.",
                        ],
                    },
                    indent=2,
                    allow_nan=False,
                )
            )
            return 0
        if args.command == "motor-point":
            import json
            import math

            from roboforge.hardware import load_project
            from roboforge.hardware.motor import MotorDatasheet

            project = load_project(args.project)
            component = next((c for c in project.components if c.id == args.component), None)
            if component is None:
                raise ValueError("motor component not found in project")
            model = MotorDatasheet.from_component(component).derive()
            point = model.operating_point(args.voltage, args.rpm * math.tau / 60, args.load_torque)
            print(
                json.dumps(
                    {
                        "kind": "motor_operating_point",
                        "result_type": "estimate",
                        "component": component.id,
                        "model": model.model_dump(mode="json"),
                        "inputs": [p.model_dump(mode="json") for p in component.parameters],
                        "point": point.model_dump(mode="json"),
                        "limitations": [
                            "Prescribed speed, not a dynamic robot simulation.",
                            "No inductance, thermal model, current limiting or battery/driver coupling.",
                            "Zero volts means shorted terminals, not coasting.",
                            "Signed regenerative power does not establish that a driver or battery can accept it.",
                        ],
                    },
                    indent=2,
                    allow_nan=False,
                )
            )
            return 0
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
