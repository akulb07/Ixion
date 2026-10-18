"""Regulate forward wheel speed from encoder counts and retain sonar braking."""

import argparse
import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.encoder_faults import EncoderCountFault
from roboforge.hardware.encoder_health import EncoderHealthConfig
from roboforge.hardware.encoder_speed import EncoderSpeedConfig
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.speed_firmware import EncoderSpeedFirmware


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-speed", type=float, default=0.2, help="forward target in m/s")
    parser.add_argument("--ramp-seconds", type=float, default=1, help="time to ramp the target")
    parser.add_argument("--seconds", type=float, default=4, help="maximum run duration")
    parser.add_argument(
        "--encoder-fault",
        choices=("none", "left-freeze", "both-freeze", "left-jump"),
        default="none",
    )
    parser.add_argument(
        "--fault-at", type=float, default=0.8, help="counter fault activation time in seconds"
    )
    parser.add_argument(
        "--no-encoder-checks",
        action="store_true",
        help="compare the virtual run without feedback checks",
    )
    parser.add_argument("--output", type=Path, help="save the complete report to a new file")
    args = parser.parse_args()
    root = Path(__file__).parent
    try:
        project = load_project(root / "robot.yaml")
        source = load_rover_scenario(root / "rover_wall_debug.yaml")
        faults = ()
        if args.encoder_fault == "left-jump":
            faults = (
                EncoderCountFault(
                    channel="left", start_time_s=args.fault_at, kind="offset", offset_counts=10_000
                ),
            )
        elif args.encoder_fault in {"left-freeze", "both-freeze"}:
            channels = ("left", "right") if args.encoder_fault == "both-freeze" else ("left",)
            faults = tuple(
                EncoderCountFault(channel=channel, start_time_s=args.fault_at)
                for channel in channels
            )
        scenario = type(source).model_validate(
            {
                **source.model_dump(),
                "control_every_steps": 20,
                "duration_s": args.seconds,
                "encoder_faults": faults,
            }
        )
        controller = EncoderSpeedFirmware(
            encoder=EncoderSpeedConfig.from_project(
                project, scenario.left_motor, scenario.right_motor
            ),
            encoder_health=None if args.no_encoder_checks else EncoderHealthConfig(),
            target_speed_m_s=args.target_speed,
            target_ramp_s=args.ramp_seconds,
            brake_threshold_m=scenario.brake_threshold_m,
            timeout_action=scenario.timeout_action,
        )
        report = run_rover_brake(project, scenario, firmware=controller)
        report["speed_control"] = controller.report()
        if args.output is not None:
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(report, stream, indent=2, allow_nan=False)
                stream.write("\n")
    except (ValueError, OSError) as exc:
        parser.exit(2, f"{exc}\n")
    print(
        json.dumps(
            {
                "status": report["status"],
                "brake_reason": report["brake_reason"],
                "metrics": report["metrics"],
                "speed_control_metrics": report["speed_control"]["metrics"],
                "encoder_health": report["speed_control"]["encoder_health"],
            },
            indent=2,
            allow_nan=False,
        )
    )
    return 0 if report["status"] == "stopped_before_wall" else 3


if __name__ == "__main__":
    raise SystemExit(main())
