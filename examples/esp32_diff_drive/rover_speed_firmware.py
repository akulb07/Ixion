"""Regulate forward wheel speed from encoder counts and retain sonar braking."""

import argparse
import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.encoder_speed import EncoderSpeedConfig
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.speed_firmware import EncoderSpeedFirmware


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-speed", type=float, default=0.2, help="forward target in m/s")
    parser.add_argument("--ramp-seconds", type=float, default=1, help="time to ramp the target")
    parser.add_argument("--seconds", type=float, default=4, help="maximum run duration")
    parser.add_argument("--output", type=Path, help="save the complete report to a new file")
    args = parser.parse_args()
    root = Path(__file__).parent
    try:
        project = load_project(root / "robot.yaml")
        source = load_rover_scenario(root / "rover_wall_debug.yaml")
        scenario = type(source).model_validate(
            {**source.model_dump(), "control_every_steps": 20, "duration_s": args.seconds}
        )
        controller = EncoderSpeedFirmware(
            encoder=EncoderSpeedConfig.from_project(
                project, scenario.left_motor, scenario.right_motor
            ),
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
                "metrics": report["metrics"],
                "speed_control_metrics": report["speed_control"]["metrics"],
            },
            indent=2,
            allow_nan=False,
        )
    )
    return 0 if report["status"] == "stopped_before_wall" else 3


if __name__ == "__main__":
    raise SystemExit(main())
