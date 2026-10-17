"""Ramp equal forward motor power, then brake from the normal sonar policy."""

import argparse
import json
from pathlib import Path

from pydantic import Field

from roboforge.config import Positive, Probability
from roboforge.hardware import load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_firmware import DriveRequest, FirmwareInputs, ObstacleBrakeFirmware


class RampedStartFirmware(ObstacleBrakeFirmware):
    initial_pwm_duty: Probability = Field(gt=0, default=0.25)
    ramp_duration_s: Positive = 0.4

    def on_tick(self, inputs: FirmwareInputs) -> DriveRequest:
        progress = min(inputs.time_s / self.ramp_duration_s, 1)
        return DriveRequest(pwm_duty=self.initial_pwm_duty + (1 - self.initial_pwm_duty) * progress)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-duty", type=float, default=0.25)
    parser.add_argument("--ramp-seconds", type=float, default=0.4)
    parser.add_argument("--output", type=Path, help="save the complete report to a new file")
    args = parser.parse_args()
    root = Path(__file__).parent
    try:
        scenario = load_rover_scenario(root / "rover_wall_debug.yaml").model_copy(
            update={"control_every_steps": 20}
        )
        controller = RampedStartFirmware(
            brake_threshold_m=scenario.brake_threshold_m,
            timeout_action=scenario.timeout_action,
            initial_pwm_duty=args.initial_duty,
            ramp_duration_s=args.ramp_seconds,
        )
        report = run_rover_brake(load_project(root / "robot.yaml"), scenario, firmware=controller)
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
                "firmware": report["firmware"],
                "metrics": report["metrics"],
            },
            indent=2,
            allow_nan=False,
        )
    )
    return 0 if report["status"] == "stopped_before_wall" else 3


if __name__ == "__main__":
    raise SystemExit(main())
