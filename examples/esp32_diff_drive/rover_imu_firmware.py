"""Brake from a fresh IMU acceleration reading, with the sonar policy as fallback."""

import argparse
import json
from pathlib import Path

from pydantic import Field

from roboforge.config import Nonnegative, Positive
from roboforge.hardware import load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_firmware import (
    BrakeRequest,
    FirmwareInputs,
    FirmwareLog,
    FirmwareResponse,
    FirmwareResult,
    IMUObservation,
    ObstacleBrakeFirmware,
)


class AccelerationLimitFirmware(ObstacleBrakeFirmware):
    maximum_forward_acceleration_m_s2: Positive = 2
    maximum_sample_age_s: Nonnegative = 0.02
    log_decisions: bool = Field(strict=True, default=False)

    def on_imu(self, observation: IMUObservation, inputs: FirmwareInputs) -> FirmwareResult:
        age = inputs.time_s - observation.sample_time_s
        request = None
        if age > self.maximum_sample_age_s:
            text = (
                f"Ignoring stale IMU frame: age {age:.6f} s exceeds {self.maximum_sample_age_s:g} s"
            )
        elif observation.specific_force_m_s2[0] >= self.maximum_forward_acceleration_m_s2:
            request = BrakeRequest(reason="acceleration_limit")
            text = (
                f"Braking requested: forward acceleration {observation.specific_force_m_s2[0]:.6f} m/s^2 "
                f"reached {self.maximum_forward_acceleration_m_s2:g} m/s^2; sample age {age:.6f} s"
            )
        else:
            return None
        if self.log_decisions:
            return FirmwareResponse(
                brake_request=request, logs=(FirmwareLog(text=text, level="warning"),)
            )
        return request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-acceleration", type=float, default=2, help="forward limit in m/s^2")
    parser.add_argument("--max-sample-age", type=float, default=0.02, help="age limit in seconds")
    parser.add_argument(
        "--log-decisions",
        action="store_true",
        help="queue controller explanations on the debug console",
    )
    parser.add_argument("--output", type=Path, help="save the complete report to a new file")
    args = parser.parse_args()
    root = Path(__file__).parent
    try:
        scenario = load_rover_scenario(root / "rover_wall_debug.yaml")
        program = AccelerationLimitFirmware(
            brake_threshold_m=scenario.brake_threshold_m,
            timeout_action=scenario.timeout_action,
            maximum_forward_acceleration_m_s2=args.max_acceleration,
            maximum_sample_age_s=args.max_sample_age,
            log_decisions=args.log_decisions,
        )
        report = run_rover_brake(load_project(root / "robot.yaml"), scenario, firmware=program)
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
