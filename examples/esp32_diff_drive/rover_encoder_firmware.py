"""Replace the wall-range policy with an encoder-count brake controller."""

import argparse
import json
from pathlib import Path

from pydantic import Field

from roboforge.config import Schema
from roboforge.hardware import load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_firmware import BrakeRequest, FirmwareInputs, RangeObservation


class EncoderLimitFirmware(Schema):
    target_count: int = Field(strict=True, gt=0, default=6000)

    def on_range(
        self, observation: RangeObservation, inputs: FirmwareInputs
    ) -> BrakeRequest | None:
        return None

    def on_tick(self, inputs: FirmwareInputs) -> BrakeRequest | None:
        if min(inputs.left_encoder_count, inputs.right_encoder_count) >= self.target_count:
            return BrakeRequest(reason="encoder_limit")
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-count", type=int, default=6000)
    parser.add_argument("--output", type=Path, help="save the complete report to a new file")
    args = parser.parse_args()
    root = Path(__file__).parent
    try:
        program = EncoderLimitFirmware(target_count=args.target_count)
        report = run_rover_brake(
            load_project(root / "robot.yaml"),
            load_rover_scenario(root / "rover_wall_debug.yaml"),
            firmware=program,
        )
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
