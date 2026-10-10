"""Estimate shared battery sag with both reference motors held at zero speed."""

import argparse
import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel, BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.motor import MotorDatasheet
from roboforge.hardware.powertrain import MotorChannel, Powertrain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--step", type=float, default=1)
    args = parser.parse_args()
    project = load_project(Path(__file__).with_name("robot.yaml"))
    parts = {c.id: c for c in project.components}
    system = Powertrain(
        battery=BatteryModel.from_component(parts["battery"]),
        channels=tuple(
            MotorChannel(
                motor=MotorDatasheet.from_component(parts[name]).derive(),
                inputs=DriverInputs(standby=True, in1=True),
                output_speed_rad_s=0,
            )
            for name in ("left_motor", "right_motor")
        ),
    )
    trace = (
        None
        if args.seconds is None
        else system.discharge_trace(BatteryState(), args.seconds, args.step)
    )
    print(
        json.dumps(
            {
                "result_type": "estimate",
                "condition": "both shafts held at zero speed; not a startup transient",
                "inputs": {
                    name: [p.model_dump() for p in parts[name].parameters]
                    for name in ("battery", "left_motor", "right_motor")
                },
                "point": system.operating_point(BatteryState()).model_dump(),
                "trace": None if trace is None else trace.model_dump(),
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
