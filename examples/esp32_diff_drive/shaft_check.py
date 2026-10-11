"""Run an assumed independent-shaft bench for the reference motors."""

import argparse
import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel, BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.motor import MotorDatasheet
from roboforge.hardware.powertrain import MotorChannel, Powertrain
from roboforge.hardware.shaft_dynamics import ShaftLoad, run_shaft_bench


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=1)
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument(
        "--inertia", type=float, default=0.01, help="assumed effective output-axis kg m^2 per shaft"
    )
    parser.add_argument(
        "--load-torque", type=float, default=0, help="signed constant Nm subtracted per shaft"
    )
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
    loads = (ShaftLoad(inertia_kg_m2=args.inertia, load_torque_nm=args.load_torque),) * 2
    trace = run_shaft_bench(system, BatteryState(), loads, args.seconds, args.step)
    print(
        json.dumps(
            {
                "result_type": "estimate",
                "condition": "independent rotating shafts; assumed inertia, no wheel contact or chassis motion",
                "inputs": {
                    name: [p.model_dump() for p in parts[name].parameters]
                    for name in ("battery", "left_motor", "right_motor")
                },
                "trace": trace.model_dump(),
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
