"""Estimate straight-line motion with explicit, assumed contact parameters."""

import argparse
import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel, BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.motor import MotorDatasheet
from roboforge.hardware.powertrain import MotorChannel, Powertrain
from roboforge.hardware.quadrature import AngularSample, QuadratureEncoder
from roboforge.hardware.rolling import RollingBody, run_rolling_bench


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=1)
    parser.add_argument("--step", type=float, default=0.001)
    parser.add_argument("--inertia", type=float, default=0.001)
    parser.add_argument("--friction", type=float, default=0.8)
    parser.add_argument("--driven-weight-fraction", type=float, default=0.8)
    parser.add_argument(
        "--encoder-cycles", type=int, help="assumed A/B cycles per output-wheel revolution"
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
    body = RollingBody(
        total_mass_kg=project.assembly.total_mass_kg,
        wheel_radius_m=project.assembly.wheel_radius_m,
        rotating_inertia_per_wheel_kg_m2=args.inertia,
        friction_coefficient=args.friction,
        driven_weight_fraction=args.driven_weight_fraction,
    )
    trace = run_rolling_bench(system, BatteryState(), body, args.seconds, args.step)
    encoders = None
    if args.encoder_cycles is not None:
        encoder = QuadratureEncoder(cycles_per_revolution=args.encoder_cycles)
        encoders = [
            encoder.trace(
                tuple(
                    AngularSample(time_s=s.shaft.time_s, angle_rad=s.shaft.displacement_rad[index])
                    for s in trace.samples
                )
            ).model_dump()
            for index in range(2)
        ]
    print(
        json.dumps(
            {
                "result_type": "estimate",
                "condition": "level ground, equal motors, no slip; inertia and contact values assumed",
                "inputs": {
                    name: [p.model_dump() for p in parts[name].parameters]
                    for name in ("battery", "left_motor", "right_motor")
                },
                "trace": trace.model_dump(),
                "encoder_traces": encoders,
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
