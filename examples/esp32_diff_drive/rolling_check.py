"""Estimate straight-line motion with explicit, assumed contact parameters."""

import argparse
import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel, BatteryState
from roboforge.hardware.board_inputs import BoardInputReplay, EncoderBinding
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.motor import MotorDatasheet
from roboforge.hardware.powertrain import MotorChannel, Powertrain
from roboforge.hardware.quadrature import AngularSample, EncoderTrace, QuadratureEncoder
from roboforge.hardware.rolling import RollingBody, run_rolling_bench


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=1)
    parser.add_argument("--step", type=float, default=0.001)
    parser.add_argument("--inertia", type=float, default=0.001)
    parser.add_argument("--friction", type=float, default=0.8)
    parser.add_argument("--driven-weight-fraction", type=float, default=0.8)
    parser.add_argument(
        "--grade-rad",
        type=float,
        default=0,
        help="constant signed grade; positive rises along travel",
    )
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
        grade_rad=args.grade_rad,
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
    gpio_counts = None
    board_pin_map = None
    if encoders is not None:
        replay = BoardInputReplay(
            project,
            "esp32",
            {
                name: EncoderTrace.model_validate(data)
                for name, data in zip(("left", "right"), encoders)
            },
            {name: EncoderBinding(component=f"{name}_encoder") for name in ("left", "right")},
        )
        board_pin_map = replay.pin_map
        pins = {signal: pin for pin, signal in board_pin_map.items()}

        def read_pair(name):
            return (replay.digital_read(pins[(name, "a")]), replay.digital_read(pins[(name, "b")]))

        phases = [(False, False), (True, False), (True, True), (False, True)]
        previous = {name: phases.index(read_pair(name)) for name in ("left", "right")}
        gpio_counts = {"left": 0, "right": 0}

        def count_edge(event):
            name = board_pin_map[event.pin][0]
            phase = phases.index(read_pair(name))
            delta = (phase - previous[name]) % 4
            if delta not in (1, 3):
                raise ValueError("invalid quadrature transition")
            gpio_counts[name] += 1 if delta == 1 else -1
            previous[name] = phase

        for pin in board_pin_map:
            replay.attach_interrupt(pin, count_edge)
        replay.run_until(args.seconds)
    print(
        json.dumps(
            {
                "result_type": "estimate",
                "condition": "constant declared grade, equal motors, no slip; inertia and contact values assumed",
                "inputs": {
                    name: [p.model_dump() for p in parts[name].parameters]
                    for name in ("battery", "left_motor", "right_motor")
                },
                "trace": trace.model_dump(),
                "encoder_traces": encoders,
                "gpio_decoded_counts": gpio_counts,
                "board_pin_map": board_pin_map,
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
