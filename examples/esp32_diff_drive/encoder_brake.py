"""Host Python loop: drive one shaft, then brake when GPIO counts reach a target."""

import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel, BatteryState
from roboforge.hardware.board_inputs import BoardInputReplay, EncoderBinding
from roboforge.hardware.board_outputs import TB6612BoardOutputs
from roboforge.hardware.motor import MotorDatasheet
from roboforge.hardware.motor_wiring import resolve_motor_connection
from roboforge.hardware.powertrain import MotorChannel, Powertrain
from roboforge.hardware.quadrature import AngularSample, QuadratureEncoder
from roboforge.hardware.shaft_dynamics import ShaftLoad
from roboforge.hardware.shaft_runtime import ShaftRuntime


def main():
    project = load_project(Path(__file__).with_name("robot.yaml"))
    parts = {c.id: c for c in project.components}
    connection = resolve_motor_connection(project, "left_motor")
    outputs = TB6612BoardOutputs(project, "esp32", connection.driver)
    outputs.write(digital={"GPIO13": True, "GPIO25": True, "GPIO26": False}, pwm={"GPIO27": 1.0})
    system = Powertrain(
        battery=BatteryModel.from_component(parts["battery"]),
        channels=(
            MotorChannel(
                motor=MotorDatasheet.from_component(parts["left_motor"]).derive(),
                inputs=connection.motor_inputs(outputs.inputs(connection.channel)),
                output_speed_rad_s=0,
            ),
        ),
    )
    runtime = ShaftRuntime(system, BatteryState(), (ShaftLoad(inertia_kg_m2=0.01),))
    encoder = QuadratureEncoder(cycles_per_revolution=20)
    phases = [(False, False), (True, False), (True, True), (False, True)]
    count, previous = 0, 0
    target, interval = 40, 0.005
    brake_time = None
    samples = []
    for _ in range(400):
        if count >= target and brake_time is None:
            brake_time = runtime.time_s
            outputs.write(digital={"GPIO26": True})
        result = runtime.step(
            interval, (connection.motor_inputs(outputs.inputs(connection.channel)),)
        )
        # Encoder events become available after this mechanical interval. The
        # controller sees them before choosing the next interval's command.
        trace = encoder.trace(
            (
                AngularSample(time_s=0, angle_rad=result.start.displacement_rad[0]),
                AngularSample(time_s=interval, angle_rad=result.end.displacement_rad[0]),
            )
        )
        gpio = BoardInputReplay(
            project, "esp32", {"shaft": trace}, {"shaft": EncoderBinding(component="left_encoder")}
        )

        def read_encoder(event):
            nonlocal previous, count
            phase = phases.index((gpio.digital_read("GPIO32"), gpio.digital_read("GPIO33")))
            delta = (phase - previous) % 4
            if delta not in (1, 3):
                raise ValueError("invalid quadrature transition")
            count += 1 if delta == 1 else -1
            previous = phase

        gpio.attach_interrupt("GPIO32", read_encoder)
        gpio.attach_interrupt("GPIO33", read_encoder)
        gpio.run_until(interval)
        samples.append(
            {
                "time_s": result.end.time_s,
                "encoder_count": count,
                "driver_mode": result.end.point.channels[0].mode,
                "speed_rad_s": result.end.speed_rad_s[0],
                "soc": result.end.state.soc,
            }
        )
    print(
        json.dumps(
            {
                "result_type": "estimate",
                "target_count": target,
                "condition": "host Python shaft bench; no compiled ESP32 firmware",
                "assumed_inertia_kg_m2": 0.01,
                "encoder_cycles_per_output_revolution": 20,
                "control_interval_s": interval,
                "brake_command_time_s": brake_time,
                "output_pin_map": outputs.pin_map,
                "motor_connection": connection.model_dump(),
                "input_pin_map": gpio.pin_map,
                "wiring_diagnostics": [d.model_dump() for d in outputs.diagnostics],
                "inputs": {
                    name: [p.model_dump() for p in parts[name].parameters]
                    for name in ("battery", "left_motor")
                },
                "samples": samples,
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
