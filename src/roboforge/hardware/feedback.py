"""Bounded, project-wired host controller experiment for one shaft."""

import math

from pydantic import Field

from roboforge.config import Positive, Schema
from roboforge.core import finite

from .battery import BatteryModel, BatteryState
from .board_inputs import BoardInputReplay, EncoderBinding
from .board_outputs import TB6612BoardOutputs
from .motor import MotorDatasheet
from .motor_wiring import resolve_motor_connection
from .mounted_encoder import resolve_mounted_encoder
from .powertrain import MotorChannel, Powertrain
from .project import Identifier, RobotProject
from .quadrature import AngularSample
from .shaft_dynamics import ShaftLoad
from .shaft_runtime import ShaftRuntime


class FeedbackConfig(Schema):
    motor: Identifier = "left_motor"
    battery: Identifier = "battery"
    mcu: Identifier = "esp32"
    inertia_kg_m2: Positive
    target_count: int = Field(strict=True, gt=0)
    duration_s: Positive = 2
    step_s: Positive = 0.005


def run_encoder_feedback(project: RobotProject, config: FeedbackConfig) -> dict:
    count_steps = max(
        1, math.ceil(finite(config.duration_s / config.step_s, "feedback step count"))
    )
    if count_steps > 10_000:
        raise ValueError("feedback run exceeds 10,000-step budget")
    parts = {c.id: c for c in project.components}
    if config.battery not in parts:
        raise ValueError("selected battery does not exist")
    connection = resolve_motor_connection(project, config.motor)
    mounted = resolve_mounted_encoder(project, config.motor)
    outputs = TB6612BoardOutputs(project, config.mcu, connection.driver)
    pins = {signal: pin for pin, signal in outputs.pin_map.items()}
    channel = connection.channel
    outputs.write(
        digital={pins["STBY"]: True, pins[f"{channel}IN1"]: True}, pwm={pins[f"PWM{channel}"]: 1.0}
    )
    system = Powertrain(
        battery=BatteryModel.from_component(parts[config.battery]),
        channels=(
            MotorChannel(
                motor=MotorDatasheet.from_component(parts[config.motor]).derive(),
                inputs=connection.motor_inputs(outputs.inputs(channel)),
                output_speed_rad_s=0,
            ),
        ),
    )
    runtime = ShaftRuntime(system, BatteryState(), (ShaftLoad(inertia_kg_m2=config.inertia_kg_m2),))
    phases = [(False, False), (True, False), (True, True), (False, True)]
    observed, previous, events = 0, 0, 0
    brake_time = None
    samples = []
    for index in range(count_steps):
        if observed >= config.target_count and brake_time is None:
            brake_time = runtime.time_s
            outputs.write(digital={pins[f"{channel}IN2"]: True})
        end = min(config.duration_s, (index + 1) * config.step_s)
        elapsed = end - runtime.time_s
        if elapsed <= 0:
            break
        result = runtime.step(elapsed, (connection.motor_inputs(outputs.inputs(channel)),))
        encoder = mounted.quadrature.trace(
            (
                AngularSample(
                    time_s=0, angle_rad=result.start.displacement_rad[0] * mounted.angle_ratio
                ),
                AngularSample(
                    time_s=elapsed, angle_rad=result.end.displacement_rad[0] * mounted.angle_ratio
                ),
            )
        )
        events += len(encoder.edges)
        if events > 100_000:
            raise ValueError("feedback run exceeds 100,000-edge budget")
        gpio = BoardInputReplay(
            project,
            config.mcu,
            {"encoder": encoder},
            {"encoder": EncoderBinding(component=mounted.encoder)},
        )
        input_pins = {signal: pin for pin, (_, signal) in gpio.pin_map.items()}

        def decode(event):
            nonlocal observed, previous
            phase = phases.index(
                (gpio.digital_read(input_pins["a"]), gpio.digital_read(input_pins["b"]))
            )
            delta = (phase - previous) % 4
            if delta not in (1, 3):
                raise ValueError("invalid quadrature transition")
            observed += 1 if delta == 1 else -1
            previous = phase

        for pin in input_pins.values():
            gpio.attach_interrupt(pin, decode)
        gpio.run_until(elapsed)
        samples.append(
            {
                "time_s": result.end.time_s,
                "encoder_count": observed,
                "driver_mode": result.end.point.channels[0].mode,
                "speed_rad_s": result.end.speed_rad_s[0],
                "angle_rad": result.end.displacement_rad[0],
                "current_a": result.end.point.battery.current_a,
                "soc": result.end.state.soc,
            }
        )
    return {
        "schema_version": 1,
        "result_type": "estimate",
        "limitations": [
            "Independent shaft with assumed inertia; no chassis contact",
            "Rotor encoder uses positive mechanical orientation and ideal X4 signals",
            "Battery and 3.3 V logic supply are explicit; power nets are not executed",
            "Averaged PWM; no CPU emulation, interrupt latency or compiled firmware",
            "Controller adapts its pin choices to project wiring; not fixed-pin firmware",
            "Brake trigger is not a guaranteed stopping position",
        ],
        "config": config.model_dump(),
        "controller": "positive-count brake latch; host Python",
        "status": (
            "brake_triggered"
            if brake_time is not None
            else "target_reached_at_end"
            if observed >= config.target_count
            else "target_not_reached"
        ),
        "brake_command_time_s": brake_time,
        "motor_connection": connection.model_dump(),
        "mounted_encoder": mounted.model_dump(),
        "output_pin_map": outputs.pin_map,
        "input_pin_map": gpio.pin_map,
        "edge_count": events,
        "project": project.model_dump(mode="json"),
        "diagnostics": [d.model_dump() for d in outputs.diagnostics],
        "samples": samples,
    }
