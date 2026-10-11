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
    control_every_steps: int = Field(strict=True, ge=1, le=10_000, default=1)


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
    target_time = None
    brake_angle = None
    previous_control_count = 0
    previous_control_time = 0.0
    counts_per_rad = finite(
        4 * mounted.quadrature.cycles_per_revolution * mounted.angle_ratio / math.tau,
        "encoder counts per output radian",
    )
    energy_j = 0.0
    peak_current = 0.0
    decisions = []
    samples = []
    for index in range(count_steps):
        if index % config.control_every_steps == 0:
            control_elapsed = runtime.time_s - previous_control_time
            measured_speed = (
                None
                if control_elapsed == 0
                else finite(
                    (observed - previous_control_count) / counts_per_rad / control_elapsed,
                    "encoder speed estimate",
                )
            )
            if observed >= config.target_count and brake_time is None:
                brake_time = runtime.time_s
                outputs.write(digital={pins[f"{channel}IN2"]: True})
            decisions.append(
                {
                    "time_s": runtime.time_s,
                    "encoder_count": observed,
                    "estimated_speed_rad_s": measured_speed,
                    "command": "drive" if brake_time is None else "brake",
                }
            )
            previous_control_count, previous_control_time = observed, runtime.time_s
        end = min(config.duration_s, (index + 1) * config.step_s)
        elapsed = end - runtime.time_s
        if elapsed <= 0:
            break
        result = runtime.step(elapsed, (connection.motor_inputs(outputs.inputs(channel)),))
        if brake_time is not None and brake_angle is None:
            brake_angle = result.start.displacement_rad[0]
        energy_j = finite(
            energy_j + result.start.point.battery.power_w * elapsed, "terminal energy"
        )
        peak_current = max(
            peak_current, result.start.point.battery.current_a, result.end.point.battery.current_a
        )
        interval_start_count = observed
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
            nonlocal observed, previous, target_time
            phase = phases.index(
                (gpio.digital_read(input_pins["a"]), gpio.digital_read(input_pins["b"]))
            )
            delta = (phase - previous) % 4
            if delta not in (1, 3):
                raise ValueError("invalid quadrature transition")
            observed += 1 if delta == 1 else -1
            previous = phase
            if observed >= config.target_count and target_time is None:
                target_time = result.start.time_s + event.time_s

        for pin in input_pins.values():
            gpio.attach_interrupt(pin, decode)
        gpio.run_until(elapsed)
        samples.append(
            {
                "time_s": result.end.time_s,
                "encoder_count": observed,
                "estimated_speed_rad_s": finite(
                    (observed - interval_start_count) / counts_per_rad / elapsed,
                    "interval encoder speed",
                ),
                "driver_mode": result.end.point.channels[0].mode,
                "speed_rad_s": result.end.speed_rad_s[0],
                "angle_rad": result.end.displacement_rad[0],
                "current_a": result.end.point.battery.current_a,
                "soc": result.end.state.soc,
            }
        )
    return {
        "schema_version": 2,
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
            else "target_reached_brake_pending"
            if observed >= config.target_count
            else "target_not_reached"
        ),
        "brake_command_time_s": brake_time,
        "control_decisions": decisions,
        "metrics": {
            "target_crossing_time_s": target_time,
            "controller_response_delay_s": None if brake_time is None else brake_time - target_time,
            "encoder_counts_per_output_revolution": counts_per_rad * math.tau,
            "final_count_error": observed - config.target_count,
            "rotation_after_brake_rad": None
            if brake_angle is None
            else samples[-1]["angle_rad"] - brake_angle,
            "final_speed_rad_s": samples[-1]["speed_rad_s"],
            "terminal_energy_j": energy_j,
            "peak_sampled_pack_current_a": peak_current,
        },
        "motor_connection": connection.model_dump(),
        "mounted_encoder": mounted.model_dump(),
        "output_pin_map": outputs.pin_map,
        "input_pin_map": gpio.pin_map,
        "edge_count": events,
        "project": project.model_dump(mode="json"),
        "diagnostics": [d.model_dump() for d in outputs.diagnostics],
        "samples": samples,
    }
