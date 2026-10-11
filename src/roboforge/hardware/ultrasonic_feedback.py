"""Brake a shaft from measured echo width in a scripted target-distance fixture."""

import math
from typing import Literal

from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Positive, Schema
from roboforge.core import finite
from roboforge.hardware.battery import BatteryModel, BatteryState
from roboforge.hardware.board_outputs import TB6612BoardOutputs
from roboforge.hardware.motor import MotorDatasheet
from roboforge.hardware.motor_wiring import resolve_motor_connection
from roboforge.hardware.powertrain import MotorChannel, Powertrain
from roboforge.hardware.project import Identifier, RobotProject
from roboforge.hardware.shaft_dynamics import ShaftLoad
from roboforge.hardware.shaft_runtime import ShaftRuntime
from roboforge.hardware.signal_clock import SignalClock, SignalEdge
from roboforge.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor


class TargetDistance(Schema):
    time_s: Nonnegative
    distance_m: Nonnegative | None


class UltrasonicFeedbackConfig(Schema):
    motor: Identifier = "left_motor"
    battery: Identifier = "battery"
    mcu: Identifier = "esp32"
    inertia_kg_m2: Positive
    sensor: UltrasonicConfig
    targets: tuple[TargetDistance, ...] = Field(min_length=1, max_length=128)
    duration_s: Positive = 1
    step_s: Positive = 0.001
    poll_every_steps: int = Field(strict=True, gt=0, le=10_000, default=61)
    brake_threshold_m: Positive = 0.2
    timeout_action: Literal["brake", "continue"] = "brake"

    @model_validator(mode="after")
    def validate_timing(self):
        if self.targets[0].time_s != 0 or any(
            b.time_s <= a.time_s for a, b in zip(self.targets, self.targets[1:])
        ):
            raise ValueError("target schedule must start at zero and strictly increase")
        period = finite(self.step_s * self.poll_every_steps, "sensor poll period")
        if period < max(
            self.sensor.minimum_cycle_s, self.sensor.minimum_trigger_s + self.sensor.timeout_s
        ):
            raise ValueError("poll period must cover sensor cycle and timeout")
        return self


def run_ultrasonic_feedback(project: RobotProject, config: UltrasonicFeedbackConfig) -> dict:
    steps = max(1, math.ceil(finite(config.duration_s / config.step_s, "ultrasonic step count")))
    if steps > 10_000:
        raise ValueError("ultrasonic feedback exceeds 10,000-step budget")
    parts = {c.id: c for c in project.components}
    connection = resolve_motor_connection(project, config.motor)
    outputs = TB6612BoardOutputs(project, config.mcu, connection.driver)
    pins = {signal: pin for pin, signal in outputs.pin_map.items()}
    channel = connection.channel
    outputs.write(
        digital={pins["STBY"]: True, pins[f"{channel}IN1"]: True}, pwm={pins[f"PWM{channel}"]: 1}
    )
    if config.battery not in parts:
        raise ValueError("selected battery does not exist")
    system = Powertrain(
        battery=BatteryModel.from_component(parts[config.battery]),
        channels=(
            MotorChannel(
                motor=MotorDatasheet.from_component(parts[config.motor]).derive(),
                inputs=connection.motor_inputs(outputs.inputs(connection.channel)),
                output_speed_rad_s=0,
            ),
        ),
    )
    runtime = ShaftRuntime(system, BatteryState(), (ShaftLoad(inertia_kg_m2=config.inertia_kg_m2),))
    clock = SignalClock({"echo": False})
    sensor = UltrasonicSensor(config.sensor)
    rising = None
    brake_reason = None
    pending_deadline = None
    detection_time = None
    fixture_index = 0
    brake_time = None
    readings, samples = [], []

    def capture(event):
        nonlocal rising, brake_reason, pending_deadline, detection_time
        if clock.read("echo"):
            rising = event.time_s
        elif rising is not None:
            distance = (event.time_s - rising) * sensor.config.sound_speed_m_s / 2
            readings.append({"time_s": event.time_s, "distance_m": distance, "status": "echo"})
            pending_deadline = None
            if distance <= config.brake_threshold_m and brake_reason is None:
                brake_reason = "obstacle"
                detection_time = event.time_s
            rising = None

    clock.on_change("echo", capture)
    for index in range(steps):
        if brake_reason is not None and brake_time is None:
            brake_time = runtime.time_s
            outputs.write(digital={pins[f"{channel}IN2"]: True})
        if index % config.poll_every_steps == 0:
            while (
                fixture_index + 1 < len(config.targets)
                and config.targets[fixture_index + 1].time_s <= runtime.time_s
            ):
                fixture_index += 1
            pulse = sensor.trigger(
                runtime.time_s,
                config.sensor.minimum_trigger_s,
                config.targets[fixture_index].distance_m,
            )
            pending_deadline = pulse.timeout_time_s
            if pulse.status == "echo":
                clock.schedule(
                    (
                        SignalEdge(time_s=pulse.echo_rise_s, pin="echo", level=True),
                        SignalEdge(time_s=pulse.echo_fall_s, pin="echo", level=False),
                    )
                )
        end = min(config.duration_s, (index + 1) * config.step_s)
        if end <= runtime.time_s:
            break
        result = runtime.step(
            end - runtime.time_s, (connection.motor_inputs(outputs.inputs(connection.channel)),)
        )
        clock.run_until(end)
        if pending_deadline is not None and pending_deadline <= end:
            readings.append({"time_s": pending_deadline, "distance_m": None, "status": "timeout"})
            if config.timeout_action == "brake" and brake_reason is None:
                brake_reason, detection_time = "echo_timeout", pending_deadline
            pending_deadline = None
        samples.append(
            {
                "time_s": end,
                "speed_rad_s": result.end.speed_rad_s[0],
                "driver_mode": result.end.point.channels[0].mode,
            }
        )
    return {
        "schema_version": 1,
        "result_type": "estimate",
        "config": config.model_dump(mode="json"),
        "project": project.model_dump(mode="json"),
        "limitations": [
            "Scripted target distances, independent shaft; no rover scene",
            "Ideal echo pin; voltage and level-shifter circuit not simulated",
            "Timeout action is a chosen controller policy, not a hardware guarantee",
        ],
        "status": "brake_triggered"
        if brake_time is not None
        else "brake_pending"
        if brake_reason
        else "completed_without_brake",
        "brake_reason": brake_reason,
        "detection_time_s": detection_time,
        "brake_command_time_s": brake_time,
        "readings": readings,
        "samples": samples,
        "motor_connection": connection.model_dump(),
        "output_pin_map": outputs.pin_map,
        "diagnostics": [d.model_dump() for d in outputs.diagnostics],
    }
