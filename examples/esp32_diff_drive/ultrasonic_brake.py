"""Brake a shaft from measured echo width in a scripted target-distance fixture."""

import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel, BatteryState
from roboforge.hardware.board_outputs import TB6612BoardOutputs
from roboforge.hardware.motor import MotorDatasheet
from roboforge.hardware.motor_wiring import resolve_motor_connection
from roboforge.hardware.powertrain import MotorChannel, Powertrain
from roboforge.hardware.shaft_dynamics import ShaftLoad
from roboforge.hardware.shaft_runtime import ShaftRuntime
from roboforge.hardware.signal_clock import SignalClock, SignalEdge
from roboforge.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor


def main():
    project = load_project(Path(__file__).with_name("robot.yaml"))
    parts = {c.id: c for c in project.components}
    connection = resolve_motor_connection(project, "left_motor")
    outputs = TB6612BoardOutputs(project, "esp32", connection.driver)
    outputs.write(digital={"GPIO13": True, "GPIO25": True}, pwm={"GPIO27": 1})
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
    clock = SignalClock({"echo": False})
    sensor = UltrasonicSensor(UltrasonicConfig(response_delay_s=0.0002))
    rising = None
    obstacle_detected = False
    brake_time = None
    readings, samples = [], []

    def capture(event):
        nonlocal rising, obstacle_detected
        if clock.read("echo"):
            rising = event.time_s
        elif rising is not None:
            distance = (event.time_s - rising) * sensor.config.sound_speed_m_s / 2
            readings.append({"time_s": event.time_s, "distance_m": distance})
            if distance <= 0.2:
                obstacle_detected = True
            rising = None

    clock.on_change("echo", capture)
    for index in range(1000):
        if obstacle_detected and brake_time is None:
            brake_time = runtime.time_s
            outputs.write(digital={"GPIO26": True})
        if index % 61 == 0:
            # External fixture: the target moves closer after 0.2 seconds.
            distance = 0.5 if runtime.time_s < 0.2 else 0.1
            pulse = sensor.trigger(runtime.time_s, 10e-6, distance)
            clock.schedule(
                (
                    SignalEdge(time_s=pulse.echo_rise_s, pin="echo", level=True),
                    SignalEdge(time_s=pulse.echo_fall_s, pin="echo", level=False),
                )
            )
        end = (index + 1) * 0.001
        result = runtime.step(
            end - runtime.time_s, (connection.motor_inputs(outputs.inputs(connection.channel)),)
        )
        clock.run_until(end)
        samples.append(
            {
                "time_s": end,
                "speed_rad_s": result.end.speed_rad_s[0],
                "driver_mode": result.end.point.channels[0].mode,
            }
        )
    print(
        json.dumps(
            {
                "result_type": "estimate",
                "fixture": "target 0.5 m until 0.2 s, then 0.1 m; not derived from rover travel",
                "assumptions": "independent shaft, inertia 0.01 kg m^2; logical echo input, no voltage/level-shifter model",
                "sensor": sensor.config.model_dump(),
                "brake_threshold_m": 0.2,
                "brake_command_time_s": brake_time,
                "readings": readings,
                "samples": samples,
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
