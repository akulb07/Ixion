"""Inspect ideal echo timing for a known one-meter target, then no target."""

import json

from roboforge.hardware.signal_clock import SignalClock, SignalEdge
from roboforge.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor


def main():
    config = UltrasonicConfig(response_delay_s=0.0002)
    sensor = UltrasonicSensor(config)
    pulses = [sensor.trigger(0, 10e-6, 1), sensor.trigger(0.1, 10e-6, None)]
    clock = SignalClock({"echo": False})
    first = pulses[0]
    clock.schedule(
        (
            SignalEdge(time_s=first.echo_rise_s, pin="echo", level=True),
            SignalEdge(time_s=first.echo_fall_s, pin="echo", level=False),
        )
    )
    width = clock.pulse_width("echo", config.timeout_s)
    measured_distance = width * config.sound_speed_m_s / 2
    clock.run_until(0.1)
    missing_width = clock.pulse_width("echo", config.timeout_s)
    print(
        json.dumps(
            {
                "result_type": "estimate",
                "config": config.model_dump(),
                "assumptions": "200 us response delay; ideal target; no echo on missing target",
                "measurements": [p.model_dump() for p in pulses],
                "captured_distance_m": measured_distance,
                "missing_echo_pulse_width_s": missing_width,
                "final_clock_time_s": clock.time_s,
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
