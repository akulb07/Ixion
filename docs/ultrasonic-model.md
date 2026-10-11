# Ultrasonic timing and obstacle-triggered braking

```sh
python examples/esp32_diff_drive/ultrasonic_check.py
python examples/esp32_diff_drive/ultrasonic_brake.py
```

The first example produces echo timing for a known 1 m target and a missing target.
It recovers distance by capturing a digital pulse. The second runs one shaft with
a scripted target: 0.5 m until 0.2 seconds, then 0.1 m. A host controller measures
echo width and writes the brake input when measured distance is at most 0.2 m.
Target distance is external fixture data, not distance computed from rover travel.

## Sensor assumptions

The [HC-SR04 sheet distributed by SparkFun](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
specifies a trigger of at least 10 microseconds, a nominal 2 cm–4 m range, an
eight-cycle 40 kHz burst, and distance proportional to echo width. It recommends
measurement cycles over 60 ms. This model uses `width = 2 * distance / sound_speed`
and defaults to 340 m/s. It does not simulate the acoustic burst itself.

`UltrasonicConfig` requires an explicit response delay. The examples assume
200 microseconds after trigger completion; that is not a verified device latency.
The minimum-cycle setting defaults to 60 ms; the brake example polls every 61 ms.
The 30 ms timeout and missing-target behavior are simulation choices. A missing
or out-of-range target produces no pulse, not a clone-specific stuck-high signal.
The timeout must cover response delay plus the maximum supported echo width.

`UltrasonicSensor.trigger(time_s, width_s, distance_m)` returns an `EchoMeasurement`.
Short triggers are ignored. Premature retriggers, backwards time and invalid
numbers raise errors. Accepted triggers reserve the sensor until its measurement
and minimum cycle finish. Target geometry, beam cones, reflections, temperature,
noise, electrical supply and level shifting are not modeled.

## Signal clock

`SignalClock` stores boolean input levels and delivers timestamped `SignalEdge`
events. `run_until` advances simulation time; callbacks see the changed pin's new
level. Equal-time events preserve insertion order. Duplicate pin/timestamp pairs
are rejected. Scheduling validates an entire batch before adding any events.
There are at most 128 pins and 200,000 scheduled events over a clock's lifetime.

`pulse_width(pin, timeout_s)` waits for a fresh high pulse and returns its width or
`None` at the total deadline. If already high, the input must fall before a fresh
pulse can start. Other pin callbacks continue during the wait. A falling edge at
the deadline is included. All waiting uses simulation time, with no wall-clock
sleep. A callback failure stops the clock; rewind and nested clock execution are
rejected. Events added by callbacks must stay within the same time and size rules.

The braking example advances shaft mechanics in 1 ms intervals while delivering
echo edges, then applies newly detected braking on the next interval. It uses the
project's motor-driver GPIO output adapter but an ideal logical echo pin. The
reference's 5 V sensor, echo interface and MCU input net are not yet executing as
an electrical circuit. This is an obstacle-response shaft fixture, not the complete
Phase E rover demo or compiled firmware support.

## Reusable feedback experiment

The example now calls `run_ultrasonic_feedback(project, config)` from
`roboforge.hardware.ultrasonic_feedback`. `UltrasonicFeedbackConfig` includes the
motor/battery/MCU selection, assumed shaft inertia, sensor settings, control step,
poll interval, distance threshold and an ordered `TargetDistance` schedule. The
schedule starts at time zero; each value holds until the next entry. A distance of
`None` explicitly means no target response. The input is fixture data, not a scene.

`timeout_action="brake"` is the default controller policy. It latches braking when
the echo deadline expires without a complete pulse. `timeout_action="continue"`
records the timeout and continues driving, allowing comparison of controller
policies. Neither policy represents built-in HC-SR04 behavior. No missing reading
is replaced with zero distance or the last valid distance.

The full target schedule and project are saved in the returned report, together
with actual echo/timeout readings, detection time, brake-command time and reason.
The reason is `obstacle` or `echo_timeout`. Status is `brake_triggered`,
`brake_pending` if no following control interval remains, or
`completed_without_brake`. A trigger does not mean the shaft has stopped.

Runs are limited to 10,000 physics steps and 128 target changes. Poll intervals
must cover both the sensor's minimum cycle and timeout window. Sensor events are
processed before checking the deadline, so a completed echo wins over a timeout
at the same instant. Once selected, the first brake reason remains latched.
Driver pins are discovered from project wiring, including for the right motor.
