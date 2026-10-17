# Ultrasonic timing and obstacle-triggered braking

Run the supplied scenarios from the repository root after installing the checkout:

```sh
ixion ultrasonic-feedback examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/obstacle.yaml --output obstacle-report.json
ixion ultrasonic-feedback examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/missing_echo.yaml --output missing-echo-report.json
```

Edit a copy of either scenario to change inertia, timing, distance threshold or
timeout policy. Set `distance_m: null` for a missing response. JSON is also accepted.
Files are limited to 64 KiB; unknown fields, duplicate keys and executable YAML
tags are rejected. Reports contain the resolved configuration and project, so the
inputs remain available alongside the results. Choose a fresh output filename;
existing reports are never overwritten and the parent folder must already exist.

The command returns 0 when braking was triggered, 3 when the experiment completed
without applying braking (including a pending brake), and 2 for invalid input or
runtime/file errors. A triggered brake is not proof that the shaft has stopped.
Failed simulations do not create a report. Each command prints status and brake
reason, making obstacle and timeout results easy to distinguish.

The smaller Python examples remain available:

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
At that boundary, an echo timestamp a few floating-point rounding units beyond
the validated deadline is clamped to the deadline. A materially late echo is
rejected. This prevents arithmetic order from turning a valid boundary echo
into a timeout; it does not extend the sensor's range or timeout window.

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

`call_at(time_s, callback)` also schedules peripheral completion callbacks on
this timeline. Equal-time GPIO and peripheral work share insertion order and the
same event budget. A callback may queue further work but cannot advance the
clock recursively. This lets I2C reads complete while sonar/encoder edges continue.

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
an electrical circuit. This shaft fixture remains separate from the newer
[rover wall-brake fixture](rover-brake.md), which supplies distance from straight-line
travel toward one fixed wall. Both run host Python; the shaft fixture uses an
ideal logical echo name, while the rover now resolves project-wired MCU pins.

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

## Moving rover fixture

`ixion rover-brake PROJECT SCENARIO --output report.json` uses the same pulse
timing model with two motors and a rolling body. At each trigger, range is
`wall_distance_m - sensor_offset_m - distance_travelled_m`. That sampled distance
determines the whole echo pulse; motion during acoustic flight, beam shape and
reflections are not simulated. Front clearance uses the separate
`front_offset_m`, so the sensor threshold and the body collision boundary need
not be at the same point.

The controller sees pulse width or timeout and decoded encoder pins. The rover
uses `UltrasonicBoardIO`: rising and falling trigger GPIO edges cause a measurement,
with distance sampled on the rising edge. Echo edges return on the resolved MCU
input, through one explicitly selected ideal non-inverting interface or a direct
net. The reference placeholder has no analog voltage-conversion or delay model.
Reports save the resolved path and delivered pin edges. See the
[fixture notes](rover-brake.md) for scenarios, outcomes and the speed threshold
used to end a successful braking run.

The adapter also supports manual `digital_write` calls and `digital_read` on its
mapped echo pin. `trigger_pulse` queues the high/low output edges without advancing
the clock or mechanics. Short pulses create no echo. Minimum-cycle and pulse-width
comparisons allow up to four timestamp units in the last place for floating point
rounding; measurably early polls and short pulses remain invalid or ignored.
