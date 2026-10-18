# Trying a different rover controller

The wall-brake runner now accepts a Python controller without needing changes to
the motion or sensor code. It can set equal forward PWM on both wheels and latch
a brake request. It's useful for trying a soft start or comparing when to stop.
Reversing, steering, restart after braking and a general firmware scheduler are
still ahead.

Try the encoder-based example from the repository root:

```sh
python examples/esp32_diff_drive/rover_encoder_firmware.py --target-count 6000 --output encoder-rover.json
```

It uses the same robot and debug scenario as the sonar demo, but requests braking
once both decoded encoder counts reach 6000. Sonar, IMU, power and serial reporting
keep running. A lower count requests braking earlier; a high count can leave too
little room. The output file must be new. The example returns 0 for a stop before
the wall, 3 for other valid outcomes and 2 for invalid inputs or file errors.

## Controller interface

`run_rover_brake(project, scenario, firmware=program)` accepts an object with two
required methods and an optional `on_imu` callback:

```python
from roboforge.hardware.rover_firmware import BrakeRequest

class MyController:
    def on_range(self, observation, inputs):
        if observation.status == "echo" and observation.distance_m < 0.6:
            return BrakeRequest(reason="close_obstacle")
        if observation.status == "timeout":
            return BrakeRequest(reason="missing_echo")
        return None

    def on_tick(self, inputs):
        if min(inputs.left_encoder_count, inputs.right_encoder_count) >= 6000:
            return BrakeRequest(reason="encoder_limit")
        return None
```

Callbacks can return a `BrakeRequest`, `DriveRequest`, `FirmwareResponse` or `None`.
Reasons are short identifiers
such as `encoder_limit`. The first request stays latched, so later events cannot
replace its reason or release the brake. A request from `on_tick` applies at that
tick. A range callback requests braking at delivery time, and the outputs change
at the next control tick. IMU callbacks follow the same rule. Reports retain both
times and their difference.

`inputs` is an immutable `FirmwareInputs` frame containing simulation time and the
two encoder counter readings. Optional [counter-read faults](encoder-faults.md)
can alter those readings after GPIO decoding. `observation` is an immutable `RangeObservation` with
delivery time, status and measured distance. A timeout has no distance. Neither
frame includes the wall position, actual travel, motor state or the fixture's true
range. Range is still decoded by the host from the project-wired echo pulse.

Echo callbacks run when the falling edge is delivered. Timeouts run at their exact
deadline on the shared clock; a complete echo at the same deadline wins. The run
doesn't advance past its end to finish a pending reading. Encoder counts contain
only transitions already delivered, including the clock's insertion order when
events share a timestamp.

The default starts at full forward duty. The runner invokes `on_tick` at time zero
before calculating the initial current, acceleration and traction. A custom
controller can therefore start at lower power or brake immediately without a
phantom full-power sample. It continues delivering callbacks until the run ends,
even after braking starts. Sonar and optional IMU polling still belong to the host.
This interface doesn't yet let controller code issue arbitrary GPIO or I2C calls.

Omitting `firmware` selects `ObstacleBrakeFirmware`, which uses the scenario's
`brake_threshold_m` and `timeout_action`. Passing a program replaces that policy;
those two scenario fields then describe the unused default. Use a fresh program
instance per run if it keeps state. YAML and JSON scenarios remain data only and
cannot name a module to import or code to execute.

## Changing motor power

Try the ramped-start example:

```sh
python examples/esp32_diff_drive/rover_pwm_firmware.py --initial-duty 0.25 --ramp-seconds 0.4 --output pwm-rover.json
```

It starts at 25% duty, ramps to full duty over 0.4 seconds, and keeps the normal
sonar braking policy. Its controller runs every 20 physics steps: with the
reference's 1 ms physics step, power changes every 20 ms. Both motors receive
the same command through their mapped TB6612 PWM pins. This is an averaged
voltage command, not a promise that 25% duty means 25% wheel speed.

Return `DriveRequest(pwm_duty=0.5)` from a callback, or put it in
`FirmwareResponse(drive_request=..., logs=...)`. Duty must be finite and greater
than zero, up to one. Zero isn't a coast command in this interface; use an
explicit `BrakeRequest` to stop. A response cannot contain both drive and brake
requests. Reverse, coast and unequal left/right power are unsupported here.

Sensor callbacks queue a duty request until the next control tick. The latest
request before application wins, including any new request returned by that
tick's callback. After application the duty holds until another request. A brake
request cancels pending duty changes and stays latched. Later drive requests are
ignored and counted, so a periodic ramp cannot undo sonar braking. Changing power
alone doesn't count as obstacle detection or create a brake result.

The power model still rejects regeneration. Reducing duty at speed can demand
negative pack current; that aborts the estimate with an explicit error rather
than clipping current or pretending the battery can absorb it. Reductions that
remain within the discharge-only model are supported. Averaged PWM doesn't
simulate switching ripple, current recirculation, CPU cost or real driver timing.
The ramp example uses increasing duty; it is not a general speed regulator.

A separate [encoder speed controller](encoder-speed.md) now closes the loop on
count-window wheel-speed observations. It combines PI feedback with feedforward,
bounded duty changes and sonar braking. Its own guide explains configuration,
saved target/measurement traces and the remaining model limits.

## Using the IMU in a controller

Add `on_imu(self, observation, inputs)` to receive each completed motion-register
read. Existing controllers don't need to add the method. The `IMURoverFirmware`
protocol describes this optional extension. Try the supplied example:

```sh
python examples/esp32_diff_drive/rover_imu_firmware.py --max-acceleration 2 --max-sample-age 0.02 --output imu-controller.json
```

`AccelerationLimitFirmware` requests braking when decoded forward specific force
reaches the chosen limit. The example uses the fixture's aligned x-forward axis;
it doesn't use the magnitude of the gravity-bearing three-axis vector. Sonar
threshold and missing-echo braking remain active as a fallback. This is an example
policy with assumed limits, not a validated protection system.

On a [declared incline](rover-inclines.md), forward specific force includes
`g*sin(grade)`. This example doesn't estimate attitude or remove that gravity
component, so tilt can change its decision even at the same body acceleration.

Its optional callback receives an immutable `IMUObservation`: delivery time,
sample time, decoded specific-force and angular-velocity tuples, and temperature.
The host builds this from the completed byte transfer, after range scaling and
quantization. It doesn't expose the ideal physical sample or give code access to
a frame while its read is pending. Encoder counts in `inputs` are those already
delivered at the callback time.

`sample_time_s` is simulation metadata for the sample latched by that read, not a
timestamp register supplied by the MPU6050. The example checks
`inputs.time_s - observation.sample_time_s` and ignores frames older than
`--max-sample-age`. A slow bus can therefore leave the sonar policy in charge.
The ideal sensor has no internal sampling clock yet, and these timestamps don't
model firmware running on a real board.

IMU callbacks continue after braking begins and run independently of serial-log
decimation. Disabling the console doesn't disable sensor delivery. When IMU
polling is absent, still initializing or busy, no extra frame is invented. A read
completing exactly at the run end is delivered, but it can't apply a new brake
unless there is a following control tick within the run.

## Debug messages from controller code

Add `--log-decisions` to the IMU example to record why it requests braking or
ignores a stale frame:

```sh
python examples/esp32_diff_drive/rover_imu_firmware.py --log-decisions --output imu-controller-debug.json
```

A callback can submit messages and a brake request together:

```python
from roboforge.hardware.rover_firmware import BrakeRequest, FirmwareLog, FirmwareResponse

def on_tick(self, inputs):
    if inputs.left_encoder_count >= 6000:
        return FirmwareResponse(
            brake_request=BrakeRequest(reason="encoder_limit"),
            logs=(FirmwareLog(text="Count limit reached; requesting brake", level="info"),),
        )
    return None
```

`brake_request` is optional, and `logs` defaults to an empty tuple. A response can
therefore just log a decision without changing the controller's request. The host
validates the entire response before latching a request or sending any messages.
Returning plain `BrakeRequest`, `DriveRequest` or `None` still works.

Each message becomes a `firmware_log` JSON event on the shared serial console,
with `callback`, `level`, `text` and the callback's simulation time. Levels are
`debug`, `info`, `warning` or `error`; they label messages and don't change motor
behavior. Embedded newlines and JSON remain escaped text inside that event. They
cannot replace the host's event type or timestamp. A message about requesting a
brake can have an earlier timestamp than the host's actual `brake` event.

The queue uses the existing UTF-8/8N1 timing and byte/write budgets. Its encoded
JSON envelope also counts toward those limits. A full buffer rejects the whole
message without delaying control. With serial disabled, messages are rejected
without creating a queue. Rejection doesn't cancel a brake request. Accepted
messages can remain pending at run end; accepted means queued, not delivered.
Rejected queue writes retain time and byte count, not their message text.

Each response allows at most 16 messages, each with 1–4,096 UTF-8 bytes of text.
Invalid text, levels, oversized responses and exhausted console lifetime budgets
fail the run. Callback code doesn't receive a direct console handle or immediate
queue feedback; use the saved report to inspect accepted and rejected counts.

## Reports and limits

Rover reports use schema version 7, with a `firmware` block containing interface version,
qualified Python class name, execution mode, latched request, delivery time and
current command. Controllers based on Ixion's `Schema` also save their parameter
fields, as the encoder example does. Other Python objects report `parameters: null`;
their internal state and configuration are not captured automatically. The report
doesn't bundle controller source or guarantee that custom code can be reproduced.

Firmware interface version 4 adds forward PWM requests and preserves existing
range/tick and optional IMU programs. `imu_callback` records whether the program has a handler;
`imu_callback_count` counts successfully delivered calls. The enclosing `imu`
report still contains every completed read, including when the controller has no
IMU handler. Optional callback exceptions and invalid return values fail the run
in the same way as range or tick callback errors.

`log_write_attempts`, `log_writes_accepted` and `log_writes_rejected` count only
controller-submitted messages. The enclosing `serial` block includes both host
and controller traffic, with completion/pending records and queue drops.

Each `control_decisions` row now includes `pwm_duty` and the source
`drive_request_time_s` of the last applied drive request. Samples include both
channels' `driver_pwm_duty`. These describe interval commands; a sample at a
later control boundary still reflects the interval that just ended. Reports also
keep drive request/application counts, ignored requests after braking and any
pending duty request at run end. A newer request can supersede an earlier one
before either is applied.

Motion samples and decisions now distinguish firmware-observed `encoder_counts`
from underlying `decoded_encoder_counts`. `encoder_feedback` records the optional
injected faults. These additional report fields aren't hidden firmware inputs.
The speed controller's separate report saves configured checks and trip evidence.

The serial `drive_pwm` event records actual duty changes, with request and command
times. Unchanged duty doesn't generate another change event. In brake mode PWM
is held high and both direction inputs are high: a recorded duty of one in that
mode means the TB6612's short brake, not forward full power.

The existing motion, GPIO, I2C and serial fields remain available. The serial brake
event records the custom reason and actual output-change time. Callback exceptions
and invalid return values abort the run rather than producing a successful report.

This runs trusted Python in the same process. A slow or stuck callback can hold up
the host; there is no simulated instruction duration, CPU budget, sandbox, RTOS or
ESP32 binary execution. The fixed step, event and peripheral budgets still apply
to the experiment. The [rover model limits](rover-brake.md) still apply too.
