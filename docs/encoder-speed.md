# Trying encoder speed control

The rover can now adjust motor power from measured wheel motion, rather than
only following a power ramp. Try this from the repository root:

```sh
python examples/esp32_diff_drive/rover_speed_firmware.py --target-speed 0.2 --ramp-seconds 1 --seconds 4 --output speed-rover.json
```

The example ramps its target from zero to 0.2 m/s over one second. Every 20 ms it
estimates wheel speed from delivered encoder counts and asks for equal forward
PWM. Sonar still requests braking for the wall or a missing echo. Once that
request latches, the controller stops updating power and freezes its integral
while continuing to record wheel-speed observations.

The default reference run stops before the wall. Other targets, gains or parts
can leave the response unfinished or fail the model's limits. `--seconds` sets
the horizon; it doesn't guarantee that the rover finishes within it. The example
returns 0 for a stop before the wall, 3 for other valid outcomes and 2 for invalid
inputs or model/file errors. Pick a new output filename each time.

## What the controller measures

`EncoderSpeedConfig.from_project` resolves each rotor encoder, its X4 edge
multiplier and motor gear ratio. The reference has 4,800 decoded counts per wheel
revolution. Wheel radius comes from the project assembly. These are declared
engineering inputs, not measurements the controller discovers automatically.

`EncoderSpeedEstimator.update(inputs)` uses the same immutable encoder-count
frames that other host controllers receive. The first frame establishes an
anchor. Later frames accumulate until at least `minimum_interval_s` has elapsed;
the default is 20 ms. Each wheel's estimate is:

```text
speed = count_delta / counts_per_wheel_revolution * 2*pi*wheel_radius / elapsed_time
```

The result includes the interval, both count deltas, each wheel's speed and their
mean. This is average wheel speed over that window. The controller never reads
the simulator's true speed, displacement, current or battery state. Calling the
mean ground speed assumes no slip, as the current fixture does.

Short windows increase count quantization error. With ideal nearest-count
encoders, each wheel's count difference can differ from true travel by up to one
count. For a window lasting `dt`, the corresponding speed bound is
`2*pi*radius / counts_per_revolution / dt`. Longer windows trade lower quantization
error for later observations. The estimator accepts signed counts but doesn't
correct counter wrap, resets, missing edges or wrong resolution. It rejects
non-increasing timestamps and doesn't move its anchor on a failed calculation.

## Power calculation and limits

`EncoderSpeedFirmware` combines feedforward with proportional and integral
feedback. Speed error is target minus the mean wheel-speed estimate. The gains
and all command limits are saved as configuration. Their units are explicit:
feedforward and proportional gains are s/m; the integral gain is 1/m, multiplying
accumulated error in metres.

PWM stays between configured positive minimum and maximum values. A duty-rate
limit bounds each change using the actual observation interval. Conditional
anti-windup stops the integral from growing further into either an amplitude or
rate limit. This doesn't guarantee stable control for another design.

The supplied gains are illustrative settings for the assumed reference rover.
There is no automatic tuning or motor identification. Positive minimum PWM can
cause creep while the target is near zero. Static friction, breakaway torque and
real motor dead zones remain unmodeled. Zero-speed holding, reverse, turning and
restart after the latched brake remain outside this interface.

The electrical model is still discharge-only. A power reduction that demands
regeneration aborts the run; controller limits don't create a battery sink model
or silently clip negative current. Change gains, targets and cadence with that
limitation in mind, and compare smaller physics steps and physical measurements.

## Saved evidence

The example adds a `speed_control` block to the normal rover report. That block
has its own schema version 1 and contains controller configuration and samples:
time, ramped target, count-window observation, requested PWM, accumulated error,
limit flag and braking state. This extension is added by the example; ordinary
`run_rover_brake` reports retain their existing schema version 6. API callers can
obtain the same controller data from `controller.report()`.

The controller trace records requested power. Compare it with `control_decisions`
and the actual driver PWM samples to inspect application. Configuration also
appears in the normal `firmware.parameters` block; private controller history
doesn't get mixed into that configuration.

The saved post-ramp RMSE compares observed mean speed with target only after the
target ramp ends and before braking. Its observation count is included. It is
`null` if there are no eligible observations. This isn't a settling-time or
hardware-accuracy certificate: it includes the controller's remaining transient
and encoder quantization, and never substitutes simulator truth for feedback.

Every tenth completed observation also submits a readable debug message. That is
every 200 ms with the default cadence. The shared console can reject messages
without changing control or the saved controller trace. Use a fresh controller
instance per experiment; its estimator and integral intentionally retain state.
