# Braking from a delivered voltage reading

The rover can now brake from a sampled battery-voltage observation. This adds
quantization, conversion time and a control-tick delay to the feedback loop.
Try the four supplied cases:

```sh
ixion rover-suite examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_voltage_feedback_suite.yaml --output feedback-suite.json --junit feedback-suite.xml
ixion rover-report feedback-suite.json --output feedback-suite.html
```

The cases compare fresh readings, slower readings, readings rejected as stale
and readings above the monitor's full scale. Each case saves its own stop reason,
motion trace and acceptance checks. The 8 V threshold is an illustrative demo
setting, not a recommended battery protection setting.

## What gets sampled

Add an optional monitor to a normal [rover scenario](rover-brake.md):

```yaml
voltage_monitor:
  full_scale_v: 10
  resolution_bits: 12
  conversion_s: 0.003
  poll_every_steps: 10
```

This is an ideal pack-voltage fixture. It doesn't resolve an ESP32 ADC pin or
simulate a divider circuit, input loading or sensor power consumption. Optional
[gain, offset and bounded noise](rover-voltage-errors.md) can be declared separately;
they default to the ideal path. The monitor samples the last committed physics endpoint at
each requested poll. A motor command applied at that boundary changes the next
physics interval; it doesn't rewrite the preceding endpoint's voltage.

Counts run from zero to `2**resolution_bits - 1`. Conversion rounds to the nearest
count, with exact half counts rounded upward. Values above full scale clip to
the maximum count and set `over_range`; a value exactly at full scale doesn't.
The decoded voltage is `raw_count * full_scale_v / maximum_count`.

Resolution is 2–24 bits, conversion time must be positive and at most one second,
and polls occur every 1–10,000 requested physics steps. Only one conversion can
be pending. Polling while busy increments `busy_skips` and preserves the held
sample. Delivered conversions and busy skips each have a 10,000-entry budget.
If the run ends before completion, that conversion stays pending.

## The example controller

A [suite case](rover-suite.md) selects the host controller and its settings:

```yaml
controller: voltage_guard
voltage_guard:
  minimum_voltage_v: 8
  maximum_sample_age_s: 0.01
  log_decisions: false
```

This controller requires a scenario with `voltage_monitor`. On delivery, it
checks the observation in this order:

1. Above full scale: request braking with `voltage_monitor_range`.
2. Delivery time minus sample time exceeds the age limit: `voltage_sample_stale`.
3. Decoded voltage is at or below the threshold: `low_voltage`.

The age limit is inclusive and is checked when the reading arrives. Sonar
braking remains available. The first brake request latches, and outputs change
at the next control tick. Later voltage recovery doesn't release that brake.
Optional decision messages use the existing serial queue; a full or disabled
console doesn't cancel the brake request.

With the supplied 1 ms physics step and 20-step control interval, the first
sample is captured at time zero. A 3 ms conversion requests braking at 3 ms,
and the motor command changes at 20 ms. A 23 ms conversion with a 50 ms age
allowance requests at 23 ms and changes outputs at 40 ms. A delivery exactly
on a control boundary runs before that boundary's controller tick.

This differs from the independently configured [declared voltage boundary](rover-voltage-limits.md),
which checks the modeled supply directly and can trigger a [latched driver
response](rover-power-response.md) without waiting for a sensor reading. The
supplied feedback suite leaves that boundary unset so the sensor/controller
path determines the brake request.

## Using it from Python

A custom host controller can add this optional callback alongside `on_range`
and `on_tick`:

```python
from roboforge.hardware.rover_firmware import BrakeRequest

def on_voltage(self, observation, inputs):
    if observation.over_range or observation.voltage_v <= 8:
        return BrakeRequest(reason="voltage_guard")
    return None
```

The observation contains delivery `time_s`, `sample_time_s`, `raw_count`, decoded
`voltage_v` and `over_range`. It doesn't expose the unquantized source voltage.
`inputs` is the normal immutable controller frame at delivery time. Return values
follow the [firmware callback contract](rover-firmware.md), including drive/brake
requests and bounded log messages.

Programs with `on_voltage` record firmware interface version 5 and a successful
callback count. Programs without it retain version 4; the monitor can still
record observations without changing their control policy.

## Saved evidence

Monitor-enabled rover reports use schema 12. They retain the configured monitor,
delivered captures, one optional pending capture and busy-skip count. Each
capture also saves its unquantized source voltage for audit, outside the
controller observation. Power and interval-start evidence are included even
when the scenario uses default battery conditions.

The offline viewer checks the recorded source endpoint, quantization, conversion
latency, capture order, nonoverlap and delivered/pending timing. When a voltage
callback is present, its count must match completed readings. It overlays decoded
voltage at delivery times, adds voltage events and keeps future or pending
observations out of the cursor readout. It doesn't infer readings between
captures or establish physical accuracy.

[Build-reference packets](build-reference.md) use manifest version 6 for suites
with an ideal voltage monitor, or version 7 with declared sensor errors. They preserve the selected controller settings and
scenario alongside the design. Older packet versions remain readable. Exported
settings are host-controller configuration; MCU ADC circuitry, board firmware
and validation against a physical rover remain ahead.
