# When the voltage reading is wrong

An imperfect sensor can change a controller's decision even when the battery and
motors are unchanged. The sampled voltage fixture now accepts explicit gain,
offset and bounded noise assumptions. Try the six-case demo:

```sh
ixion rover-suite examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_voltage_error_suite.yaml --output voltage-errors.json --junit voltage-errors.xml
ixion rover-report voltage-errors.json --output voltage-errors.html
```

The same file ships in new starter projects. These are software experiments
with chosen values, not measured accuracy specifications for an ESP32 ADC.

## Declaring the error

Add `errors` under a scenario's [voltage monitor](rover-voltage-feedback.md):

```yaml
voltage_monitor:
  full_scale_v: 10
  resolution_bits: 12
  conversion_s: 0.003
  poll_every_steps: 10
  errors:
    gain: 1.05
    offset_v: -0.2
    noise_bound_v: 0.05
    seed: 7
```

At capture time, the fixture computes:

```text
unclipped voltage = source pack voltage * gain + offset_v + noise_v
```

Noise is bounded between `-noise_bound_v` and `+noise_bound_v`. The resulting
input clips below zero and above full scale, then uses the existing nearest-count
quantization. A value above full scale sets `over_range`. A negative input yields
zero counts; firmware sees a zero reading without a separate under-range flag.
Conversion delay and control-tick timing still apply after sampling.

Gain must be positive and at most 10, offset is bounded to ±1,000 V, noise bound
is 0–1,000 V, and seed is an integer from zero through `2**32 - 1`. These are
software input bounds, not component ratings. Leaving `errors` absent preserves
the ideal fixture and its saved encoding. An explicit empty `errors: {}` records
a neutral error model: gain one, offset zero and noise zero.

## What the demo shows

| Pair | Conditions | Observed result |
|---|---|---|
| `ideal_low` / `gain_reads_high` | 8 V brake threshold; ideal versus gain 1.05 | Startup pack voltage is about 7.7903 V in both. The ideal reading requests braking at 3 ms. The high reading misses that condition, and sonar later requests braking for the wall. |
| `ideal_above_threshold` / `offset_reads_low` | 7.5 V threshold; ideal versus −0.5 V offset | The ideal case reaches sonar braking. The low reading requests voltage braking at 3 ms despite the same initial pack voltage being above the chosen threshold. |
| `noise_seed_0` / `noise_seed_1` | 7.79 V threshold, ±0.05 V noise | The first low-voltage request occurs at 3 ms versus 33 ms. The same seeds reproduce those simulated sequences. |

The guard still checks only delivered observations. It can't read the saved
unquantized source or discover the declared bias. Braking remains latched after
the first request. Changing thresholds or adding filtering is a controller design
choice; this demo doesn't tune or certify one automatically.

## Repeating and inspecting it

Each accepted capture gets one deterministic noise draw, including a conversion
that is still pending at run end. Busy polls don't consume draws. A fresh monitor
starts at capture index zero, so unrelated sensors and other suite cases don't
advance its sequence.

The sequence is versioned by the ASCII input
`ixion-voltage-noise-v1:<seed>:<capture_index>`. The first eight SHA-256 bytes are
read as an unsigned big-endian integer, normalized by `2**64 - 1`, mapped to
`[-1, 1]`, and multiplied by the noise bound. This defines a repeatable software
perturbation; it doesn't establish the distribution, correlation or temperature
behavior of a physical sensor.

The raw report saves `source_voltage_v`, signed `unclipped_voltage_v` and
`noise_v` alongside each delivered or pending capture. Only the observation's
count, decoded voltage, timestamps and over-range flag reach firmware. The viewer
recomputes error draws and quantization from the saved configuration, and rejects
inconsistent evidence. Its plot overlays physical voltage and delivered readings;
the caption shows gain, offset, noise bound and seed. This is still rover schema
12, with a distinct `pack_voltage_monitor_with_errors` model identity.

[Build packets](build-reference.md) use manifest version 7 for any suite with
declared voltage errors, preserving them even when the selected case is ideal.
Versions 1–6 remain readable. MCU ADC pins, divider loading, sensor consumption,
drift, physical calibration and validation against measured data remain ahead.
