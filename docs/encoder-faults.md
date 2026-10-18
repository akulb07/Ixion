# Trying bad encoder feedback

The speed demo can now run with frozen counter readings or a sudden count jump.
The point is to see how the controller responds when the readings stop making
sense. Run one of these from the repository root, using a new output name each time:

```sh
python examples/esp32_diff_drive/rover_speed_firmware.py --encoder-fault left-freeze --output left-freeze.json
python examples/esp32_diff_drive/rover_speed_firmware.py --encoder-fault both-freeze --output both-freeze.json
python examples/esp32_diff_drive/rover_speed_firmware.py --encoder-fault left-jump --output left-jump.json
```

Faults start at 0.8 s unless changed with `--fault-at`. `left-freeze` holds the left
count at its value on activation. `both-freeze` holds both. `left-jump` adds 10,000
to each subsequent left count reading. Each run still includes the ordinary sonar
brake policy and the same physical rover model.

The example enables feedback checks by default, including when no fault is
selected. `--no-encoder-checks` disables them for a virtual comparison; it leaves
sonar braking enabled. Some comparisons can fail the model's electrical limits
before reaching a stop. The flag isn't a hardware operating recommendation.

## Where the fault happens

This is a fault in the counter readings passed to firmware, after quadrature GPIO
delivery and decoding. The underlying decoder keeps counting; the motors and
physical state keep moving until a control command changes them. This does not
simulate an unplugged wire, missing electrical edges, noise, voltage levels or an
interrupt handler losing events.

The bounded scenario schema accepts `encoder_faults`, with at most one permanent
fault per wheel. For example, add this to a copy of `rover_wall_debug.yaml`:

```yaml
encoder_faults:
  - channel: left
    start_time_s: 0.8
    kind: freeze
```

An `offset` instead requires a nonzero integer `offset_counts` between
-1,000,000,000 and 1,000,000,000. `freeze` requires zero offset. Starts must be
finite, nonnegative and within the run horizon. There is no recovery event or
sequence of faults on the same wheel yet. YAML selects data, not Python code or
a controller. The ordinary `ixion rover-brake` command still uses its sonar-only
controller; the Python speed example installs the feedback controller explicitly.

Activations share the simulation clock's insertion order with GPIO and peripheral
events. A time-zero fault is visible before the first control tick. For later
equal-time events, counts delivered before activation enter the snapshot; counts
delivered afterward don't. If a fault activates at the final horizon, the last
sample shows it, but there is no extra controller tick after the run ends.

## What makes the controller brake

API users opt in with
`EncoderSpeedFirmware(..., encoder_health=EncoderHealthConfig())`. The class
default is `None`, which leaves these checks disabled. Each fresh controller
checks complete [speed-estimator windows](encoder-speed.md) before its PI update:

| Check | Default condition | Saved reason |
|---|---|---|
| Reverse counts | Either wheel has a negative count delta | `encoder_reverse` |
| Implausible speed | Either absolute wheel-speed estimate exceeds 2 m/s | `encoder_speed_implausible` |
| No progress | A wheel has zero count delta for at least 0.2 s after its first qualifying zero-window observation | `encoder_no_progress` |
| Disagreement | Wheel-speed estimates differ by more than 0.05 m/s for at least 0.1 s after the first qualifying observation | `encoder_disagreement` |

No-progress and disagreement timers run only while the ramped target is at least
0.05 m/s and the previously requested duty is at least 0.05. Ineligible windows
clear those timers. Count progress resets that wheel's no-progress timer; an
allowed speed difference resets disagreement. Reverse and implausible-speed checks
don't have that eligibility gate. All thresholds are configuration, saved with the
controller. No-progress takes priority if both persistence checks expire together.

The first trip returns a brake request on that control tick. The host latches it,
and PI integration and further drive requests stop. Speed observations continue
through braking. A prior sonar brake keeps its original reason. Console drops or
a disabled console cannot postpone these decisions.

With the supplied reference parameters and a 20 ms control interval, the 0.8 s
left freeze trips at 0.92 s, both freezes trip at 1.02 s, and the jump trips at
0.8 s. All three reach the stopped-speed threshold before wall contact. These are
deterministic fixture results, not measured hardware response times.

## Reading the report

Rover report schema 7 saves:

- `encoder_feedback`: injection layer, requested faults and actual activations,
  including each raw count at activation. A fault that hasn't fired has no activation.
- `encoder_counts`: readings delivered to firmware, in motion samples and control
  decisions. Serial status and brake events use the same observed readings.
- `decoded_encoder_counts`: underlying GPIO-decoder counts in those samples and
  decisions. These are report evidence and are never passed as hidden controller inputs.

The example's `speed_control` extension uses schema 2. `encoder_health` is `null`
when checks are disabled; otherwise it contains `fault: null` or the first latched
fault. Fault evidence includes detection time, first suspect observation time,
involved wheels, the exact count-window observation, ramped target and previous
requested duty. Each subsequent control sample retains `feedback_fault_reason`.
The main report's `brake_reason` and timing metrics show whether and when the host
applied that request. Fault activation and controller error messages also attempt
to enter the bounded serial queue, so a missing console message isn't evidence
that no fault or brake occurred.

## Limits of these checks

A stalled mechanism can produce the same zero-count evidence as a frozen sensor.
Disagreement flags the pair; it can't establish which encoder is wrong. Persistent
rules assume equal forward wheel commands, matching the current straight-line
fixture. They aren't suitable unchanged for turning, reverse motion, slipping
wheels or arbitrary encoder resolution and timing. Set thresholds from the
intended mechanism, measurement resolution and operating range.

A constant offset already present at the estimator's first anchor cancels out of
subsequent count differences and is not detected. Small jumps, matching faults,
wrong scaling or errors below thresholds can also escape detection. No-progress
timing begins with the first complete suspect observation, not the unknown moment
at which hardware failed. There is no recovery, watchdog, emergency-stop circuit,
general fault diagnosis or safety certification here. Physical validation and a
broader fault model are still ahead.
