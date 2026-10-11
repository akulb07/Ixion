# Encoder edges from wheel motion

```sh
python examples/esp32_diff_drive/rolling_check.py --encoder-cycles 20
```

The optional encoder output converts each wheel's sampled rotation into ideal
timestamped A/B transitions. The example's 20 cycles per wheel revolution is an
explicit assumption, not a verified encoder rating. Four transitions per cycle
give 80 counts per wheel revolution with X4 decoding. NI describes the
[X4 edge-counting convention](https://knowledge.ni.com/KnowledgeArticleDetails?id=kA03q000000x1riCAA&l=en-US).

`QuadratureEncoder.trace` accepts time and angle samples. Resolution is cycles per
revolution of the supplied angle. For a motor-mounted encoder, supply rotor angle
or convert its resolution to output-wheel cycles yourself; no gear ratio is
silently applied. Positive rotation uses A/B states `00 -> 10 -> 11 -> 01 -> 00`.
Reverse rotation traverses them backward. Initial phase and count are reported
separately without inventing events before the first sample.

Angles are quantized into centered count cells: `floor(angle * counts_per_rad +
0.5)`. Zero angle sits halfway between transitions. At an exact boundary, the
higher count owns the boundary. A reversal from that boundary may generate an
edge at the next segment's starting time. This is an ideal convention with no
hysteresis or debounce. Counts can be negative and do not wrap.

Edge times interpolate linearly between angle samples. Each segment is assumed
monotonic, so a reversal occurring entirely between two samples cannot be
recovered. There can be many edges within one simulation step. The converter
keeps them all or rejects the run if its 100,000-edge budget is exceeded; it never
silently drops pulses. Input is limited to 10,001 samples with strictly increasing
times, and extremely large angles are rejected before floating-point precision
can erase adjacent thresholds.

## Logical GPIO replay

The same example now reports `gpio_decoded_counts`. It delivers the encoder
traces through `EncoderGPIOReplay`, then reconstructs signed counts in callbacks
using A/B pin reads alone. Callbacks receive time, channel name, changed pin and
new level; they do not receive the encoder's ground-truth count.

Register callbacks with `on_change(channel, callback)`, read inputs with
`read(channel, "a")` or `read(channel, "b")`, and advance simulation time with
`run_until(seconds)`. Initial levels are available immediately without triggering
callbacks. An event at the requested end time is included exactly once. Equal-time
events run in channel-name order, preserving source order within each channel;
callbacks see other channels only after their events have been delivered.

The replay accepts at most 16 traces and 200,000 events, all initialized at time
zero. Backwards time, reentrant execution and callback registration after starting
are rejected. Callback errors propagate and permanently stop that replay object;
already delivered effects are not rolled back. Create a new replay to start again.
There is no wall-clock wait or simulated callback execution cost.

## Project MCU inputs

`BoardInputReplay` resolves named encoder traces through an `EncoderBinding` and
the project's nets to a selected MCU. The reference rolling example now reads
GPIO32/33 for the left encoder and GPIO34/35 for the right, and exports the resolved
`board_pin_map`. Bindings identify the encoder component and its A/B output pins;
destination MCU pins come from the wiring, not a hardcoded adapter map.

Use `digital_read("GPIO32")` for a mapped input, and
`attach_interrupt("GPIO32", callback)` for a pin assigned `INTERRUPT`. Callbacks
receive MCU name, pin name, time and level. A `DIGITAL_IN` assignment permits reads
but not interrupt attachment. Unmapped pins cannot be read through this adapter.
The example discovers the mapping for convenience; firmware that uses fixed pin
names will see reversed counts if the project swaps the A/B wires.

This first adapter supports direct two-endpoint encoder-to-MCU nets. It rejects
project validation errors, duplicate bindings and unsupported pin assignments.
Validation warnings remain available in `diagnostics`; unknown electrical ratings
are still unknown. Interface components, signal fan-out and power activation are
not simulated. Passing validation is not a hardware readiness guarantee.

This still replays a completed mechanical trace. It does not yet execute a live
motor-control loop, emulate ESP32 instructions or run compiled firmware. Interrupt
latency, missed pulses, noise, pull-ups and voltage levels are not modeled by this
logical input layer.

The separate [shaft feedback example](shaft-runtime.md) now uses these ideal
signals between incremental mechanical steps to choose the next driver command.
