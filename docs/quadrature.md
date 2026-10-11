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

This is post-processing of a mechanical trace. GPIO scheduling, interrupt
latency, missed pulses, noise, pull-ups, voltage levels and firmware execution
are not implemented here. The edges are the input for that future runtime.
