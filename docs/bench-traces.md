# Comparing a trace from the bench

The first bench-data comparison imports a time-stamped CSV and compares its
values with one saved rover run. It doesn't rerun or change the simulation model.

```sh
ixion rover-bench run.json trace.csv metadata.json --time-offset-s 0 --max-sample-age-s 0.002 --output bench-comparison.json
ixion rover-report bench-comparison.json --output bench-comparison.html
```

For a suite report, add `--case normal` (or another case name). A multi-case suite
needs explicit selection. Model-error cases have no trace to compare. A design
comparison report isn't accepted as an implicit choice of its first design.

## Input format

The CSV requires `time_s` and at least one of `speed_m_s`, `distance_m`,
`pack_voltage_v` and `pack_current_a`. These column names declare SI units. There
is no automatic conversion from RPM, mph, milliamps or timestamp text. For example:

```csv
time_s,speed_m_s,pack_voltage_v,pack_current_a
0.00,0.00,7.30,0.10
0.02,0.01,,0.12
```

Blank signal cells mean missing evidence; zero is a measured zero. Timestamps
must be finite and strictly increasing. Negative source times are allowed for
pre-trigger recordings. Duplicate/unknown columns, ragged rows, nonfinite numbers
and traces with no signal values are rejected. The limits are 4 MiB and 50,000
samples. Metadata is bounded to 64 KiB and uses strict JSON with duplicate-key
rejection. A metadata file looks like:

```json
{
  "schema_version": 1,
  "source_kind": "measured",
  "reference": "Identifier of the original recording",
  "instrument": "Instrument and setup used for this recording",
  "conditions": "Payload, battery state, surface and operating conditions",
  "captured_at": null,
  "speed_reference": "wheel"
}
```

`source_kind` must be `measured` or `synthetic`. `reference`, `instrument` and
`conditions` must contain text. `captured_at` is an optional descriptive timestamp,
not a clock-alignment instruction. `speed_reference` is `wheel`, `ground` or
`unspecified`. These are user declarations; import doesn't authenticate an
instrument or establish that data really came from hardware.

## Alignment and errors

Both alignment arguments are required. Positive `--time-offset-s` adds to the CSV
time to place it on the simulation clock. There is no automatic first-sample
shift or drift correction. At each aligned time the comparator selects the latest
recorded simulation sample. `--max-sample-age-s` bounds how old that sample may be;
zero allows timestamp matches only, subject to floating-point rounding tolerance.

Before-run, after-run and stale rows remain unpaired. Four floating-point ULPs of
the source time, offset and selected times handle arithmetic at boundaries. A
source whose floating-point resolution is too coarse for the recorded cadence is
marked `insufficient_time_precision`. No interpolation or extrapolation supplies
missing state. Review alignment choices against the actual instrument clock and
sampling behavior; the example age limit isn't a hardware accuracy recommendation.

Each signal reports paired count, mean error, RMSE and maximum absolute error.
The error sign is **prediction minus measurement**. Signals without paired values
have `null` statistics. The report preserves every source row, matching reason,
aligned time, sample time/age and compared values. It includes the simulation and
source fingerprints for offline inspection. It doesn't automatically tune the
model, estimate confidence intervals or replace physical validation. A separate
[saved acceptance policy](bench-acceptance.md) can check declared error and
coverage limits without treating missing statistics as a pass.

The HTML shows source kind, metadata, coverage and error statistics. Paired values
overlay simulated speed/current, with voltage and travel plots when supplied.
The measurement-row control exposes individual paired or rejected rows and moves
the simulation cursor to its latest available earlier sample for an in-range time.
Out-of-range rows leave the simulation cursor unchanged. Rejected rows don't
become paired by selecting them. The viewer recomputes statistics from embedded
measurement rows and simulation evidence rather than trusting edited totals.

## Trying it without hardware

```sh
python examples/esp32_diff_drive/synthetic_bench_check.py --output results/synthetic-bench
ixion rover-report results/synthetic-bench/comparison.json --output results/synthetic-bench/comparison.html
```

This generates deliberately biased synthetic data: speed +0.01 m/s, voltage
-0.05 V and current +0.02 A relative to the saved run. Its source kind stays
`synthetic`. It demonstrates import/alignment/error calculation; it is not a
physical measurement or evidence of the rover's real accuracy.

CLI exit 0 means at least one value was paired and a comparison was produced,
not that prediction errors are acceptable. Exit 3 means no values could be paired
(the report still saves the exclusions). Exit 2 means invalid inputs or file errors.
Original reports, traces and metadata are never modified; outputs must be new.
The offline viewer's 64 MiB input limit also applies to the generated report.
