# Checking bench evidence against saved limits

A bench comparison tells you how far the saved simulation and recording differ.
A policy makes the limits explicit and lets a local check or CI fail when the
evidence doesn't meet them. Set and review your limits before collecting data.
The report records the policy and its fingerprint, but cannot prove when you
decided those limits.

```sh
ixion rover-bench-check comparison.json policy.json --output acceptance.json --junit acceptance.xml
ixion rover-report acceptance.json --output acceptance.html
```

The input is a [bench comparison](bench-traces.md), not a simulation run by itself.
The command rebuilds alignment and errors from the embedded source rows and
simulation. Edited statistics, status and check totals aren't trusted. The JSON
retains the normalized policy, source-file fingerprints, individual checks,
coverage and the recomputed comparison. HTML and JUnit rebuild the checks too.
Outputs must be new and distinct from each other and the inputs. JSON and optional
XML are prepared before writing, then written in order; the pair isn't an atomic
filesystem transaction. The output directory must already exist.

For an offline handoff with the original CSV, metadata, run and policy together,
use a [portable bench bundle](bench-bundles.md). The recipient can verify the
included inputs and recompute the checks without needing the original folder.

## A deliberate synthetic demonstration

```sh
python examples/esp32_diff_drive/synthetic_bench_check.py --output results/synthetic-bench
ixion rover-bench-check results/synthetic-bench/comparison.json examples/esp32_diff_drive/bench_synthetic_policy.json --output results/synthetic-bench/acceptance.json --junit results/synthetic-bench/acceptance.xml
ixion rover-report results/synthetic-bench/acceptance.json --output results/synthetic-bench/acceptance.html
```

This policy expects the deliberately biased **synthetic** trace. Its limits are
only for demonstrating the workflow. Passing it says nothing about hardware
accuracy. A policy requiring `measured` rejects a trace declared `synthetic`.
Origin and instrument metadata are still user declarations: relabelling a file
cannot establish that it came from a physical instrument.

## Policy format

Policies are strict JSON, bounded to 64 KiB. Unknown fields, duplicate keys,
nonfinite values and unsupported signals are rejected. This small example
describes a synthetic test, with illustrative limits:

```json
{
  "schema_version": 1,
  "name": "Synthetic speed demonstration",
  "reference": "Fixed generated bias; not hardware tolerances",
  "source_kind": "synthetic",
  "speed_reference": "wheel",
  "maximum_alignment_age_s": 0,
  "signals": {
    "speed_m_s": {
      "minimum_pairs": 50,
      "minimum_paired_fraction": 1,
      "minimum_paired_span_s": 1,
      "maximum_paired_gap_s": 0.03,
      "maximum_rmse": 0.011,
      "maximum_absolute_error": 0.011,
      "maximum_absolute_mean_error": 0.011
    }
  }
}
```

Supported signal names and error units are `speed_m_s` (m/s), `distance_m` (m),
`pack_voltage_v` (V) and `pack_current_a` (A). Every selected signal requires:

- A positive minimum pair count, a paired fraction from 0 to 1, and a nonnegative
  minimum time span. These must be stated explicitly.
- At least one nonnegative error ceiling: RMSE, maximum absolute error, or absolute
  mean error. The signed mean still uses prediction minus measurement.
- An optional maximum gap between adjacent paired timestamps. A gap needs at least
  two pairs; one pair can't establish it.

Speed limits also require an explicit `wheel` or `ground` reference, which must
match the declared recording. `maximum_alignment_age_s` limits the age allowance
used to create the comparison. It doesn't estimate the actual clock error.
Numeric thresholds are inclusive and use the computed values without adding an
implicit engineering tolerance. Choose margins for the intended use and instrument.

Paired fraction means paired values divided by **nonmissing supplied values for
that signal**. Rejected rows with a supplied value remain in the denominator;
blank cells aren't invented measurements. Span is last paired time minus first
paired time, and gaps use adjacent paired times. These checks don't infer omitted
instrument samples, continuous observation or independence between samples.
They also don't ensure the trace includes every operating condition you need.

## Results and limits

Every check keeps its actual value, limit, unit and result. A missing statistic
is `inconclusive`, never zero error. Overall status is `fail` if any limit fails,
otherwise `inconclusive` if any check lacks evidence, otherwise `pass`.
JUnit exports threshold failures as failures and inconclusive checks as errors.
It preserves source kind and policy identity and doesn't call simulation time
test execution time.

Exit codes are 0 for pass, 3 for fail, 4 for inconclusive and 2 for invalid input
or file errors. A pass only covers the declared limits and supplied evidence.
Instrument uncertainty, authenticity, physical readiness and model calibration
still require separate work. Source hashes identify bytes; they aren't signatures.
