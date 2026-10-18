# Comparing two rover designs

This is the first version of the parts-comparison workflow. Run the same
[suite](rover-suite.md) against two project files, then compare the saved evidence:

```sh
ixion rover-suite baseline-project.yaml examples/esp32_diff_drive/rover_feedback_suite.yaml --output baseline.json
ixion rover-suite candidate-project.yaml examples/esp32_diff_drive/rover_feedback_suite.yaml --output candidate.json
ixion rover-compare baseline.json candidate.json --output comparison.json
ixion rover-report comparison.json --output comparison.html
```

The comparison doesn't rerun either design. It checks that both reports declare
the same named cases, scenarios, controller settings and acceptance policies.
Case order can differ; silently changing the wall, target, timing or pass criteria
can't produce a comparison. The project itself is what may change: for example,
total mass, wheel radius, compatible motor parameters or battery parameters.
Both designs still have to fit the supported straight-line fixture.

For a complete example that adds an assumed 1 kg payload:

```sh
python examples/esp32_diff_drive/compare_payload.py --extra-mass-kg 1 --output results/payload-check
```

The example saves both project inputs, the common suite, both run reports, the
comparison JSON and an offline HTML viewer in a new directory. It adds mass to
the assembly; it doesn't model a payload's placement, changed centre of gravity
or weight distribution. Those remain explicit limits of the fixture. Use a new
directory for each run. Parent directories are created when needed.

## Reading the differences

Each case has baseline/candidate outcomes and values for clearance, stopping and
controller timing, travel, terminal energy, sampled peak pack current, final charge
and observed tracking RMSE when available. Every metric carries a unit. Deltas
are **candidate minus baseline**. They aren't an automatic better/worse verdict.
Missing evidence, including model-error runs or a missing stop, stays `null`.

Energy is accumulated over the actual recorded run duration. A design that hits
the wall or ends early may use less energy simply because it ran for less time.
Check outcomes and recorded duration before comparing cumulative values. The
tracking RMSE is based on observed encoder speed; faulty counts can make it
misleading even though the calculation is consistent with the saved feedback.

The HTML shows the paired cases and a metric table. Physical speed, PWM and pack
current include the other design's trace on the same time axis. Encoder plots and
the cursor readout belong to the selected design. The cursor never extrapolates
past that run's last saved sample; select the other design to inspect its later
events. Event details still belong to the selected run.

## What gets checked

The comparison validates supported report versions, project/schema consistency,
case membership, declared scenario and firmware settings, finite metric values,
final-sample metrics, summed interval energies, brake response arithmetic and
recomputed acceptance results. A declared peak below any saved current sample
is rejected. The original peak can also include interval-start operating points
that aren't separate plot samples, so equality with the plotted maximum isn't
required. It rejects missing or
contradictory evidence rather than silently treating it as a passing run. This
isn't a full physics recomputation, source-code attestation or tamper-proof audit.
The viewer recomputes comparisons from the embedded source reports rather than
displaying edited precomputed deltas.

Project fingerprints use normalized graph data: component, pin, parameter, net,
assignment and bus ordering doesn't count as a design change. Actual parameter
values, units and evidence references remain part of the fingerprint. Changes
are listed by explicit paths, retaining before/after values and whether a field
was added or removed. Lists longer than 1,000 changes are explicitly truncated.

The comparison includes both full source reports for offline inspection. CLI
source hashes identify the exact input JSON bytes; API calls without supplied
byte hashes use canonical JSON hashes and label that distinction. These hashes
identify data, not the simulator binary that produced it. The existing 64 MiB
viewer input limit still applies, including to comparisons with embedded runs.

Exit code 0 means the candidate meets the common suite's acceptance policy. Code
3 means candidate check failures or model errors. Code 2 means mismatched or
invalid evidence, unsupported inputs or file errors. A zero exit code doesn't
mean the candidate improves every metric. The comparison is saved for valid
candidate failures as well. Existing outputs and input files are never replaced.
