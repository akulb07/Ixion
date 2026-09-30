# Acceptance checks

The first testing workflow compares two saved service runs without rerunning them.
In Compare runs, choose the baseline, compare the runs, and use Acceptance checks.
The JSON policy can be saved and pasted back later. The check report includes the
policy hash, exact settings, recorded measurements and per-rule explanations.

For automation, use real run IDs from the service store:

```sh
roboforge check configs/acceptance.json --store results/service --baseline run-BASELINE_ID --candidate run-CANDIDATE_ID --output results/check-001.json
```

Exit codes: 0 pass, 3 failed requirement, 4 inconclusive, 2 invalid input or
integrity error. The output filename must be new and outside the run store.
The CLI reads the same artifacts as the API, without starting workers, recovering
jobs or modifying baselines. Active runs cannot be checked. This currently works
with RoboForge service runs, not external recordings or bare CLI simulation folders.

Rules name a recorded metric and an inclusive minimum, maximum, or both.
`mode: "absolute"` checks the candidate value; `mode: "delta"` checks candidate
minus baseline in the metric's units. Negative changes are allowed. There is no
implicit percentage conversion or assumption that larger values are better.
Only metrics in the saved run's metrics.json are supported in this first version;
offline analysis panel metrics are not automatically included.

Candidate execution must complete normally. Missing data is inconclusive.
Failure takes precedence over inconclusive when combining checks. Normal execution
does not necessarily mean navigation reached its goal; an explicit navigation-goal
rule is still needed before using this as a navigation acceptance suite.

Config differences apart from names make the result inconclusive until their exact
comparison paths are listed in `allowed_config_changes`. Array differences use
the whole array's path, such as `commands`. Software version differences require
`allow_software_change: true`. These acknowledgments do not prove scenarios are
scientifically comparable. A delta also needs a completed baseline and both values.

Checks validate recorded configuration and metric checksums. Hashes detect artifact
changes, not malicious replacement of both artifacts and manifests. This version
records package versions, not full source/dependency provenance yet. A passing
single pair is not a statistical result or a safety certification.

`POST /api/regressions` takes `baseline_id`, `candidate_id` and `policy`. Invalid
policies return 422; unfinished runs and integrity conflicts return 409.
