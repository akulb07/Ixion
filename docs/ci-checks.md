# Acceptance checks in CI

`check` and `bundle-check` can write JUnit XML. Each acceptance check becomes a
test case, including the built-in execution and compatibility checks. The report
contains baseline/candidate IDs, the policy hash and recorded values/reasons.

For two saved runs:

```sh
roboforge check configs/navigation-acceptance.json --store results/service --baseline run-BASELINE_ID --candidate run-CANDIDATE_ID --output results/check.json --junit results/check.xml
```

For an existing portable package:

```sh
roboforge bundle-check results/navigation-check.zip --junit results/bundle-check.xml
```

Replace the IDs with real saved service run IDs. Output filenames must be new on
each invocation. Keep outputs outside the service store. An existing output,
duplicate JSON/XML output path or a path replacing an input is rejected before
creating the reports. Both commands use the same evaluator as the workspace;
they do not rerun simulation or change the baseline.

| Acceptance result | JUnit representation | Exit code |
| --- | --- | --- |
| Pass | Passing test case | 0 |
| Failed requirement | `failure`, type `AcceptanceFailure` | 3 |
| Inconclusive evidence | `error`, type `InconclusiveEvidence` | 4 |
| Invalid input, corrupt artifacts or output error | No complete report is guaranteed | 2 |

Inconclusive results are errors rather than skipped tests. A missing measurement
must not look like a successful build. If a report includes both a failure and
inconclusive checks, both appear in XML and the command returns 3. XML control
characters in user text are replaced; JSON remains the detailed original record.

Configure the CI runner to retain JSON/XML artifacts and publish the XML even
when the command returns nonzero. Preserve the command's exit code when adding
artifact-copying steps. For example, in a PowerShell script:

```powershell
roboforge bundle-check results/navigation-check.zip --junit results/bundle-check.xml
$checkExitCode = $LASTEXITCODE
# Artifact upload/publishing belongs to the CI runner configuration.
exit $checkExitCode
```

Use the intended RoboForge environment and archive the policy with the results.
This is report integration, not yet a hosted CI workflow, automatic scenario
execution, baseline promotion or code-review posting. JUnit consumers vary; the
generated XML is parsed in tests, but vendor dashboards have not been validated.
