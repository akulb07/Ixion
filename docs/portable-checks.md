# Portable check bundles

This packages a saved baseline, candidate and acceptance policy into one ZIP. A
teammate can verify the files and repeat the acceptance check without the original
run store. This repeats evaluation of recorded results, not the physical simulation.

```sh
roboforge bundle configs/navigation-acceptance.json --store results/service --baseline run-BASELINE_ID --candidate run-CANDIDATE_ID --output results/navigation-check.zip
roboforge bundle-check results/navigation-check.zip
```

Replace the two IDs with saved run IDs. The output must be new and outside the run
store. Creating a package returns 0 even when its acceptance result is a failure:
packaging a failure is a normal use case. `bundle-check` returns 0 for pass, 3 for
failed requirements, 4 for inconclusive and 2 for invalid or damaged input.

The ZIP contains the request/policy, full check report, both run directories,
an export-environment record, instructions and SHA256 hashes. Completed runs
include the artifacts listed in their manifests, including replay telemetry.
Failed jobs retain their configuration and error record; absent telemetry is not
invented. The export verifies every copied artifact, not just metric files.

Checking validates the archive hashes, the per-run hashes and the repeated result.
It writes allowlisted run files to a temporary directory and cleans that directory
afterward. It never starts workers, executes archive code or imports runs into an
existing store. Archives are limited to 64 members and 128 MiB uncompressed.
Duplicate names and unexpected paths are rejected. Existing packages are never
overwritten.

The dependency and Python versions describe the environment creating the package.
Original run source revisions and dependency environments are not available yet,
so the package labels them as unknown. Package hashes detect corruption; they do
not authenticate an author who could replace both the data and the manifest.
If a future evaluator produces different check results, verification fails rather
than silently changing the recorded outcome. Use the recorded evaluator version
when investigating that difference.

This first version is CLI-only and supports RoboForge service runs. Full source
provenance, repeatable simulation execution and workspace ZIP downloads are still
separate work. A package can include application error text and all selected run
telemetry; inspect it before sharing it outside the team.
