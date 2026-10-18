# Sending a bench check to someone else

The bench bundle keeps the original recording inputs and the declared limits in
one ZIP. The other person can verify it offline, recompute the checks and produce
their own HTML report. Verification reads saved evidence; it doesn't rerun physics.

```sh
ixion rover-bench-bundle run.json trace.csv metadata.json policy.json --time-offset-s 0 --max-sample-age-s 0.002 --output bench-evidence.zip
ixion verify-bench-bundle bench-evidence.zip --output verified.json --junit verified.xml
ixion rover-report verified.json --output verified.html
```

For a multi-case suite input, add `--case normal` or another explicit case name
when creating the bundle. The same [bench trace format and timing rules](bench-traces.md)
and [acceptance policy](bench-acceptance.md) apply. Source-kind, clock alignment,
conditions and instrument remain user declarations. Passing the limits isn't a
physical validation certificate.

## What's saved

There are exactly seven members:

- `run.json`, `trace.csv`, `metadata.json` and `policy.json`: the original input
  bytes, including their existing descriptions and references. A suite is kept
  whole even when one case is selected.
- `alignment.json`: explicit offset, sample-age allowance and case selection.
- `acceptance.json`: the check results, coverage, policy and fingerprints. This
  compact summary excludes the embedded comparison to avoid storing the simulation
  twice; use the verifier's `--output` to create a complete report for the viewer.
- `manifest.json`: format version, originating Ixion version, declared source
  kind, result and every other member's size and SHA-256 hash.

`comparison_sha256` in the acceptance summary is null because no standalone
comparison input file exists in this workflow. The original run, CSV, metadata
and policy fingerprints are retained and checked against their included bytes.

The writer uses stable member order, dates and file permissions. Identical inputs
produce identical archives in the same runtime. ZIP compression bytes may differ
between runtime versions. The initial format supports bench comparison and
acceptance schema 1; incompatible future changes to generated summaries need a
bundle format version change.

## What verification establishes

The verifier checks the fixed file names, member types, byte sizes and hashes,
then reparses the original files. It rebuilds the selected comparison, alignment,
metrics, coverage and acceptance checks and compares them with the saved summary
and manifest. Rehashing an altered result alone can't turn a failed check into a
pass. Someone can still create a different, internally consistent bundle from
different inputs: there are no signatures or instrument-authentication claims.

Nothing is extracted to disk. Optional JSON and XML exports must be new and
distinct from each other and the archive. Both are prepared before writing, then
written in order; the pair isn't an atomic transaction. The output directory must
exist. A failed acceptance result remains a valid archive and can still be
exported for review.

Limits are 70 MiB compressed and 69 MiB total uncompressed, with 64 MiB for the
run, 4 MiB for the CSV, 64 KiB each for metadata/policy/manifest, 16 KiB for
alignment and 128 KiB for the compact acceptance summary. The seven-file ZIP
footer and a 64 KiB directory limit are checked before member objects are built.
Extra names, paths, duplicates, directories, symbolic links, encrypted entries,
ZIP comments, ZIP64, split archives and compression other than stored/deflate
are rejected. The viewer's separate 64 MiB JSON limit still applies to an export.

Exit codes for both commands are 0 for passing declared acceptance, 3 for failed
acceptance, 4 for inconclusive acceptance and 2 for malformed inputs or archive
errors. A successful consistency check can therefore return 3 when the recorded
experiment fails its policy; the console states both results separately.
