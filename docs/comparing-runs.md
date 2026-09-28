# Comparing runs

I added this so I can put a normal run next to a faulty one without copying
numbers out of separate files.

Click the two-column icon in the activity bar. Select two to four finished runs,
choose the baseline, then click **Compare selected runs**. The run picker has
Newer/Older pages and keeps selections when changing pages. Refresh it after
running another experiment. Each result has an **Open replay** button when a
recorded replay exists; returning to comparison keeps the current snapshot.

The table shows saved measurements and signed differences from the baseline.
Units come from the metric names. A dash means a measurement or difference is
unavailable, not zero. Failed, cancelled and interrupted runs keep their status
and error instead of disappearing from the comparison. Collisions and exhausted
navigation budgets keep whatever metrics the simulator recorded.

There is no automatic winner. A shorter path could just mean an early collision,
and reaching an odometry goal doesn't mean the true position reached it. Check
the outcomes and both navigation error measurements before drawing conclusions.
These are individual trials, not statistical evidence or confidence intervals.

Changed configuration fields appear below the metrics. Arrays stay together and
can be expanded; missing keys are distinct from explicit null values. Different
seeds and different software versions are visible. Matching settings alone do
not establish a fair algorithm comparison.

**Export comparison** saves full configurations, run IDs, config hashes, recorded
versions, statuses, errors, metrics, deltas and the snapshot timestamp as JSON.
Changing the selection or baseline clears the previous snapshot. Comparisons
aren't stored as separate server jobs yet, so export before closing the page.

The API checks the config digest and the config/job/metrics manifest entries for
runs with saved results. This detects accidental changes to those files; it is
not a signature or an audit of every trajectory sample. The replay reader still
performs its own integrity checks. Older artifacts without these records aren't
silently converted.
