# Planner benchmark suite v1

The static suite contains an empty room, corridor, alternating-wall maze, and
cluttered circle field. Exact geometry and start/goal coordinates are versioned
in code and copied into every result. All four cases have tested feasible grid
routes for the documented 0.2 m radius. Moving obstacles remain a future feature.

Every algorithm receives the same map, endpoints, footprint and explicit seed.
A*/Dijkstra use the same grid resolution and expansion budget. RRT/RRT* use the
same step size, seed, sampling iteration budget and collision oracle; RRT* also
records its rewire radius. A deterministic algorithm ignores the seed, so repeated
seeds do not represent independent trials for its output. Goal bias remains .1.

Reports retain status, path length when successful, expansion/iteration count,
collision-check count, and measured planner runtime. Every returned path is checked
again for exact endpoints and swept collision freedom before being called valid.
Failure has no path-length value and is never encoded as zero. Sampling budget
exhaustion is not proof that no path exists. Runtime excludes export and validation
on successful calls; error runtime may include time until failure is caught.

Per-case/algorithm summaries report trial and successful counts, failure rate,
mean successful path length, and median planner runtime. Path statistics are
conditional on success and must be read alongside failure counts. Local machine
runtimes are observations, not cross-machine performance guarantees. A single-seed
demo is illustrative, not a statistically powered comparison or universal ranking.

`roboforge benchmark --output results/benchmarks --seed 42 --iterations 500`
creates a unique directory with configuration, world fixtures, every path, JSON/CSV
summaries and SHA256 manifest. `report.html` is a standalone report with the exact
design and geometry inside it; `trials.csv` has one row per attempted search.
The HTML uses the same dark charcoal and purple theme as the workspace, with a
light print stylesheet. Open the details you want before printing. JSON and CSV
keep full numeric precision. All these files are included in the manifest.
Exit 0 means the benchmark report was produced;
individual search failures remain visible in the report. Python callers can choose
multiple seeds/cases/algorithms through BenchmarkConfig. Localization/control
comparisons use the separate experiment engine; map-quality and hardware-speed
benchmark standards are not yet claimed.
