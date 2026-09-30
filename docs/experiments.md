# Experiments, comparisons and replay

ExperimentConfig embeds a validated RunConfig, unique nonnegative seeds and sweep
axes. Each axis addresses an existing field using dot paths (list entries use an
index, e.g. sensors.0.noise.stddev). Cartesian products run once per seed. Overlapping
paths and seed axes are rejected. Entire subobjects may be replaced. Parameter
values must serialize as finite JSON. Typos and run/step budget overruns fail before
execution; invalid individual field values become retained failed trials.

Each invocation creates a new UUID directory. Trial IDs include index and canonical
resolved-input SHA256; reports store full parameters, seed, software version, UTC
start and measured wall runtime. Simulation time never depends on wall runtime.
Trial exceptions and collision stops are retained as failures; subsequent trials
continue. Completed-trial reports are atomically replaced after each trial so an
interrupted batch keeps prior results. There is no automatic resume/retry yet.

Metrics include executed path length, duration, terminal pose, collision count,
wheel-rate-squared time integral and optional measured PID error RMSE. The wheel
integral is an effort proxy, not physical energy. Extra metric functions may be
registered; nonfinite values fail the trial. Per-group summaries report total and
failed counts, failure rate with a Wilson 95% interval, and each available metric's
sample count, mean, sample standard deviation, median and 5th/95th percentiles.
Metrics from collision runs remain included if available; missing metrics are
counted explicitly, never filled with zero. Wilson intervals assume independent
Bernoulli trials; a few seeds cannot establish a reliable general failure rate.

paired_differences compares challenger-minus-baseline by shared seed and reports
excluded seeds (failed/missing runs or metrics). Callers choose comparable variants;
matching seeds alone does not establish equal environments when sweeping geometry.
No automatic winner, composite score or significance claim is generated.

CLI: `roboforge experiment experiment.yaml --output results/runs [--plot]`.
Exit 0 means every trial completed; 3 means a retained failure; 2 is a setup error.
JSON and CSV summaries support inspection without the Python object graph.

Each trial has a SHA256 file manifest. ReplayLog verifies it when present, loads
recorded telemetry, and reconstructs pose inside a step from the stored exact/Euler
motion segment. It never runs simulation, sensors, controllers or planners. Sensor
visibility uses delivery time, preserving latency. Control and actuator records
are held from the latest recorded update; collision terminal states retain zero
wheel rates. Exported actuator commands describe attempted intervals, not post-stop
physical velocity. Replay is currently for SimulationResult traces; SLAM graph/map
artifacts remain separately inspectable exports.

CLI: `roboforge replay <trial-directory> --time 1.25` prints a recorded snapshot.
Manifest hashes detect accidental file corruption, not authenticity or signatures.
## Portable reports

CLI experiments also save `report.html` and `trials.csv`. The HTML includes the
design, summary and every recorded trial, and can be opened without a server.
Expand a trial to see its measurements and configuration hash. The existing
`metrics.csv` remains available. A root `manifest.json` hashes the report files;
each trial directory still has its own manifest. Reports describe recorded
measurements and do not rerun the simulator.
