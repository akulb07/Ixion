# Configuration, state, time and the foundation simulator

## Configuration contract

`RunConfig` is the current root schema, with `schema_version: 1`, name, seed,
`RobotConfig`, `Environment`, `SimulationConfig`, and a nonempty tuple of
`WheelCommand` segments. All schemas are frozen and forbid extra fields. A typo
such as `wheel_raduis` therefore fails instead of silently choosing a default.

Numerical floats must be finite real values (integers are accepted), while steps
and seeds must be strict integers. Booleans and numeric strings are rejected.
Dimensions and dt must be positive. Commands allow signed angular rates and
require positive step counts. The integrator is `exact` or `euler`.

`load_config` uses safe YAML parsing for both YAML and JSON, rejects duplicate
mapping keys and non-string keys, validates schemas, and includes the filename
and field path in errors. JSON does not imply a second schema. Configuration
loading performs no evaluation, network lookup, or arbitrary object creation.

Environment bounds are `[0,width] x [0,height]`. Rectangles use a lower-left
origin and positive width/height; circles use centre/radius. Obstacles must fit
inside bounds. Initial robot centre must be inside bounds. Footprint clearance,
initial obstacle overlap, and future motion outside bounds are **not checked**;
they belong to the collision phase. Obstacles and bounds are passive geometry.

## Robot state

`RobotState` carries immutable `pose`, `wheels`, `twist`, and simulation `time`.
`vx`, `vy`, and `omega` are derived properties, preventing pose and stored world
velocity components from disagreeing. Pose is canonical; the twist represents
the just-applied segment's constant motion. Initial state is stationary.

The ideal robot has no acceleration or torque model: the command becomes the
wheel rate immediately. `footprint_radius` is used only by visualization. Wheel
mass, chassis mass and actuator response are deliberately deferred until they
have meaningful physical effects rather than becoming unused parameters.

## Clock and update order

`SimulationClock(dt,tick)` is an immutable clock with nonnegative integer ticks.
Its time is calculated as `tick*dt`, not by repeated floating-point addition.
Frequency is `1/dt`. `Simulator` owns the clock and supplies timestamps to the
robot. Wall-clock duration has no influence on motion.

The initial state is recorded at tick 0. For each command segment:

1. Choose constant left/right angular rates.
2. Advance the authoritative tick.
3. Integrate the state over dt with those rates.
4. Record the end-of-step state at the new clock time.

The end-of-step sample stores the command used during the preceding interval.
For the example, sample 1000 at t=10 s still contains rates (1,1); sample 1001
contains (-1,1). Total samples equal one plus the sum of segment steps.

`Simulator.run()` allocates fresh robot state and clock. A second call cannot
continue a previous run accidentally. No random generator is needed yet; the
validated seed is retained as metadata, explicitly marked unused by this model.

## Result formats

`save_result` writes the resolved `config.json`, `metadata.json`, and
`trajectory.csv`. CSV includes simulation time, true pose, world velocity, yaw
rate and actual ideal wheel rates. Metadata includes software/model/format
versions, Python/NumPy versions, integrator, dt, seed, duration, step count, and
the explicit flag `collision_enabled: false`.

These are inspectable foundation exports, not the later experiment/replay
manifest format. They do not yet include sensor streams, algorithm internals,
trial IDs, wall timestamps, configuration hashes, git revision, lifecycle events,
transactional writes or statistical summaries. Saving to an existing directory
replaces these named files; use separate directories to retain runs.

`SimulationResult` stores every sample in memory, so storage is O(steps).
Large runs and streaming telemetry are future work; no expensive sweep runner
or background job is created here.

## Visualization

`plot_trajectory` is an optional matplotlib consumer, using a headless Agg canvas.
It draws environment geometry, measured ground-truth trajectory, sampled body
heading arrows, the final circular footprint, and an orientation time series.
The plot labels the ideal model and disabled collision. It extends axes to show
out-of-bounds states rather than concealing them. It never edits simulation state.

The plotted unwrapped orientation reconstructs continuity from canonical heading
samples when rotation is less than pi radians per sample. For larger increments,
the plot displays explicitly labelled wrapped angles to avoid ambiguous unwrapping.
The final heading arrow is red. CSV always retains the actual wrapped heading. There is no interactive 3D
frontend, live replay, estimated trajectory, sensor visualization or covariance.

## Testing

Configuration tests cover malformed YAML, unsafe tags, duplicate keys, invalid
geometry, field typos, nonfinite values, invalid dt/steps/seeds, and round trips.
Simulation tests cover segment timing, pure spin, straight and curved motion,
stationarity, repeated runs, saving/reloading config and CSV, CLI failure reporting,
and generating a PNG from actual results. Tests never claim collision support.
