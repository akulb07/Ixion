# Configuration, state, time and simulation

## Configuration

RunConfig contains schema_version 1, name, seed, RobotConfig, Environment,
SimulationConfig, and nonempty WheelCommand segments. Milestone 2 adds fields
with defaults, so existing configurations retain their original motion. Old
releases reject unknown new fields. Schemas are frozen and forbid extra keys.
Numerical inputs must be finite; numeric strings and booleans are rejected.
Steps and seeds are strict integers; dimensions, masses, dt and tolerances are positive.

load_config uses a local safe YAML loader for YAML and JSON. Duplicate/non-string
keys fail with file/field context. A local resolver accepts unquoted exponent
notation such as 1e-6, needed for serialized JSON; quoted numeric strings fail.

Environment bounds are [0,width] x [0,height]. Rectangles use lower-left origins,
circles centre/radius. Obstacles must fit inside bounds; the initial robot centre
must be inside. With collision mode stop, CLI validate also checks the footprint.
Simulate records initial overlap as a terminal event. In disabled mode geometry
has no effect on motion. See [robot model](robot_model.md) and [collision](collision.md).

## State and recorded motion

RobotState contains immutable pose, wheel rates, body twist and time. World
vx/vy and omega are derived. Initial state is stationary. A normal end-of-step
sample records the ideal command applied during the preceding interval.

A terminal collision state instead has zero wheel/body velocity. Executed
KinematicMotion segments retain motion over each state interval, while the
collision event retains the requested command that was blocked. Stopping thus
does not erase the preceding path. There is one motion segment per state interval.
Actuators still respond immediately; these are ideal kinematics.

## Authoritative clock

SimulationClock computes time as tick*dt rather than repeated additions. The
simulator owns it. Normal steps define motion, optionally check the sweep,
advance the tick, integrate, and record. Command durations are integer step
counts times dt. Wall-clock time never affects motion.

For a collision interruption, time_at_fraction(f) returns (tick+f)*dt for f in
[0,1] inside the next tick. The run terminates at the certified safe fraction.
There can be one terminal partial interval. A zero-fraction stop updates the
last state's velocity without adding a duplicate timestamp; executed motion
segments remain intact. Robot.step rejects inconsistent timestamps.

Every Simulator.run call starts fresh. Current models make no random draws, so
the validated and recorded seed is unused. Repeated runs compare complete state,
collision-event and motion-segment sequences exactly on the tested runtime.

## Version 2 exports

- config.json: resolved robot/world/command and collision configuration.
- metadata.json: software/model/numerical versions, dt, seed, duration, state
  update count, completion/collision status, collision count and mass properties.
- trajectory.csv: recorded true poses, times, world velocities and wheel rates.
- collisions.json: stop/candidate times, unresolved interval, reason, requested
  command, witness points, penetration, candidate pose and query count.
- motion_segments.json: start times and executed ideal motion segments.
- trajectory.png: optional plot reconstructed from those motion segments.

The metadata steps field counts state updates, including a terminal partial
interval. It is distinct from planned full ticks. These exports are not a full
experiment/replay engine: no sensor streams, trial orchestration, atomic writes,
statistical summaries or replay controls. Saving in an existing directory replaces
these named files. Retain separate directories for comparisons.

Results keep all states and motion segments in memory: O(steps). Streaming is
future work. CLI exit codes: 0 completed; 3 recorded collision/conservative stop;
2 invalid input or numerical/query-budget failure.

## Visualization

sample_trajectory reconstructs executed motion with default maximum spacing
.05 m and .1 rad, using the existing integrator. These are display samples, not
new sensor measurements or telemetry; the plot labels them as resampled. Arcs
remain visible even with only two recorded states, and several revolutions within
one step can be unwrapped correctly. A sample budget fails explicitly rather than
silently replacing a complex path with a chord.

The matplotlib plot displays world geometry, resampled path, heading arrows,
final footprint and heading, candidate surface contact, and heading over time.
Axes expand to show out-of-world motion in disabled mode. There is no interactive
frontend, live replay, state estimate or covariance yet.

## Validation

Tests cover malformed/duplicate configs, scientific-notation round trips, model
immutability, timing, exact/Euler motion, repeatability, masses/frames, static and
swept contact, partial/zero-fraction stops, exports, CLI behavior and faithful
curve reconstruction. Milestone-specific reports record measured results.
