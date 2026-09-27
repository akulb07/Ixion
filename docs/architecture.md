# Architecture and implementation boundary

Current release: 0.15.0. EKF, occupancy mapping, incremental and batch SLAM, experiments, replay, timed faults and benchmarks are implemented. See the current README and per-model documents; the foundation sections below are historical rationale. Historical Milestones 1–2 sections below retain their
original design rationale. Current scheduling is delivered encoder readings ->
optional wheel PID -> actuator response -> swept kinematics -> sensor captures ->
next tick. Sensor noise uses named independent PCG64 streams. EncoderOdometry
consumes measurement contracts only. Known-map planners use an explicit
Environment; PurePursuit accepts estimated poses only. The navigation laboratory
joins them in a separate bounded loop. See progress.md for current validation.

## Specification analysis

The revised specification's sections 98–103 defined Milestone 1. They
expand the earlier math-only milestone with configuration, robot state, a clock,
an environment, a simple simulator, and visualization. That revised boundary
takes precedence. The long-term platform remains a research system; building
all its algorithm layers now would prevent independent validation.

Milestone 2 added physical robot descriptions and circular-footprint collision
queries, with optional conservative simulation stopping. Sensors and force
dynamics were left for later phases; the original validated equations were
retained.

The first implementation must prove the chain from configuration to numerical
model to trajectory to measured analytical error. It must establish interfaces
that preserve ground truth, measurement, and estimate as different concepts.

## Final target architecture

| Layer | Responsibility | Allowed information |
|---|---|---|
| Geometry/core | Units, transforms, typed values, numerical validation | Pure values |
| Robot | Robot description, kinematics, immutable state | Robot parameters |
| Physics/actuators | Motion, limits, delay, friction, contact response | Ground truth + commands |
| Simulation | Clock, deterministic scheduling, lifecycle | World, physical state, sensor providers |
| Sensors | Sampling, noise, dropout, latency | Read-only ground truth through simulation adapter |
| Estimation/localization | Infer pose and uncertainty | Timestamped measurements + configured models |
| Mapping | Build estimated maps | Estimated pose + observations |
| Planning | Find paths and report search diagnostics | Explicit planning map + start/goal |
| Control | Convert goals/paths into bounded commands | Estimates + path/target |
| Experiments/research | Trials, sweeps, metrics, comparison, failure reports | Configurations + logged outputs |
| IO/adapters | Versioned storage, imported logs, future ROS 2 | Typed records at boundaries |
| Visualization/API | Inspection, plots, interaction, replay display | Serialized snapshots and telemetry |

The simulator may provide ground truth to evaluation and visualization. An
estimator must never receive a simulator or world-state handle. A special
ground-truth estimator, if added, must be an explicit experimental choice.
An ideal sensor still emits a `SensorReading`, not a `RobotState`.

The backend and CLI invoke the same Python API. A future frontend issues commands
and displays snapshots; it does not duplicate robotics equations in TypeScript.
Rendering can interpolate presentation positions without changing logged physics.

## Minimum viable architecture, implemented now

```text
YAML/JSON -> RunConfig -> Simulator -> DifferentialDriveRobot -> kinematics
                              |                |
                       SimulationClock     RobotState
                              |                |
                              +-> SimulationResult -> CSV / JSON / matplotlib

Environment -> CollisionWorld -> signed clearance / swept interval checks
                                   |
Simulator -> KinematicMotion -------+-> optional terminal collision event
```

`geometry` and `robotics` are pure numerical modules. `config` is the validated
boundary. `Simulator.run()` constructs fresh runtime state on every call.
`SimulationClock` supplies all sample timestamps. Immutable results can be
exported or plotted after execution. A collision can interrupt a tick at a clock-
derived fractional time. Results retain executed motion segments, allowing plots
to reconstruct curved paths between sparse state samples. No API server, database, asynchronous event
bus, plugin registry, or frontend is needed to validate this foundation.

## Repository structure

```text
roboforge/
  pyproject.toml
  src/roboforge/
    core/                  scalar validation and angle conventions
    geometry/              Vector2, Pose2, Transform2
    robotics/              differential-drive mathematical model
    physics/               signed clearance and conservative swept collision
    config.py              schemas and YAML/JSON parsing
    robot.py               ground-truth state and ideal robot
    simulation.py          clock, runner, result
    trajectory.py          reconstruction from recorded ideal motion segments
    io.py                  foundation exports
    visualization.py       optional static plotting
    cli.py                 validate/simulate entry points
  configs/foundation.yaml
  examples/
    milestone_1.py          mathematical demonstration
    01_differential_drive/run.py
    milestone_2/run.py
  tests/{unit,integration,regression,numerical}/
  docs/
  scripts/test.py
```

This is one distribution with a `src` layout rather than many separately built
`packages/*` distributions. That avoids dependency/version coordination overhead
while retaining import boundaries. Each module can become a directory when it
has multiple implementations. Do not create empty packages to imply features.

Later add `sensors`, `actuators`, `estimation`, `localization`,
`mapping`, `planning`, `control`, `experiments`, and `metrics` under this namespace.
Split distributions only if independent release/deployment requirements emerge.
Add `apps/backend` and `apps/frontend` when the service/UI phase begins; the thin
CLI already lives next to the Python API. Create datasets/notebooks/models only
when there are real artifacts to store there.

## Interfaces

### Current public API

| API | Input | Output / contract |
|---|---|---|
| `Pose2(x, y, theta)` | Finite position and heading | Immutable planar pose (the specification's Pose2D) |
| `Transform2(target, source, translation, theta)` | Named frames and SE(2) values | `apply_point`, `apply_vector`, `apply_pose`, `inverse`, `@` |
| `DifferentialDrive.forward(wheels)` | `WheelSpeeds` in rad/s | `BodyTwist2` in m/s and rad/s |
| `DifferentialDrive.inverse(twist)` | Nonholonomic body twist | `WheelSpeeds` |
| `DifferentialDrive.integrate(...)` | Pose, wheel rates, dt, method | New pose; no mutation |
| `load_config(path)` | YAML or JSON | Frozen `RunConfig`; actionable validation errors |
| `DifferentialDriveRobot.step(...)` | State, commands, dt, clock timestamp | New ground-truth `RobotState` |
| `Simulator.run()` | Constructor-supplied `RunConfig` | Immutable `SimulationResult` |
| `CollisionWorld.query(centre,radius)` | Circular footprint | Signed clearance, contacts, surface witnesses/normals |
| `CollisionWorld.sweep(motion,radius)` | Exact/Euler kinematic segment | Certified clear or earliest unresolved/contact interval |
| `DifferentialDriveRobot.frame_transforms(pose)` | World-from-base pose | World-from-base/wheels/configured mounts |
| `sample_trajectory(result)` | Recorded ideal motion segments | Resampled display poses, distinct from telemetry |
| `save_result` / `plot_trajectory` | Result + path | Exported artifacts |

### Future contracts (design only, not placeholder implementations)

- `SensorProvider.read(until_time) -> sequence[SensorReading]`: transport-neutral,
  with capture time, delivery time, frame, sequence ID, validity, and units.
  Simulated providers privately call observation models on world snapshots;
  hardware providers return the same public reading type.
- `ActuatorSink.command(ActuatorCommand)`: explicitly timestamped command and
  requested units. Applied commands and delayed/saturated responses are logged
  separately from requested commands.
- `Estimator.update(readings) -> StateEstimate`: typed estimate, frame, covariance,
  innovation and gain diagnostics. No ground-truth argument.
- `Planner.plan(map, start, goal, parameters) -> PlanningResult`: path, success,
  failure reason, explored nodes and timings; map provenance is explicit.
- `Controller.compute(estimate, reference, dt) -> ControlResult`: command plus
  target/error/internal contribution diagnostics.
- `Mapper.update(estimate, scan) -> MapUpdate`: estimated occupancy and log odds,
  separate from the world geometry.
- `ExperimentRunner.run(config) -> ExperimentRun`: orchestrates seeded trials;
  retains failures and full resolved configuration.
- `Metric.evaluate(telemetry) -> Metric`: value, unit, sample count, provenance,
  missing-data policy; paired comparisons retain individual metrics.

Strong schemas for sensors, faults, trajectories, experiments, results, metrics,
and metadata will be introduced when these contracts have concrete consumers.
`Pose2` and `BodyTwist2` intentionally state their planar/nonholonomic semantics;
a future generic 3D twist must not silently reuse the constrained drive type.

## Mathematical dependencies

SE(2), trigonometry, and float64 linear algebra support the current foundation.
Differential-drive integration depends on geometry; it does not depend on world
geometry or visualization. Later dependencies must be introduced in this order:

1. Computational geometry -> distance, intersection, ray casting, collision.
2. Actuator differential equations + integrators -> PID dynamics experiments.
3. Sensor error distributions + timing -> measurement models and odometry.
4. Graph search + collision inflation -> grid and sampling planners.
5. Path geometry -> Pure Pursuit and tracking errors.
6. Jacobians + covariance propagation + stable linear solves -> EKF.
7. Inverse sensor models + log odds -> mapping; then scan matching/SLAM.
8. Statistical estimators + trial design -> research comparisons and intervals.

Document analytic Jacobians and test against finite differences when EKF arrives.
Use linear solves rather than explicit inverses; validate symmetry and positive
semidefiniteness of covariance. Do not assume an IMU gyro alone observes absolute
position or heading drift.

## Technology choices

- **Python 3.12+**: one accessible scientific implementation for CLI, tests, and
  future backend, with type hints and immutable data values.
- **NumPy**: float64 matrices and scientific array interoperability. Tiny scalar
  kinematic formulas use `math`; no custom general-purpose matrix library.
- **Pydantic 2 + PyYAML**: strict boundary validation and human-readable configs;
  reject unknown fields, nonfinite values, numeric strings, and duplicate keys.
- **pytest**: complete test runner. Standard-library test cases also allow an
  alternate runner without changing assertions. Seeded invariant tests need no
  additional property-testing dependency at this stage.
- **matplotlib, optional**: reproducible static plots with no browser or GPU.
- **SciPy, later**: add when differential equations, optimization or sparse solves
  need it. It is not needed for the exact planar solution.
- **React/TypeScript + Three.js, later**: interactive engineering visualization.
  FastAPI is the preferred thin service boundary when a service becomes useful.
  ROS 2, CUDA, AI services, and a desktop runtime are not core dependencies.

## Reproducibility and future scheduling

Sensor-free prescribed-motion runs contain no random draws. The seed is validated and
recorded, with an empty list of stochastic components in metadata. Changing it
does not change this ideal model. Same inputs on the same software/runtime yield
identical numerical state sequences. Cross-platform comparisons use stated
tolerances because libm and floating-point evaluation can differ.

Each sensor noise component now receives its own named RNG stream derived from
the trial seed. Persist the stream allocation scheme/version; one algorithm's
random draws must not perturb another algorithm's sensor noise. Comparisons use
paired initial conditions and exogenous seeds, while acknowledging that different
closed-loop trajectories generate different sensor observations.

Use one integer-tick clock; schedule future sensor sample and delivery events
with deterministic tie-breaking. Fix and document the order of actuator update,
physics, sensing, delivery, estimation, control, and telemetry. Measure wall-clock
runtime separately and never use it to alter the simulated dynamics. Replay reads
recorded state/events without rerunning stochastic logic. Rerun is a separate action.

## Risks and architectural mistakes to avoid

| Risk | Required treatment |
|---|---|
| Reversed frames, sign or units | Named transforms, SI units, analytical examples and matrix checks |
| Ground truth leaks | Distinct state/reading/estimate types; no world handle for estimators |
| Near-straight integration cancellation | Stable sinc formulation; tests at tiny yaw rates |
| Timer drift and latency ambiguity | Integer ticks; separate capture/delivery timestamps |
| Fake physical realism | Label ideal commands, descriptive mass properties and geometric stop policy |
| Numerical plots hiding failures | Export actual data; never substitute successful trajectories |
| Euler accuracy assumed exact | Demonstrate first-order convergence and retain integrator metadata |
| Tunnelling through obstacles | Add swept-footprint tests before claiming collision correctness |
| Unobservable localization | State measurement assumptions; do not fabricate absolute IMU pose |
| Shared random streams | Named independent generators and deterministic scheduling |
| Reproducibility lost to revisions | Version resolved config, models, algorithms, schemas and dependencies |
| Unbounded result memory | Current O(steps) storage is explicit; later stream telemetry to disk |
| Premature plugin abstractions | Introduce protocols when a second implementation needs them |
| Composite scores masking tradeoffs | Preserve metrics and failure counts; require explicit objectives |

## Validation gates and next milestone

The current tests cover deterministic simulation, exact references, Euler
convergence, transforms and inverses, immutable state, config errors, export/reload,
CLI behavior and a real plot. No tests imply navigation or sensor functionality.

Milestone 2 adds analytical contact/clearance tests, boundary and overlap cases,
thin-wall and full-loop sweeps, numerical tolerance/query-budget checks, mass and
frame tests, and reproducible collision event/export/CLI tests. Sweeps follow the
actual kinematic path, never just its endpoint chord. Conservative near-contact
results are explicitly distinguished from confirmed sampled collisions.

Sensors, actuators, PID, odometry, planning and tracking are implemented within
their documented boundaries. Next add and validate EKF localization.
