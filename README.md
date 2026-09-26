# RoboForge

A robotics experimentation laboratory built around explicit equations, measured
results and reproducible runs. Release **0.17.0** adds an interactive React workspace
for configuring simulations, inspecting sensors and replaying saved experiments.

## Implemented capabilities

- Planar geometry, named SE(2) frames, exact/Euler differential-drive kinematics.
- Strict YAML/JSON configuration, robot descriptions and deterministic clocks.
- Static worlds, signed clearance and conservative swept-footprint collision stops.
- Seeded encoders, planar IMU and ray-cast LiDAR with dropout and latency.
- Wheel response lag, delay, gain mismatch, deadzone and speed/acceleration limits.
- Encoder-only PID with anti-windup and complete control telemetry.
- Encoder odometry, three-state EKF and optional associated-landmark corrections.
- A*, Dijkstra, RRT, RRT*, and encoder-guided Pure Pursuit navigation.
- Log-odds occupancy mapping from scans and capture-time pose estimates.
- ICP scan-to-map SLAM, anchored pose graphs, geometric loop verification and map rebuilding.
- Bounded seed/parameter sweeps, retained failures, metrics, paired comparisons and replay.
- Timed sensor/motor faults and slip that separates shaft rotation from ground motion.
- Versioned empty-room, corridor, maze and clutter planner benchmarks.
- Optional local API with bounded background runs, cancellation, persistence and recorded replay.
- Packaged offline workspace with world visualization, timeline, fault inspector and run history.

The current platform includes an interactive workspace, Python APIs, a local HTTP service, CLI tools, exported data and plots.
The navigation and SLAM laboratories use explicit integration loops; a unified
navigation scenario schema and research workflow integration are next. AI,
hardware adapters, dynamic obstacles and force/friction/contact dynamics remain
unimplemented. Model documents explain narrower algorithm assumptions and limits.

## Quick start

Use Python 3.12 or newer, from this source directory:

```sh
python -m venv .venv
# Windows PowerShell: .venv\Scripts\Activate.ps1
# Linux/macOS: source .venv/bin/activate
python -m pip install -e ".[dev,plot]"
python -m pytest
roboforge validate configs/foundation.yaml
roboforge simulate configs/sensors.yaml --output results/sensors --plot
```

Enable `simulation.collision.mode: stop` for collision stopping; the default is
`disabled` for compatibility with early configurations. Collision is a geometric
stop policy, not impact dynamics. Simulate returns 0 for completion, 3 for collision,
and 2 for input/numerical failure. Invalid sensor rays use null values, never NaN.

## Reproducible laboratories

For the optional local service, install `pip install -e ".[api]"` and run
`roboforge serve --output results/service`. Open http://127.0.0.1:8765/ for the
[interactive workspace](docs/workspace.md), or `/docs` for endpoint documentation. See [local API](docs/local-api.md) for
workflow, resource bounds, persistence, and current scope.

```sh
python examples/milestone_3/run.py   # seeded sensors
python examples/milestone_4/run.py   # actuator limits and response
python examples/milestone_5/run.py   # encoder PID
python examples/milestone_6/run.py   # odometry and calibration drift
python examples/milestone_7/run.py   # four path planners
python examples/milestone_8/run.py   # encoder-guided navigation
python examples/milestone_9/run.py   # encoder/gyro EKF and uncertainty ellipses
python examples/milestone_10/run.py  # estimated occupancy grid
python examples/milestone_11/run.py  # incremental scan-to-map SLAM
python examples/milestone_12/run.py  # verified loops and pose-graph correction
python examples/milestone_13/run.py  # paired-seed PID experiment
python examples/milestone_14/run.py  # timed faults and encoder-bias sweep
python examples/milestone_15/run.py  # standardized planner benchmarks
```

Each accepts `--output <directory>` and writes actual data, plots and validation
results. Milestones 1 and 2 retain analytical motion and collision demonstrations.
The benchmark demo deliberately retains sampling searches that exhaust their
budget; it never replaces failed trials with successful paths.

## Experiments and replay

```sh
roboforge experiment experiment.yaml --output results/runs --plot
roboforge replay results/runs/<experiment-id>/<trial-id> --time 1.25
roboforge benchmark --output results/benchmarks --seed 42 --iterations 500
```

Experiment YAML contains `base` (a RunConfig), `seeds`, and `axes` with `path` and
`values`. Paths such as `wheel_controller.left.ki` or `faults.0.magnitude` identify
existing fields. The Milestone 13 demo exports a complete runnable experiment.json.
Every invocation creates a unique run directory. Resolved configuration hashes,
seeds, software versions, runtime, individual metrics and failures are preserved.
Replay reconstructs exported states, motion segments and sensor delivery without
rerunning simulation. Experiment manifests detect accidental file corruption.

## Models and validation

See [checkpoint history](docs/progress.md) and
[release validation](docs/validation-milestone-17.md).
The current source and installed wheel are validated with pytest; the historical
`scripts/test.py` runs only the older unittest subset and is not the release gate.

| Area | Model documentation |
|---|---|
| Geometry and dynamics | [Frames](docs/coordinate_systems.md), [drive](docs/differential_drive.md), [robot](docs/robot_model.md), [collision](docs/collision.md) |
| Sensors and controls | [Sensors](docs/sensors.md), [actuators](docs/actuators.md), [PID](docs/control.md), [tracking](docs/tracking.md) |
| Estimation and mapping | [Odometry](docs/odometry.md), [EKF](docs/localization.md), [occupancy](docs/mapping.md) |
| Planning and SLAM | [Planning](docs/planning.md), [ICP](docs/slam.md), [pose graphs](docs/pose_graph.md) |
| Research tools | [Experiments/replay](docs/experiments.md), [faults](docs/faults.md), [benchmarks](docs/benchmarks.md) |

Same-runtime numerical repeatability is tested. Floating-point library versions
can affect cross-platform results. Covariance and benchmark figures are model-based
or measured results, not claims of universal accuracy or hardware performance.
Estimators consume measurements; only simulation, sensor adapters and evaluation
have ground-truth access. Preserve that boundary when adding the application UI.
