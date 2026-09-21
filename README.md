# RoboForge

A robotics experimentation laboratory built around explicit equations, measured
results, and reproducible runs. This release completes **Milestone 2**: the tested
foundation plus physical robot descriptions and circular-footprint collision queries.

## What works

- Immutable planar vectors, poses, and named SE(2) transforms.
- Differential-drive forward/inverse kinematics; exact and Euler pose updates.
- Validated YAML/JSON configuration, robot state, and one integer-tick clock.
- Physical mass/inertia descriptions and configurable mounting frames.
- 2D environments with rectangles, circles, and world boundaries.
- Signed clearance, contact/penetration diagnostics, and swept straight/arc checks.
- Opt-in conservative collision stopping, including partial-timestep events.
- Prescribed wheel commands, deterministic trajectory export, and optional plots.
- Unit, integration, regression, and numerical convergence tests.

This is an ideal kinematic simulator. Enable `simulation.collision.mode: stop`
to stop before contact; the default remains `disabled` for existing configurations.
Stopping is a geometric policy, not physical impact response. Sensors, actuator dynamics, localization, planning, SLAM,
experiment sweeps, replay, a web frontend, AI, and hardware adapters are not yet
implemented. Static visualization is deliberately sufficient for this milestone.

## Quick start

Use Python 3.12 or newer. From this directory:

```sh
python -m venv .venv
# Windows PowerShell: .venv\Scripts\Activate.ps1
# Linux/macOS: source .venv/bin/activate
python -m pip install -e ".[dev,plot]"
python -m pytest
python examples/01_differential_drive/run.py
python examples/milestone_2/run.py
```

The demo validates its analytical endpoints and writes `results/foundation/`:
`config.json`, `metadata.json`, `trajectory.csv`, `trajectory.png`, and
`validation.json`. It starts at (1, 1), drives straight at 1 rad/s on both wheels
for 10 s, then spins with wheel rates -1/+1 rad/s for 10 s.

The Milestone 2 demo writes `results/milestone_2/{wall,loop}/` and a validation
summary. It detects a thin wall between clear endpoints and a circle encountered
during a full loop, stopping without penetration. Each result records uncertainty,
contact geometry, requested commands, actual states, and reconstructable motion segments.

```sh
roboforge simulate configs/collision_wall.yaml --output results/wall --plot
```

The CLI returns 0 for completion, 3 for a recorded collision stop, and 2 for
invalid configuration or numerical/query-budget failure. A collision is retained
as an inspectable run result. `roboforge validate` also rejects an initially
contacting footprint when collision stop mode is enabled.

## First simulation

```sh
roboforge validate configs/foundation.yaml
roboforge simulate configs/foundation.yaml --output results/my_run --plot
# Equivalent: python -m roboforge simulate ...
```

Edit the YAML's robot dimensions, environment, integration method, timestep, and
command segments. Each segment specifies integer `steps`, so its duration is
unambiguous: `steps * dt`. The example is declarative; the simulator contains no
special-case demo geometry. A rerun writes the requested output directory again.

```python
from roboforge.config import load_config
from roboforge.simulation import Simulator

config = load_config("configs/foundation.yaml")
result = Simulator(config).run()
print(result.states[-1].pose)
```

The independent math demonstration also covers left/right arcs, square closure,
vectors, and sensor-mount transforms:

```sh
python examples/milestone_1.py
```

## Architecture

One installable Python distribution contains small, separated modules. Numerical
geometry and robot kinematics depend only on shared validation and NumPy. The
simulator owns ground truth and simulation time; plotting and exports consume
results. Future algorithms will consume sensor readings and estimates through
interfaces, with no implicit access to simulator ground truth.

See [architecture](docs/architecture.md) for the complete target architecture,
minimum viable architecture, interfaces, dependencies, risks, and roadmap.
See [coordinate systems](docs/coordinate_systems.md),
[differential drive](docs/differential_drive.md),
[robot model](docs/robot_model.md), [collision model](docs/collision.md),
[configuration and simulation](docs/simulation.md), and
[testing](docs/testing.md) for the implemented APIs and assumptions.

The [Milestone 2 validation report](docs/validation-milestone-2.md) records exact
test counts, analytical comparisons and known limitations.

## First experiment and roadmap

The current demo is a deterministic model-validation run. The future experiment
engine will add run IDs, manifests, trials, parameter sweeps, aggregate metrics,
failure reports, and replay. Those capabilities are intentionally not claimed
by this release.

The next milestone should add the sensor interface and deterministic encoders,
IMU, and ray-cast LiDAR with measurement/ground-truth separation. Subsequent phases follow the
revised specification: actuators, PID, odometry, planning,
tracking, EKF, mapping, SLAM, experiment orchestration, faults, benchmarks,
advanced visualization, AI, and hardware integration. Each phase has its own
validation gate and requires explicit approval to start.

## Running tests without pytest

The suite uses `unittest.TestCase`, including seeded invariant checks. Pytest is
the standard runner, but every test is also discoverable with:

```sh
python scripts/test.py
```

NumPy, Pydantic, and PyYAML are still required. The plotting test skips only if
the optional matplotlib dependency is absent. See the delivered validation
report for the exact environment used for this build.

## Research philosophy

Separate ground truth from measurements. Record assumptions and configuration.
Compare against analytical expectations before trusting plots. Preserve failures
and individual metrics, and never invent benchmark results or scientific claims.
