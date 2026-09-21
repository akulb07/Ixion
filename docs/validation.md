# Milestone 1 validation report

Validated on 2026-09-21 against the revised foundation specification (sections
98–103). No later milestone was implemented.

## Architecture and files delivered

- `core`, `geometry`, `robotics`: immutable vectors/poses, named SE(2) transforms,
  finite-value validation, forward/inverse drive kinematics, exact/Euler integration.
- `config.py`: frozen Pydantic schemas and safe YAML/JSON loading.
- `robot.py`, `simulation.py`: ground-truth state, ideal robot, one integer-tick
  simulation clock, deterministic prescribed-command runner.
- `io.py`, `visualization.py`, `cli.py`: CSV/JSON exports, optional matplotlib
  trajectory/orientation plot, `validate` and `simulate` commands.
- `configs/foundation.yaml`, both example entry points, and unit/integration/
  regression/numerical test directories.
- Architecture, coordinate, kinematic, simulation, testing and contribution docs.

The source checkout is accompanied by a wheel, source ZIP, demo data/plot,
dependency snapshot, and machine-readable pytest reports in the delivered outputs.

## Checks performed

| Check | Result |
|---|---|
| Complete source test suite | 73 tests passed; 110 subtests passed; no failures or skips |
| Complete suite against installed wheel | 73 tests passed; 110 subtests passed; no failures or skips |
| Ruff lint and formatting | Passed |
| Wheel construction | `roboforge-0.1.0-py3-none-any.whl` built successfully |
| Installed module | Imported from isolated installation, outside source checkout |
| Installed CLI | Foundation configuration validated successfully |
| Foundation simulation | 2,000 steps, dt=.01 s, duration 20 s |
| Deterministic rerun | Complete result equality passed |
| Mathematical demonstration | Straight, spin, left/right arcs, stationary, square and frame chain verified |
| Visualization | PNG rendered from actual states and visually inspected |

An early environment lacked pytest. After installing workspace-local tooling,
the final checks used real pytest, including the optional matplotlib test.
No unrun or skipped tests are counted as passes. Subtests are reported separately
from test methods; they are not 110 additional independent test functions.

## Numerical demo

Robot wheel radius=.05 m, separation=.30 m, initial pose=(1,1,0).

| State | Expected | Actual |
|---|---|---|
| Straight, after 10 s with wheels (1,1) rad/s | (1.5, 1, 0) | (1.499999999999945, 1, 0) |
| Spin, after another 10 s with wheels (-1,1) rad/s | (1.5, 1, -2.949851973846253) | (1.499999999999945, 1, -2.949851973846203) |

Coordinates are metres; heading is radians in [-pi,pi). The spin accumulates
10/3 radians and does not translate. Maximum absolute discrepancy across the
displayed pose components is **5.5067062021407764e-14**, compared with an absolute
assertion tolerance of **1e-12 per component**. Position and angular quantities
have distinct units; this maximum is a validation diagnostic, not a physical
combined-error metric.

The plot shows actual straight travel, the subsequent in-place orientation
changes, the final footprint/heading, passive obstacles, and a heading time series.

## Tested environment

- Windows, Python 3.12.14.
- NumPy 2.5.3; Pydantic 2.13.5; PyYAML 6.0.3.
- matplotlib 3.11.2; pytest 9.1.1; Ruff 0.16.8.
- setuptools 84.0.0; wheel 0.48.0 for building.

See the dependency snapshot for installed transitive versions. The package's
version ranges are not a claim that every combination has been tested.
The model contains no stochastic components; the stored seed is reserved for
later models and does not influence current trajectories.

## Known limitations

This is an ideal planar kinematic model. Obstacle/boundary contact, swept
collision, physical dynamics, actuator imperfections, sensor readings, odometry,
EKF, PID, planning, mapping, SLAM, experiment sweeps, replay, live/3D frontend,
AI, ROS 2 and hardware are absent. Environment geometry is passive and initial
footprint overlap is not checked. All samples are stored in memory. There is no
claim of real-world model fidelity or cross-platform bitwise reproducibility.

## Next milestone

Formalize physical robot configuration and environment query interfaces, then
add circular-footprint collision detection with contact/clearance, overlap,
tangency, boundaries and swept-motion tests. Continue through the revised phase
order only after explicit approval. Work stops at this validation gate.
