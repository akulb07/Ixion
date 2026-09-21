# Milestone 2 validation report

RoboForge 0.2.0 completes the authorized next step: physical robot descriptions,
named mounts, static circular-footprint contact queries, conservative swept
collision detection, and optional simulation stopping. The Milestone 1 report
remains historical; this report describes the current release.

## Architecture and changed files

- `config.py`, `robot.py`: validated masses/inertia, model type, mounting frames,
  and opt-in collision configuration. Existing input configurations remain valid.
- `physics/collision.py`: signed clearance, witness points, normals, per-contact
  penetration and conservative swept checks along exact/Euler motion.
- `simulation.py`: terminal collision events, fractional clock stops, requested
  commands, safe terminal states and retained executed motion segments.
- `trajectory.py`, `visualization.py`: reconstruct actual within-step motion for
  display; sparse arcs and multiple revolutions are not replaced by chords.
- `io.py`, `cli.py`: version 2 exports, contact/motion JSON, initial-footprint
  validation, and distinct completion/collision/error exit codes.
- Two declarative demos, collision/model/trajectory/integration tests, updated
  documentation, and a scientific-notation config round-trip regression fix.

## Verification

| Check | Result |
|---|---|
| Full source suite | **117 tests and 153 subtests passed**, no failures/skips |
| Full suite against isolated wheel installation | **117 tests and 153 subtests passed**, no failures/skips |
| Ruff lint and formatting | Passed |
| Package | `roboforge-0.2.0-py3-none-any.whl` built and installed |
| Installed CLI | Collision demo configuration validated |
| Original foundation demo | Still within 1e-12 per pose component; maximum discrepancy 5.5067e-14 |
| Collision demos | Thin wall and full loop both stop without penetration |
| Repeatability | States, collision events and motion segments equal on rerun |
| Visualization | Regenerated from recorded motion and visually inspected |

There are 44 additional test methods beyond the 73 in Milestone 1. Subtests are
reported separately, not counted as additional test functions. Coverage includes
all boundary directions, overlap, tangency, corner near misses, a full loop,
reverse/Euler motion, query limits, zero-fraction stops, serialization, mass
properties, mounting frames, and sparse trajectory reconstruction.

## Measured demo results

Both start and end of the originally requested timestep are collision-free; an
endpoint-only detector would miss these obstacles.

| Scenario | Analytical first contact (s) | Recorded safe stop (s) | Positive clearance at stop (m) |
|---|---:|---:|---:|
| Thin wall | 0.950000000000000 | 0.949999809265137 | 7.629394531e-7 |
| Full circular loop | 1.269659781241525 | 1.269659719429323 | 6.111285847e-8 |

Absolute time errors are 1.907348632e-7 s and 6.181220158e-8 s respectively.
The configured spatial tolerance is 1e-6 m. This is not a universal time-error
bound; grazing/tangent paths have different conditioning.

The wall's candidate is labelled `conservative_contact`: its midpoint is still
slightly clear, but the small interval is unresolved. The loop has a confirmed
sampled `contact`. Both stop at a prior certified-clear fraction, with terminal
wheel/body velocities zero. The two sweeps used 37 and 36 geometry queries; these
counts are diagnostics, not a performance benchmark.

## Reproduce

```sh
python -m pip install -e ".[dev,plot]"
python -m pytest
python examples/milestone_2/run.py
python examples/01_differential_drive/run.py
```

Each collision demo exports configuration, metadata, trajectory CSV, contact
JSON, motion-segment JSON, and a PNG. The summary `validation.json` stores the
expected/actual values above. The source archive and wheel are separate release
artifacts; machine-readable test reports and hashes accompany the delivery.

The tested runtime is Windows/Python 3.12.14, NumPy 2.5.3, Pydantic 2.13.5,
PyYAML 6.0.3, matplotlib 3.11.2, pytest 9.1.1, Ruff 0.16.8, setuptools 84.0.0
and wheel 0.48.0. Dependency ranges are not a claim that every combination was
tested. Workspace tooling was copied into a normal inherited-access directory
after a Windows sandbox-account permission issue; no application permissions or
system-wide dependency installations were changed.

## Limitations and next step

Mass properties describe ideal geometry but no force/actuator dynamics are
integrated. The footprint is circular; obstacles are static circles or
axis-aligned rectangles. Stopping has no sliding, bounce or contact forces.
Conservative near misses may stop within the specified spatial tolerance, and
query-budget exhaustion fails explicitly. Results remain stored in memory.

Sensors, actuator imperfections, localization, control, planning and mapping
are not implemented. Next is the sensor interface and seeded encoder/IMU/LiDAR
models, preserving the strict separation of ground truth and measurements.
