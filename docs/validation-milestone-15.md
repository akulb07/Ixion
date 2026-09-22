# Release 0.15.0 validation

Source suite: **247 tests and 157 subtests passed**. Tests cover analytical motion,
sensor timing/noise, swept collision, control/actuators, odometry, planners,
navigation, EKF/landmark/graph Jacobians, covariance PSD, mapping boundaries,
ICP failures, loop verification, experiments, fault semantics and recorded replay.
Ruff lint and formatting checks are part of the release validation.

Tested runtime: Python 3.12.14, NumPy 2.5.3, Pydantic 2.13.5, PyYAML 6.0.3,
pytest 9.1.1, matplotlib 3.11.2 and Ruff 0.16.8. These are the observed versions,
not evidence for every version allowed by dependency ranges.

| Laboratory | Measured result |
|---|---|
| EKF calibration error | Endpoint error: odometry 0.4020 m; encoder/gyro EKF 0.001015 m |
| Occupancy | 51 scans; 20,915 free, 646 occupied, 7,339 unknown/uncertain cells |
| Incremental SLAM | 60 matches, zero rejections in this trace; endpoint 0.3402 m to 0.02415 m |
| Pose graph | Seven accepted loops; converged; endpoint 0.3409 m to 0.01228 m |
| PID experiment | 12 completed trials; paired mean RMSE change -0.5671 rad/s for both integral gains enabled |
| Fault sweep | 10% encoder scale error produces 0.1800 m final odometry error over 1.8 m travel |
| Planner benchmark | 16 trials; 14 paths verified; maze RRT/RRT* exhausted 300 iterations |

These are individual demonstrations under saved configurations, not broad
statistical claims. Seeded repeatability is checked for sensor/fault/estimator and
incremental-SLAM traces. Optimizer and graph regression tests are deterministic.
Wall-clock runtimes and generated UUIDs are intentionally not reproducible values.

Release artifacts include an installable wheel, a source archive, SHA256 hashes
and separate installed-wheel test evidence under validation-milestone-15 outside
the repository. The advanced application UI, online scenario orchestration, AI and
hardware are still future work. SLAM is bounded static-world laboratory code.
