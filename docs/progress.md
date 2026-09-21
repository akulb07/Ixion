# Validated implementation checkpoints

The user has authorized continuing across milestones in order as usage permits.
Each checkpoint must pass tests and its demonstration before proceeding.

| Milestone | Implemented | Validation |
|---|---|---|
| 1 | Planar math, configuration, clock, ideal simulator | 73 tests, 110 subtests |
| 2 | Physical description, frames, static/swept collision | 117 tests, 153 subtests |
| 3 | Seeded encoders, IMU, LiDAR, timestamped readings | 131 tests, 157 subtests; 528-reading repeatable demo |

Current sensor model limitations and units are documented in sensors.md.
Next: actuator dynamics, then PID, odometry, planning, tracking, localization,
mapping and the remaining research/visualization layers in specification order.

Milestone 4: discrete wheel actuators, delay, asymmetry and limits; 144 tests and 157 subtests pass. Repeatable 450-step actuator demonstration. See actuators.md.

Milestone 5: encoder-only wheel PID, derivative filtering, conditional anti-windup, telemetry; 155 tests and 157 subtests pass. 399 feedback updates, final wheel errors below 0.00008 rad/s in the deterministic gain-mismatch demo. See control.md.
