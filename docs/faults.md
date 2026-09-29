# Timed fault injection

The workspace's Fault injection section edits all eight fault types without
hand-editing JSON. Add a fault, choose its effect and compatible target, then set
the start, optional end and magnitude. Command delay uses whole simulation steps
instead of magnitude. Validate faults checks the whole setup with the same schema
used when starting a run. Names must be unique, lowercase identifiers.

The Encoder dropout laboratory combines wheel PID feedback with a 75% encoder
packet-drop probability from 1 to 2 seconds. Use it to inspect missing measurements
and held controller output. Faults model degraded behavior; they do not necessarily
make a run's status become failed or cause a collision.

To reproduce an experiment, choose it in history, use its saved setup and retain
the seed and fault names. To make a baseline, save the faulty run first, choose
Clear faults for baseline, and run again. This changes the draft only. Compare
both saved runs or inspect their localization/controller analyses. Export the
setup to keep the schedule and seed together. Reproduction assumes the same
software version and numerical environment.

RunConfig.faults contains uniquely named, typed effects with target and half-open
interval [start,end). Missing end keeps a fault active. Configuration validates
sensor compatibility, wheel targets, magnitude ranges and delay budgets. Sensor
effects use capture time; motor/slip effects sample at the start of each simulation
tick and hold through that tick. Non-aligned motor fault boundaries are therefore
quantized to the next tick. faults.json records nominal scheduled activation and
deactivation events through the terminal run time, not extra physics steps.

| Kind | Magnitude and target | Effect |
|---|---|---|
| sensor_dropout | probability [0,1], sensor name | Encoder/IMU packet fields become None; LiDAR rays drop independently |
| encoder_scale | relative scale error >-1, encoder name | Cumulative reported counts multiply by 1+magnitude |
| gyro_bias | rad/s, IMU name | Additive gyro offset |
| gyro_drift | rad/s², IMU name | Offset grows with elapsed fault time |
| lidar_noise | range standard deviation in m, LiDAR name | Extra Gaussian hit-range noise; out-of-range values become invalid |
| wheel_slip | fraction lost [0,1], left/right/both | Ground motion is reduced while shaft encoders keep rotating |
| actuator_saturation | rad/s limit, left/right/both | Additional symmetric physical wheel-speed cap |
| actuator_delay | positive delay_steps, both | Additional delayed command samples; magnitude must stay zero |

Wheel slip is a prescribed kinematic traction-loss model, not force/friction
dynamics. RobotState.wheels records shaft rates; RobotState.twist and KinematicMotion
describe ground motion. Slipping motion segments separately retain encoder_wheels.
IMU and LiDAR see ground motion. CSV now records body_linear_m_s explicitly, and
replay preserves the distinction at recorded and interpolated times.

Changing cumulative encoder scale mid-run intentionally produces a readout jump;
it is not a smooth increment-scale model. Gyro drift resets when its fault ends.
Delay faults add when overlapping; expiration returns to the current command rather
than draining a queue. Saturation combines through the most restrictive active cap.
Slip fractions multiply. Sensor faults apply in sorted name order and use independent
SHA256-named PCG64 streams. Invalid readings remain invalid under later effects.
Additional LiDAR noise affects actual hit ranges, not maximum-range non-returns.

Wrong wheel radius/separation are estimator calibration choices already exposed
by the odometry/EKF APIs, rather than hidden mutations of simulation truth. The
experiment engine can sweep fault fields or whole fault lists; fault schedules,
seed, affected readings and motor telemetry remain in the exported run.
