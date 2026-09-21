# Sensor models and measurement contracts

Sensors receive simulator-only motion/state through SensorSuite. Algorithms
receive immutable EncoderReading, ImuReading, or LidarReading records. Those
records contain no true pose or simulator handle. Each includes sensor/frame,
integer sequence, capture_time and delivery_time. Invalid channels use None.

Sampling instants are index/rate_hz; the simulator supplies interval start times
from its clock. A sample exactly on a segment boundary observes the preceding
segment (left limit). Sampling may be faster than simulation steps because the
known within-step kinematic model provides the pose. Latency is deterministic:
delivery_time=capture_time+latency. SensorSuite.deliver releases each reading
once, ordered by delivery time/name/sequence, only when due. Exports retain all
captures, including deliveries later than the run end; consumers must honor time.

## Noise

Each sensor/channel uses named PCG64 generators seeded by SHA-256 of the run seed
and channel name (scheme version 1). White noise, bias walk and dropout have
separate streams. Sensor order or adding another sensor cannot change an existing
sensor's draws. y=x+b+walk+N(0,sigma), with walk increments N(0,q*sqrt(dt)). Dropout
returns None while the latent bias continues evolving. Zero parameters disable
their effects; no global random generator is used.

## Encoders

Cumulative physical wheel angle is integrated from executed wheel rates, including
partial collision intervals. The measured angle is (1+scale_error)*angle plus
configured noise/bias. Ticks=round(angle*ticks_per_revolution/(2*pi)); Python's
nearest-even tie rule is used. Ideal quantization error is at most half a tick.
Noise/bias are in radians, bias_walk in rad/sqrt(s). Left/right dropouts are
independent. Output is cumulative signed counts, not odometry or true pose.

## Planar IMU

Gyro measures yaw rate. At base, acceleration is approximately (dv/dt,v*omega).
For mount offset (x,y), add (-alpha*y-omega²*x, alpha*x-omega²*y), then rotate by
negative mount yaw into sensor axes. dv/dt and alpha are differences between IMU
sampled velocities, so ideal velocity jumps are averaged over sample intervals.
This is a planar approximation: no gravity z channel, roll/pitch, vibration,
impact impulse, temperature model, or claim of high-fidelity IMU dynamics.
Gyro parameters use rad/s; acceleration parameters use m/s². Initial derivatives
are zero. Steady circular motion correctly measures centripetal acceleration.

## LiDAR

Rays use analytical slab intersections for rectangles and quadratic intersections
for circles, plus world walls. The nearest positive hit wins; origins inside a
solid report zero range. Rays span [-FOV/2,FOV/2) without a duplicate full-circle
endpoint; one ray points forward. Pose includes the configured mount transform.

Ranges outside the minimum/maximum measurement window or dropped rays are None.
No return within max_range is encoded as range=max_range, hit=False. A real
surface at max_range has hit=True. Gaussian noise affects actual hits; no-return
rays retain the range limit (and can drop out). Range noise/bias is in metres.
Per-ray bias walks advance once per scan, avoiding ray-count-dependent drift.

## Validation and limitations

Tests cover known ray distances, tangency/occlusion, encoder quantization,
centripetal acceleration, mount offsets, cadence, seed independence, white-noise
statistics, dropout, deterministic repeats and latency. The one-command
`python examples/milestone_3/run.py` saves actual measurements and a diagnostic plot.
Scans are instantaneous, with no rolling acquisition distortion or multi-return
physics. Scene geometry is static and primitive. Measurements are held in memory;
live feedback algorithms and hardware providers are later layers.
