# EKF localization

State is (x,y,theta) in metres and radians. The prior is supplied explicitly.
PoseEKF.predict consumes distance and yaw increments with a 2x2 covariance.
For z=yaw/2, position advances by distance*sinc(z) at heading theta+z.
Analytic F=df/dpose and G=df/dincrement propagate P as F P Fᵀ + G Q Gᵀ.
The sinc and its derivative use stable series near zero. Tests independently
compare both Jacobians with central finite differences.

EncoderImuEKF derives wheel distances from complete cumulative encoder pairs.
Wheel increment variances are tick_metres²/6 + wheel_variance_per_m*abs(distance).
The 2x2 transform [[1/2,1/2],[-1/L,1/L]] yields translation/yaw and their full
cross covariance. A synchronized endpoint gyro supplies gyro_z*dt as an observed
yaw increment, with variance (gyro_stddev*dt)². A scalar Gaussian update conditions
that increment before nonlinear pose propagation. Innovation, S, normalized squared
innovation and gating outcome are logged. The default gyro gate is 25.

The endpoint gyro approximates the interval-average rate: assume constant motion
between consecutive captures. Inputs require matching encoder/IMU capture times;
callers must buffer by capture time and wait for both deliveries. Missing encoder
values produce no estimate; recovery bridges the complete gap using encoders only.
The filter never treats a single gyro endpoint as the integral over a dropout gap.
Gyro dropout uses encoder prediction. Quantization differences can be temporally
correlated; the covariance model approximates them as independent. Wheel slip,
unknown scale error and gyro bias require appropriate uncertainty or larger models.

Gyro fusion is dead reckoning: global position/heading remain unobservable and
initial-pose uncertainty is retained. This three-state model does not estimate
gyro bias. No accelerometer double-integration or invented absolute IMU pose.

An optional known, correctly associated point-landmark correction uses range
and wrapped base-frame bearing. Its analytic measurement Jacobian is tested.
The update uses a linear solve and Joseph covariance form; the default squared
Mahalanobis gate is 9.21034 (two-dimensional 99% chi-square threshold).
The caller supplies landmark identity, coordinates, measurement and positive
definite R. This primitive does not extract landmarks from LiDAR or solve data
association. Capture time must exactly match the current estimate; delayed
correction needs a future replay layer. Covariances are checked for symmetry,
finiteness and PSD within 1e-12 roundoff. Public estimates are immutable snapshots.

The demo compares truth, encoder odometry and encoder/gyro EKF under a wheelbase
calibration error, and plots estimated 95% position ellipses. Truth is used only
by the simulator and evaluator. A single successful trace does not prove calibrated
uncertainty; Monte Carlo consistency tests belong in the experiment phase.
