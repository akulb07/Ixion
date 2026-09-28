# Encoder + IMU analysis

Open a saved run containing encoders and an IMU. Below mapping, **Localization ·
Encoder + IMU EKF** selects the streams and estimator noise assumptions. Analyze
EKF plots truth in green, encoder-only odometry in dashed copper, and the EKF in
blue. The shaded ellipse is the final position uncertainty.

This reuses EncoderImuEKF. Encoder increments give distance and yaw with their
cross covariance. A synchronized interval-end gyro updates the yaw increment,
then the nonlinear pose model propagates the pose covariance. The initial pose
comes from the saved configuration; initial standard deviations are 0.01 m in
x/y and 0.01 rad in heading. Truth is used for scoring only. There are no landmark
corrections or absolute position observations in this workspace yet.

Wheel variance per metre and gyro standard deviation describe filter assumptions,
not new noise injected into the saved run. Quantization variance is included by
the existing filter. The gyro innovation gate is 25. Rejected updates are counted
separately from intervals with no usable gyro. A dropout gap uses encoder-only
prediction on recovery: one endpoint gyro is not an average over a missing gap.

Only readings delivered by the run end are used. Offline reconstruction sorts by
capture time and requires exactly matching encoder/IMU capture timestamps; it
does not resample or interpolate. Gyro and encoder delivery order may differ.
Duplicate IMU times, non-increasing sequence numbers and changing frames are
rejected. A complete encoder sample at time zero is required.

Both estimators are scored at every usable encoder capture time. Metrics use all
samples even when the displayed path is sampled. The final capture time can
precede run end. Position RMSE/MAE are sample weighted; heading error is signed
and wrapped. A gyro has no absolute heading/position fix, so improved performance
is not guaranteed.

The position ellipse uses the covariance's x/y marginal and the 95% contour of
an assumed two-dimensional Gaussian (squared Mahalanobis radius 5.9914645471).
It is model uncertainty, not a measured guarantee of coverage. Bias and model
mismatch can put truth outside the ellipse. Full sample covariance matrices are
included in the JSON report along with settings, input run ID, algorithm and
software version.

`GET /api/runs/{id}/ekf?encoder=encoders&imu=imu&wheel_variance=0.001&gyro_stddev=0.01`
returns the report. At most 20,000 delivered encoder and 50,000 delivered IMU
readings are analyzed. `max_points` (2–2000, default 1000) limits returned samples
while preserving both endpoints. This never starts a new simulation or changes
the run's recorded navigation.
