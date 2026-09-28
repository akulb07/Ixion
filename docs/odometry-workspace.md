# Checking encoder drift

Open a saved run in Simulation and find **Localization · Encoder odometry**
below wheel telemetry. Pick an encoder stream and click **Analyze odometry**.
The solid green path is truth and the dashed copper path is the estimate.

This is an offline reconstruction using the existing EncoderOdometry class.
Cumulative tick differences give wheel rotations; the differential-drive model
integrates constant twist between complete measurements. The initial pose and
wheel dimensions come from the saved configuration. Ground truth is only read
after each estimate to score it, never to correct it.

Only readings delivered by the run's end are used. Estimates are evaluated at
capture time, not delivery time. An incomplete pair of counts is skipped; the
next complete pair integrates the whole gap. A complete sample at time zero is
required so a later first sample doesn't silently receive the initial pose.

Metrics are position RMSE, mean absolute position error, maximum and final
position error, and signed wrapped final heading error. Each estimate has equal
weight; these are sample-based metrics, not a time integral. The final error is
at the last usable capture time, which can precede the run end because of latency
or dropout. The UI reports both times. This is encoder dead reckoning, not EKF
fusion or SLAM.

`GET /api/runs/{id}/odometry?sensor=encoders&max_points=1000` returns a JSON report.
The display/export sample limit is 2–2000; endpoints are preserved when sampling.
Metrics always use every estimate. Analysis is bounded to 100,000 delivered
encoder readings. It never starts another simulation or edits saved artifacts.
The report records the initial pose prior, sensor, run ID and algorithm name.

Try the wheel-slip preset to see the encoder estimate diverge from truth.
