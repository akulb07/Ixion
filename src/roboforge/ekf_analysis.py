"""Offline encoder/gyro fusion with paired odometry scoring and model uncertainty."""

import math
from dataclasses import asdict

import numpy as np

from roboforge import __version__
from roboforge.core import wrap_angle
from roboforge.localization import EncoderImuEKF
from roboforge.odometry_analysis import analyze_odometry, pose_error_metrics
from roboforge.sensors.readings import EncoderReading, ImuReading
from roboforge.service import ServiceError


def analyze_ekf(replay, encoder, imu, wheel_variance=0.001, gyro_stddev=0.01, max_points=1000):
    end = replay.states[-1].time
    encoders = sorted(
        (
            r
            for r in replay.readings
            if isinstance(r, EncoderReading) and r.sensor == encoder and r.delivery_time <= end
        ),
        key=lambda r: r.capture_time,
    )
    gyros = sorted(
        (
            r
            for r in replay.readings
            if isinstance(r, ImuReading) and r.sensor == imu and r.delivery_time <= end
        ),
        key=lambda r: r.capture_time,
    )
    if len(encoders) > 20000 or len(gyros) > 50000:
        raise ServiceError("EKF analysis exceeds 20,000 encoder or 50,000 IMU readings", 422)
    if not gyros:
        raise ServiceError("no delivered readings for this IMU", 422)
    if any(
        b.capture_time <= a.capture_time or b.sequence <= a.sequence or b.frame != a.frame
        for a, b in zip(gyros, gyros[1:])
    ):
        raise ServiceError(
            "IMU stream has duplicate timestamps, invalid sequence or changed frame", 422
        )
    # The baseline validates the initial encoder sample and preserves the full common time grid.
    baseline = analyze_odometry(replay, encoder, 20000)
    by_time = {r.capture_time: r for r in gyros}
    robot = replay.config.robot
    initial_covariance = np.diag([0.01**2, 0.01**2, 0.01**2])
    try:
        estimator = EncoderImuEKF(
            robot.wheel_radius,
            robot.wheel_separation,
            robot.initial_pose.to_pose(),
            initial_covariance,
            wheel_variance_per_m=wheel_variance,
            gyro_stddev=gyro_stddev,
        )
        samples = []
        for reading in encoders:
            estimate = estimator.update(reading, by_time.get(reading.capture_time))
            if estimate is None:
                continue
            reference = baseline["samples"][len(samples)]
            truth = reference["truth"]
            samples.append(
                {
                    "capture_time": estimate.time,
                    "estimate": asdict(estimate.pose),
                    "odometry": reference["estimate"],
                    "truth": truth,
                    "covariance": estimate.covariance,
                    "position_error_m": math.hypot(
                        estimate.pose.x - truth["x"], estimate.pose.y - truth["y"]
                    ),
                    "heading_error_rad": wrap_angle(estimate.pose.theta - truth["theta"]),
                }
            )
        metrics = pose_error_metrics(samples)
        final = samples[-1]
        eigenvalues, vectors = np.linalg.eigh(np.array(final["covariance"])[:2, :2])
        # 95% contour of the assumed two-dimensional Gaussian position marginal.
        radii = np.sqrt(np.maximum(eigenvalues, 0) * 5.991464547107979)
        ellipse = []
        for angle in np.linspace(0, math.tau, 49):
            offset = vectors @ (radii * [math.cos(angle), math.sin(angle)])
            ellipse.append(
                {
                    "x": float(final["estimate"]["x"] + offset[0]),
                    "y": float(final["estimate"]["y"] + offset[1]),
                }
            )
        if not all(math.isfinite(v) for point in ellipse for v in point.values()):
            raise ValueError("uncertainty ellipse exceeds finite arithmetic")
    except (ValueError, OverflowError, np.linalg.LinAlgError) as exc:
        raise ServiceError(f"invalid EKF analysis: {exc}", 422) from exc
    accepted = sum(i.accepted for i in estimator.innovations)
    count = min(max_points, len(samples))
    indices = (
        [round(i * (len(samples) - 1) / (count - 1)) for i in range(count)] if count > 1 else [0]
    )
    return {
        "format_version": 1,
        "software_version": __version__,
        "algorithm": "encoder_imu_ekf",
        "settings": {
            "encoder": encoder,
            "imu": imu,
            "wheel_variance_per_m": wheel_variance,
            "gyro_stddev_rad_s": gyro_stddev,
            "gyro_gate": estimator.gate,
            "initial_pose": robot.initial_pose.model_dump(),
            "initial_covariance": initial_covariance.tolist(),
        },
        "metrics": metrics,
        "odometry_metrics": baseline["metrics"],
        "total_estimates": len(samples),
        "sampled": count < len(samples),
        "gyro_accepted": accepted,
        "gyro_rejected": len(estimator.innovations) - accepted,
        "encoder_only_intervals": len(samples) - 1 - len(estimator.innovations),
        "dropped_encoder_readings": baseline["dropped_readings"],
        "last_capture_time": final["capture_time"],
        "run_end_time": end,
        "final_position_ellipse95": ellipse,
        "samples": [samples[i] for i in indices],
    }
