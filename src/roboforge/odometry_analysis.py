"""Offline encoder reconstruction from a verified replay, separate from simulation."""

import math
from dataclasses import asdict

from roboforge.core import wrap_angle
from roboforge.geometry import Pose2
from roboforge.odometry import EncoderOdometry
from roboforge.sensors.readings import EncoderReading
from roboforge.service import ServiceError


def pose_error_metrics(samples):
    """Sample-weighted position errors and signed final heading error."""
    errors = [sample["position_error_m"] for sample in samples]
    scale = max(errors) or 1
    metrics = {
        "position_rmse_m": scale * math.sqrt(sum((e / scale) ** 2 for e in errors) / len(errors)),
        "position_mae_m": math.fsum(e / len(errors) for e in errors),
        "max_position_error_m": max(errors),
        "final_position_error_m": errors[-1],
        "final_heading_error_rad": samples[-1]["heading_error_rad"],
    }
    if not all(math.isfinite(v) for v in metrics.values()):
        raise ServiceError("analysis exceeds finite metric arithmetic", 422)
    return metrics


def analyze_odometry(replay, sensor: str, max_points: int = 1000):
    readings = sorted(
        (
            r
            for r in replay.readings
            if isinstance(r, EncoderReading)
            and r.sensor == sensor
            and r.delivery_time <= replay.states[-1].time
        ),
        key=lambda r: r.capture_time,
    )
    if not readings:
        raise ServiceError("no delivered readings for this encoder", 422)
    if len(readings) > 100000:
        raise ServiceError("odometry analysis exceeds 100,000 reading budget", 422)
    complete = [r for r in readings if r.left_ticks is not None and r.right_ticks is not None]
    if not complete or complete[0].capture_time != 0:
        raise ServiceError("analysis requires a complete encoder sample captured at time zero", 422)
    robot = replay.config.robot
    estimator = EncoderOdometry(
        robot.wheel_radius, robot.wheel_separation, Pose2(**robot.initial_pose.model_dump())
    )
    samples = []
    try:
        for reading in readings:
            estimate = estimator.update(reading)
            if estimate is None:
                continue
            truth = replay.state_at(estimate.time).pose
            position_error = math.hypot(estimate.pose.x - truth.x, estimate.pose.y - truth.y)
            heading_error = wrap_angle(estimate.pose.theta - truth.theta)
            samples.append(
                {
                    "capture_time": estimate.time,
                    "delivery_time": reading.delivery_time,
                    "estimate": asdict(estimate.pose),
                    "truth": asdict(truth),
                    "position_error_m": position_error,
                    "heading_error_rad": heading_error,
                }
            )
    except ValueError as exc:
        raise ServiceError(f"invalid encoder stream: {exc}", 422) from exc
    metrics = pose_error_metrics(samples)
    count = min(max_points, len(samples))
    indices = (
        [round(i * (len(samples) - 1) / (count - 1)) for i in range(count)] if count > 1 else [0]
    )
    return {
        "format_version": 1,
        "algorithm": "encoder_odometry",
        "sensor": sensor,
        "initial_pose_prior": robot.initial_pose.model_dump(),
        "total_estimates": len(samples),
        "dropped_readings": len(readings) - len(complete),
        "last_capture_time": samples[-1]["capture_time"],
        "run_end_time": replay.states[-1].time,
        "sampled": count < len(samples),
        "metrics": metrics,
        "samples": [samples[i] for i in indices],
    }
