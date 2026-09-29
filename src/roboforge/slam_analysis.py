"""Bounded scan-to-map inspection using saved encoder priors, with truth scoring only."""

import bisect
import math
from collections import Counter
from dataclasses import asdict

import numpy as np

from roboforge import __version__
from roboforge.core import wrap_angle
from roboforge.geometry import Pose2
from roboforge.mapping import GridConfig
from roboforge.odometry_analysis import analyze_odometry, pose_error_metrics
from roboforge.sensors.readings import LidarReading
from roboforge.service import ServiceError
from roboforge.slam import IcpConfig, IncrementalSlam, SlamConfig


def analyze_slam(replay, sensor, encoder, resolution=0.1, max_match_rmse=0.1):
    if not math.isfinite(resolution) or resolution < 0.05:
        raise ServiceError("SLAM resolution must be at least 0.05 m", 422)
    world = replay.config.environment
    columns, rows = math.ceil(world.width / resolution), math.ceil(world.height / resolution)
    if columns * rows > 10000:
        raise ServiceError("SLAM map exceeds 10,000 cells; increase resolution", 422)
    end = replay.states[-1].time
    scans = sorted(
        (
            r
            for r in replay.readings
            if isinstance(r, LidarReading) and r.sensor == sensor and r.delivery_time <= end
        ),
        key=lambda r: r.capture_time,
    )
    if not scans:
        raise ServiceError("no delivered scans for this LiDAR", 422)
    rays = sum(len(s.angles) for s in scans)
    # ICP computes nearest pairs twice per iteration; bound total distance evaluations.
    work = sum(min(len(s.angles), 300) * 300 * 25 * 2 for s in scans)
    if len(scans) > 200 or rays > 50000 or rays * (rows + columns) > 5000000 or work > 100000000:
        raise ServiceError(
            "SLAM analysis exceeds work budget; use a shorter or lower-rate run", 422
        )
    if any(
        b.capture_time <= a.capture_time or b.sequence <= a.sequence or b.frame != a.frame
        for a, b in zip(scans, scans[1:])
    ):
        raise ServiceError(
            "LiDAR stream has duplicate timestamps, invalid sequence or changed frame", 422
        )
    robot = replay.config.robot
    mounts = {
        "base": Pose2(),
        "left_wheel": Pose2(0, robot.wheel_separation / 2),
        "right_wheel": Pose2(0, -robot.wheel_separation / 2),
    }
    mounts.update({m.name: m.pose.to_pose() for m in robot.mounts})
    if scans[0].frame not in mounts:
        raise ServiceError("LiDAR frame has no configured mount", 422)
    baseline = analyze_odometry(replay, encoder, 100000)["samples"]
    times = [s["capture_time"] for s in baseline]
    samples, skipped, paired_odometry = [], [], []
    try:
        settings = SlamConfig(
            icp=IcpConfig(max_points=300, iterations=25), max_match_rmse=max_match_rmse
        )
        slam = IncrementalSlam(
            GridConfig(columns=columns, rows=rows, resolution=resolution),
            settings,
            mount=mounts[scans[0].frame],
            frame=scans[0].frame,
        )
        for scan in scans:
            index = bisect.bisect_left(times, scan.capture_time)
            candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
            nearest = min(candidates, key=lambda i: abs(times[i] - scan.capture_time))
            if abs(times[nearest] - scan.capture_time) > 1e-9:
                skipped.append(
                    {"capture_time": scan.capture_time, "reason": "missing_encoder_prior"}
                )
                continue
            prior = baseline[nearest]
            estimate = slam.update(scan, Pose2(**prior["estimate"]), prior_time=scan.capture_time)
            truth = replay.state_at(scan.capture_time).pose
            samples.append(
                {
                    "capture_time": scan.capture_time,
                    "estimate": asdict(estimate.pose),
                    "odometry": prior["estimate"],
                    "truth": asdict(truth),
                    "status": estimate.status,
                    "map_updated": estimate.map_updated,
                    "rejection_reasons": estimate.rejection_reasons,
                    "match": asdict(estimate.match) if estimate.match else None,
                    "position_error_m": math.hypot(
                        estimate.pose.x - truth.x, estimate.pose.y - truth.y
                    ),
                    "heading_error_rad": wrap_angle(estimate.pose.theta - truth.theta),
                }
            )
            paired_odometry.append(
                {
                    **prior,
                    "position_error_m": math.hypot(
                        prior["estimate"]["x"] - truth.x, prior["estimate"]["y"] - truth.y
                    ),
                    "heading_error_rad": wrap_angle(prior["estimate"]["theta"] - truth.theta),
                }
            )
        if not samples:
            raise ServiceError("no scans have matching encoder capture times", 422)
    except (ValueError, OverflowError, np.linalg.LinAlgError) as exc:
        raise ServiceError(f"invalid SLAM analysis: {exc}", 422) from exc
    return {
        "format_version": 1,
        "software_version": __version__,
        "algorithm": "incremental_scan_to_map_icp",
        "settings": {
            "sensor": sensor,
            "encoder": encoder,
            "slam": settings.model_dump(),
            "mount": asdict(mounts[scans[0].frame]),
            "initial_pose": robot.initial_pose.model_dump(),
        },
        "grid": slam.grid.config.model_dump(),
        "states": slam.grid.states().tolist(),
        "map_points": slam.points.tolist(),
        "samples": samples,
        "skipped_scans": skipped,
        "counts": dict(Counter(s["status"] for s in samples)),
        "rejection_counts": dict(Counter(r for s in samples for r in s["rejection_reasons"])),
        "metrics": pose_error_metrics(samples),
        "odometry_metrics": pose_error_metrics(paired_odometry),
        "run_end_time": end,
    }
