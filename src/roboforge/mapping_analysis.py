"""Bounded offline mapping from delivered LiDAR with an explicit pose source."""

import bisect
import math

from roboforge.geometry import Pose2
from roboforge.mapping import GridConfig, OccupancyGrid
from roboforge.odometry_analysis import analyze_odometry
from roboforge.sensors.readings import LidarReading
from roboforge.service import ServiceError


def analyze_map(replay, sensor, pose_source, encoder, resolution):
    if pose_source not in {"encoder", "truth"}:
        raise ServiceError("unknown map pose source", 422)
    if not math.isfinite(resolution) or resolution < 0.05:
        raise ServiceError("map resolution must be at least 0.05 m", 422)
    world = replay.config.environment
    columns, rows = math.ceil(world.width / resolution), math.ceil(world.height / resolution)
    if columns * rows > 10000:
        raise ServiceError("map exceeds 10,000 cells; increase resolution", 422)
    scans = sorted(
        (
            r
            for r in replay.readings
            if isinstance(r, LidarReading)
            and r.sensor == sensor
            and r.delivery_time <= replay.states[-1].time
        ),
        key=lambda r: r.capture_time,
    )
    if not scans:
        raise ServiceError("no delivered scans for this LiDAR", 422)
    rays = sum(len(scan.angles) for scan in scans)
    if rays > 50000 or rays * (rows + columns) > 5000000:
        raise ServiceError(
            "mapping exceeds ray traversal budget; use a shorter or lower-rate run", 422
        )
    mounts = {
        "base": Pose2(),
        "left_wheel": Pose2(0, replay.config.robot.wheel_separation / 2),
        "right_wheel": Pose2(0, -replay.config.robot.wheel_separation / 2),
    }
    mounts.update({mount.name: mount.pose.to_pose() for mount in replay.config.robot.mounts})
    estimates = (
        analyze_odometry(replay, encoder, 100000)["samples"] if pose_source == "encoder" else []
    )
    times = [s["capture_time"] for s in estimates]
    grid = OccupancyGrid(GridConfig(columns=columns, rows=rows, resolution=resolution))
    used, skipped, valid_rays = 0, 0, 0
    try:
        for scan in scans:
            if scan.frame not in mounts:
                raise ValueError("LiDAR frame has no configured mount")
            if pose_source == "truth":
                pose = replay.state_at(scan.capture_time).pose
            else:
                index = bisect.bisect_left(times, scan.capture_time)
                candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
                match = min(candidates, key=lambda i: abs(times[i] - scan.capture_time))
                if abs(times[match] - scan.capture_time) > 1e-9:
                    skipped += 1
                    continue
                pose = Pose2(**estimates[match]["estimate"])
            update = grid.update(
                scan, pose, pose_time=scan.capture_time, mount=mounts[scan.frame], frame=scan.frame
            )
            used += 1
            valid_rays += update.valid_rays
    except ValueError as exc:
        raise ServiceError(f"invalid mapping input: {exc}", 422) from exc
    states = grid.states()
    return {
        "format_version": 1,
        "algorithm": "occupancy_log_odds",
        "sensor": sensor,
        "pose_source": pose_source,
        "encoder": encoder if pose_source == "encoder" else None,
        "grid": grid.config.model_dump(),
        "states": states.tolist(),
        "probabilities": grid.probabilities.tolist(),
        "observed": grid.observed.tolist(),
        "used_scans": used,
        "skipped_scans": skipped,
        "valid_rays": valid_rays,
        "run_end_time": replay.states[-1].time,
        "counts": {
            "unknown": int((states == -1).sum()),
            "free": int((states == 0).sum()),
            "occupied": int((states == 100).sum()),
        },
    }
