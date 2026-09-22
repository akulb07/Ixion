"""Bounded point-to-point ICP and a replaceable incremental scan-to-map front end."""

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from pydantic import Field

from roboforge.config import Positive, Real, Schema, Steps
from roboforge.core import finite
from roboforge.geometry import Pose2
from roboforge.mapping import GridConfig, OccupancyGrid
from roboforge.sensors.readings import LidarReading


def compose(a: Pose2, b: Pose2) -> Pose2:
    translation = a.position + b.position.rotated(a.theta)
    return Pose2(translation.x, translation.y, a.theta + b.theta)


def relative(a: Pose2, b: Pose2) -> Pose2:
    translation = (b.position - a.position).rotated(-a.theta)
    return Pose2(translation.x, translation.y, b.theta - a.theta)


def transform_points(points: np.ndarray, pose: Pose2) -> np.ndarray:
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    return points @ np.array([[c, s], [-s, c]]) + [pose.x, pose.y]


def lidar_points(scan: LidarReading, mount: Pose2 = Pose2()) -> np.ndarray:
    """Valid hit endpoints only, in base coordinates; no max-range fake points."""
    points = np.array(
        [
            [distance * math.cos(angle), distance * math.sin(angle)]
            for angle, distance, hit in zip(scan.angles, scan.ranges, scan.hits)
            if distance is not None and hit
        ],
        dtype=float,
    ).reshape((-1, 2))
    return transform_points(points, mount)


class IcpConfig(Schema):
    iterations: Steps = 50
    max_points: Steps = Field(default=1500, le=2000, ge=6)
    minimum_pairs: Steps = Field(default=6, ge=3)
    max_correspondence_distance: Positive = 0.5
    trim_fraction: Real = Field(default=0.9, gt=0, le=1)
    translation_tolerance: Positive = 1e-5
    rotation_tolerance: Positive = 1e-5


@dataclass(frozen=True, slots=True)
class MatchResult:
    pose: Pose2
    status: str
    iterations: int
    pairs: int
    rmse: float | None
    residual_history: tuple[float, ...]


def _cloud(values, budget):
    points = np.array(values, dtype=float, copy=True)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("point cloud must have finite shape (n,2)")
    if len(points) > budget:
        raise ValueError("point cloud exceeds configured point budget")
    return points


def _pairs(source, target, config):
    squared = ((source[:, None, :] - target[None, :, :]) ** 2).sum(axis=2)
    if not np.isfinite(squared).all():
        raise ValueError("point distances overflowed")
    nearest = squared.argmin(axis=1)
    distances = squared[np.arange(len(source)), nearest]
    valid = np.flatnonzero(distances <= config.max_correspondence_distance**2)
    count = min(
        len(valid), max(config.minimum_pairs, math.floor(len(valid) * config.trim_fraction))
    )
    chosen = valid[np.argsort(distances[valid], kind="stable")[:count]]
    return chosen, nearest[chosen]


def match_points(
    source, target, initial: Pose2 = Pose2(), config: IcpConfig = IcpConfig()
) -> MatchResult:
    """Estimate target-from-source. Local ICP needs overlap and a good initial prior."""
    source, target = _cloud(source, config.max_points), _cloud(target, config.max_points)
    if min(len(source), len(target)) < config.minimum_pairs:
        return MatchResult(initial, "insufficient_correspondences", 0, 0, None, ())
    pose, history = initial, []
    for iteration in range(1, config.iterations + 1):
        transformed = transform_points(source, pose)
        a, b = _pairs(transformed, target, config)
        if len(a) < config.minimum_pairs:
            return MatchResult(
                pose, "insufficient_correspondences", iteration, len(a), None, tuple(history)
            )
        x, y = transformed[a], target[b]
        xc, yc = x - x.mean(axis=0), y - y.mean(axis=0)
        # Conservative refusal of collapsed or collinear local correspondences.
        for centered in (xc, yc):
            singular = np.linalg.svd(centered, compute_uv=False)
            if singular[0] < 1e-10 or singular[1] < singular[0] * 1e-6:
                return MatchResult(pose, "degenerate", iteration, len(a), None, tuple(history))
        u, _, vt = np.linalg.svd(xc.T @ yc)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        translation = y.mean(axis=0) - rotation @ x.mean(axis=0)
        angle = math.atan2(rotation[1, 0], rotation[0, 0])
        delta = Pose2(*translation, angle)
        pose = compose(delta, pose)
        transformed = transform_points(source, pose)
        a, b = _pairs(transformed, target, config)
        if len(a) < config.minimum_pairs:
            return MatchResult(
                pose, "insufficient_correspondences", iteration, len(a), None, tuple(history)
            )
        rmse = float(np.sqrt(np.mean(np.sum((transformed[a] - target[b]) ** 2, axis=1))))
        history.append(rmse)
        if (
            delta.position.norm <= config.translation_tolerance
            and abs(angle) <= config.rotation_tolerance
        ):
            return MatchResult(pose, "converged", iteration, len(a), rmse, tuple(history))
    return MatchResult(
        pose, "iteration_limit", config.iterations, len(a), history[-1], tuple(history)
    )


class SlamConfig(Schema):
    icp: IcpConfig = IcpConfig()
    voxel_size: Positive = 0.05
    max_match_rmse: Positive = 0.1
    max_correction_distance: Positive = 0.5
    max_correction_angle: Positive = 0.3


@dataclass(frozen=True, slots=True)
class SlamEstimate:
    time: float
    pose: Pose2
    status: str
    match: MatchResult | None
    map_updated: bool


class SlamBackend(Protocol):
    def update(self, scan: LidarReading, prior: Pose2, *, prior_time: float) -> SlamEstimate: ...


class IncrementalSlam:
    """Scan-to-map ICP + occupancy mapping, anchored at the first odometry prior.

    Rejected matches report propagated odometry and do not insert scan evidence.
    No pose graph, loop closure or statistically calibrated covariance is implied.
    """

    def __init__(
        self,
        grid: GridConfig,
        config: SlamConfig = SlamConfig(),
        *,
        mount: Pose2 = Pose2(),
        frame: str = "lidar",
    ):
        self.grid = OccupancyGrid(grid)
        self.config, self.mount, self.frame = config, mount, frame
        self._points = np.empty((0, 2))
        self._prior: Pose2 | None = None
        self._pose: Pose2 | None = None
        self._last_scan: LidarReading | None = None

    @property
    def points(self):
        return self._points.copy()

    def _bounded_points(self, points):
        scaled = np.floor(points / self.config.voxel_size)
        if not np.isfinite(scaled).all():
            raise ValueError("voxel coordinates overflowed")
        _, indices = np.unique(scaled, axis=0, return_index=True)
        points = points[np.sort(indices)]
        if len(points) > self.config.icp.max_points:
            points = points[np.linspace(0, len(points) - 1, self.config.icp.max_points, dtype=int)]
        return points

    def update(self, scan: LidarReading, prior: Pose2, *, prior_time: float) -> SlamEstimate:
        if finite(prior_time, "prior timestamp") != scan.capture_time or scan.frame != self.frame:
            raise ValueError("scan requires matching prior capture time and mount frame")
        last = self._last_scan
        if last is not None and (
            scan.sensor != last.sensor
            or scan.sequence <= last.sequence
            or scan.capture_time <= last.capture_time
        ):
            raise ValueError("SLAM scan stream must strictly increase without changing sensors")
        points = self._bounded_points(lidar_points(scan, self.mount))
        predicted = (
            prior if self._prior is None else compose(self._pose, relative(self._prior, prior))
        )
        match = None
        enough = len(points) >= self.config.icp.minimum_pairs
        if len(self._points) == 0:
            accepted = enough
            pose, status = predicted, "initialized" if enough else "rejected"
        else:
            match = match_points(points, self._points, predicted, self.config.icp)
            correction = relative(predicted, match.pose)
            accepted = (
                match.status == "converged"
                and match.rmse <= self.config.max_match_rmse
                and correction.position.norm <= self.config.max_correction_distance
                and abs(correction.theta) <= self.config.max_correction_angle
            )
            pose = match.pose if accepted else predicted
            status = "matched" if accepted else "rejected"
        if accepted:
            merged = self._bounded_points(np.vstack((self._points, transform_points(points, pose))))
            self.grid.update(
                scan, pose, pose_time=scan.capture_time, mount=self.mount, frame=self.frame
            )
            self._points = merged
        self._prior, self._pose, self._last_scan = prior, pose, scan
        return SlamEstimate(scan.capture_time, pose, status, match, accepted)
