"""Conservative geometric loop proposals and bounded batch graph/map correction."""

from dataclasses import dataclass

import numpy as np
from pydantic import Field

from roboforge.config import Positive, Real, Schema, Steps
from roboforge.geometry import Pose2
from roboforge.mapping import GridConfig, OccupancyGrid
from roboforge.pose_graph import GraphResult, PoseGraph
from roboforge.sensors.readings import LidarReading
from roboforge.slam import IcpConfig, MatchResult, lidar_points, match_points, relative


class LoopConfig(Schema):
    minimum_separation: Steps = 20
    search_radius: Positive = 1.0
    max_rmse: Positive = 0.06
    minimum_overlap: Real = Field(default=0.6, gt=0, le=1)
    max_translation_correction: Positive = 0.75
    max_angle_correction: Positive = 0.6
    icp: IcpConfig = IcpConfig(max_correspondence_distance=1.0, iterations=80)


@dataclass(frozen=True, slots=True)
class ScanKeyframe:
    scan: LidarReading
    pose: Pose2


@dataclass(frozen=True, slots=True)
class LoopProposal:
    source: int
    target: int
    match: MatchResult
    accepted: bool


@dataclass(frozen=True, slots=True)
class BatchSlamResult:
    graph: GraphResult
    loops: tuple[LoopProposal, ...]


def detect_loops(
    keyframes: tuple[ScanKeyframe, ...],
    config: LoopConfig = LoopConfig(),
    *,
    mount: Pose2 = Pose2(),
) -> tuple[LoopProposal, ...]:
    """At most one accepted loop per target; all tested candidates remain visible."""
    if len(keyframes) > 200:
        raise ValueError("loop detection exceeds 200-keyframe budget")
    clouds = [lidar_points(k.scan, mount) for k in keyframes]
    clouds = [
        p
        if len(p) <= config.icp.max_points
        else p[np.linspace(0, len(p) - 1, config.icp.max_points, dtype=int)]
        for p in clouds
    ]
    proposals = []
    for target in range(config.minimum_separation, len(keyframes)):
        candidates = [
            i
            for i in range(target - config.minimum_separation + 1)
            if (keyframes[i].pose.position - keyframes[target].pose.position).norm
            <= config.search_radius
        ]
        for source in sorted(
            candidates,
            key=lambda i: ((keyframes[i].pose.position - keyframes[target].pose.position).norm, i),
        ):
            prior = relative(keyframes[source].pose, keyframes[target].pose)
            match = match_points(clouds[target], clouds[source], prior, config.icp)
            correction = relative(prior, match.pose)
            accepted = (
                match.status == "converged"
                and match.rmse <= config.max_rmse
                and match.pairs >= config.minimum_overlap * len(clouds[target])
                and correction.position.norm <= config.max_translation_correction
                and abs(correction.theta) <= config.max_angle_correction
            )
            proposals.append(LoopProposal(source, target, match, accepted))
            if accepted:
                break
    return tuple(proposals)


def optimize_scan_graph(
    keyframes: tuple[ScanKeyframe, ...],
    grid: GridConfig,
    config: LoopConfig = LoopConfig(),
    *,
    mount: Pose2 = Pose2(),
    frame: str = "lidar",
    odometry_information=None,
    loop_information=None,
) -> tuple[BatchSlamResult, OccupancyGrid]:
    """Batch backend: relative priors + verified loop factors, then rebuild every scan.

    Information matrices are explicit user assumptions, not ICP covariance estimates.
    """
    if not keyframes or len(keyframes) > 200:
        raise ValueError("batch SLAM requires one to 200 keyframes")
    for index, k in enumerate(keyframes):
        if k.scan.frame != frame:
            raise ValueError("keyframe scan frame differs from mount")
        if index and (
            k.scan.sensor != keyframes[0].scan.sensor
            or k.scan.capture_time <= keyframes[index - 1].scan.capture_time
            or k.scan.sequence <= keyframes[index - 1].scan.sequence
        ):
            raise ValueError("keyframe scan stream must strictly increase")
    odometry_information = (
        np.diag([400, 400, 2500]) if odometry_information is None else odometry_information
    )
    loop_information = (
        np.diag([2500, 2500, 10000]) if loop_information is None else loop_information
    )
    graph = PoseGraph()
    for k in keyframes:
        graph.add_pose(k.pose)
    for index in range(1, len(keyframes)):
        graph.add_constraint(
            index - 1,
            index,
            relative(keyframes[index - 1].pose, keyframes[index].pose),
            odometry_information,
        )
    loops = detect_loops(keyframes, config, mount=mount)
    for loop in loops:
        if loop.accepted:
            graph.add_constraint(
                loop.source, loop.target, loop.match.pose, loop_information, "loop"
            )
    optimized = graph.optimize()
    occupancy = OccupancyGrid(grid)
    for keyframe, pose in zip(keyframes, optimized.poses):
        occupancy.update(
            keyframe.scan, pose, pose_time=keyframe.scan.capture_time, mount=mount, frame=frame
        )
    return BatchSlamResult(optimized, loops), occupancy
