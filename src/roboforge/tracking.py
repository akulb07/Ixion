"""Pure Pursuit for supplied pose estimates; no access to simulation truth."""

import math
from dataclasses import dataclass

from roboforge.core import positive
from roboforge.geometry import Pose2, Vector2
from roboforge.robotics import BodyTwist2


@dataclass(frozen=True, slots=True)
class TrackingSample:
    command: BodyTwist2
    target: Vector2
    progress: float
    cross_track_error: float
    goal_distance: float
    reached: bool


class PurePursuit:
    """Forward-only arc-length lookahead with turn-in-place and terminal braking."""

    def __init__(
        self,
        path: tuple[Vector2, ...],
        lookahead: float = 0.35,
        max_speed: float = 0.3,
        max_yaw_rate: float = 1.5,
        goal_tolerance: float = 0.05,
    ) -> None:
        self.path = tuple(path)
        if len(self.path) < 2 or not all(isinstance(p, Vector2) for p in self.path):
            raise ValueError("tracking requires at least two Vector2 waypoints")
        self.lengths = [(b - a).norm for a, b in zip(self.path, self.path[1:])]
        if any(length == 0 for length in self.lengths):
            raise ValueError("consecutive waypoints must differ")
        self.cumulative = [0.0]
        for length in self.lengths:
            self.cumulative.append(self.cumulative[-1] + length)
        self.lookahead = positive(lookahead, "lookahead")
        self.max_speed = positive(max_speed, "max speed")
        self.max_yaw_rate = positive(max_yaw_rate, "max yaw rate")
        self.goal_tolerance = positive(goal_tolerance, "goal tolerance")
        self.progress = 0.0

    def update(self, pose: Pose2) -> TrackingSample:
        position = Vector2(pose.x, pose.y)
        best_distance, best_progress = math.inf, self.progress
        for i, (start, end) in enumerate(zip(self.path, self.path[1:])):
            if self.cumulative[i + 1] < self.progress:
                continue
            delta = end - start
            lower = max(0, (self.progress - self.cumulative[i]) / self.lengths[i])
            fraction = max(lower, min(1, (position - start).dot(delta) / self.lengths[i] ** 2))
            distance = (position - (start + delta * fraction)).norm
            progress = self.cumulative[i] + fraction * self.lengths[i]
            if (distance, progress) < (best_distance, best_progress):
                best_distance, best_progress = distance, progress
        self.progress = best_progress
        target_progress = min(self.cumulative[-1], self.progress + self.lookahead)
        target = self.path[-1]
        for i, length in enumerate(self.lengths):
            if target_progress <= self.cumulative[i + 1]:
                fraction = (target_progress - self.cumulative[i]) / length
                target = self.path[i] + (self.path[i + 1] - self.path[i]) * fraction
                break
        goal_distance = (position - self.path[-1]).norm
        reached = (
            goal_distance <= self.goal_tolerance
            and self.cumulative[-1] - self.progress <= self.lookahead
        )
        local = (target - position).rotated(-pose.theta)
        heading = math.atan2(local.y, local.x)
        if reached:
            command = BodyTwist2(0, 0)
        elif abs(heading) > math.pi / 3:
            command = BodyTwist2(0, max(-self.max_yaw_rate, min(self.max_yaw_rate, 2 * heading)))
        else:
            curvature = 2 * local.y / max(local.norm**2, 1e-12)
            speed = min(self.max_speed, goal_distance)
            if abs(curvature) > 1e-12:
                speed = min(speed, self.max_yaw_rate / abs(curvature))
            command = BodyTwist2(speed, speed * curvature)
        return TrackingSample(command, target, self.progress, best_distance, goal_distance, reached)
