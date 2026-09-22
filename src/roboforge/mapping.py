"""LiDAR inverse sensor model in a bounded, half-open occupancy grid."""

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from pydantic import Field, model_validator

from roboforge.config import Positive, Real, Schema, Steps
from roboforge.core import finite
from roboforge.geometry import Pose2, Vector2
from roboforge.sensors.readings import LidarReading


class GridConfig(Schema):
    columns: Steps = 100
    rows: Steps = 100
    resolution: Positive = 0.05
    origin_x: Real = 0.0
    origin_y: Real = 0.0
    free_probability: Real = Field(default=0.3, gt=0, lt=0.5)
    occupied_probability: Real = Field(default=0.7, gt=0.5, lt=1)
    log_odds_limit: Positive = 5.0
    free_threshold: Real = Field(default=0.4, gt=0, lt=0.5)
    occupied_threshold: Real = Field(default=0.6, gt=0.5, lt=1)

    @model_validator(mode="after")
    def bounded_storage(self):
        if self.rows * self.columns > 4_000_000:
            raise ValueError("grid exceeds four million cell allocation budget")
        for origin, count in ((self.origin_x, self.columns), (self.origin_y, self.rows)):
            extent = finite(count * self.resolution, "map extent")
            upper = finite(origin + extent, "map bound")
            if upper <= origin or origin + self.resolution == origin:
                raise ValueError("map resolution is not representable at this origin")
        if self.log_odds_limit > 100:
            raise ValueError("log odds limit must not exceed 100")
        return self


@dataclass(frozen=True, slots=True)
class MapUpdate:
    sensor: str
    sequence: int
    time: float
    valid_rays: int
    free_cells: int
    occupied_cells: int


class OccupancyGrid:
    """One evidence increment per cell per scan; occupied evidence wins overlaps."""

    def __init__(self, config: GridConfig = GridConfig()):
        self.config = config
        self._log_odds = np.zeros((config.rows, config.columns), dtype=float)
        self._observed = np.zeros((config.rows, config.columns), dtype=bool)
        self._streams: dict[str, tuple[str, int, float]] = {}

    @property
    def log_odds(self):
        return self._log_odds.copy()

    @property
    def observed(self):
        return self._observed.copy()

    @property
    def probabilities(self):
        return 1 / (1 + np.exp(-self._log_odds))

    def states(self):
        """int8 array: -1 unknown/uncertain, 0 free, 100 occupied; indexed [row,y][col,x]."""
        p = self.probabilities
        state = np.full(p.shape, -1, dtype=np.int8)
        state[self._observed & (p <= self.config.free_threshold)] = 0
        state[self._observed & (p >= self.config.occupied_threshold)] = 100
        return state

    def cell(self, point: Vector2) -> tuple[int, int] | None:
        c = self.config
        x = (point.x - c.origin_x) / c.resolution
        y = (point.y - c.origin_y) / c.resolution
        if not (0 <= x < c.columns and 0 <= y < c.rows):
            return None
        return math.floor(y), math.floor(x)

    def ray_cells(self, start: Vector2, end: Vector2) -> tuple[tuple[int, int], ...]:
        """Clip then traverse positive-length cell intersections; no fixed sampling.

        Grid-line crossings partition the clipped segment. Midpoints identify
        half-open cells, so corner touches alone add no cells. Endpoints are
        handled separately by the inverse sensor model.
        """
        c = self.config
        a = ((start.x - c.origin_x) / c.resolution, (start.y - c.origin_y) / c.resolution)
        b = ((end.x - c.origin_x) / c.resolution, (end.y - c.origin_y) / c.resolution)
        delta = (b[0] - a[0], b[1] - a[1])
        if not all(math.isfinite(v) for v in (*a, *b, *delta)):
            raise ValueError("ray coordinates exceed finite grid arithmetic")
        lower, upper = 0.0, 1.0
        for origin, direction, size in zip(a, delta, (c.columns, c.rows)):
            if direction == 0:
                if not 0 <= origin < size:
                    return ()
            else:
                entry, leave = sorted((-origin / direction, (size - origin) / direction))
                lower, upper = max(lower, entry), min(upper, leave)
        if lower >= upper or delta == (0, 0):
            return ()
        crossings = {lower, upper}
        for origin, direction, size in zip(a, delta, (c.columns, c.rows)):
            if direction:
                near, far = sorted((origin + lower * direction, origin + upper * direction))
                for boundary in range(max(0, math.floor(near) + 1), min(size, math.ceil(far))):
                    crossing = (boundary - origin) / direction
                    if lower < crossing < upper:
                        crossings.add(crossing)
        times = sorted(crossings)
        cells = []
        for left, right in zip(times, times[1:]):
            middle = (left + right) / 2
            x, y = a[0] + middle * delta[0], a[1] + middle * delta[1]
            if 0 <= x < c.columns and 0 <= y < c.rows:
                cell = math.floor(y), math.floor(x)
                if not cells or cells[-1] != cell:
                    cells.append(cell)
        return tuple(cells)

    def update(
        self,
        scan: LidarReading,
        pose: Pose2,
        *,
        pose_time: float,
        mount: Pose2 = Pose2(),
        frame: str = "lidar",
    ) -> MapUpdate:
        if finite(pose_time, "map pose time") != scan.capture_time:
            raise ValueError("mapping pose time must match LiDAR capture time")
        if scan.frame != frame:
            raise ValueError("LiDAR frame does not match the supplied mount")
        previous = self._streams.get(scan.sensor)
        if previous is not None and (
            scan.frame != previous[0]
            or scan.sequence <= previous[1]
            or scan.capture_time <= previous[2]
        ):
            raise ValueError(
                "scan stream must have a fixed frame and strictly increasing sequence/time"
            )
        if len(scan.angles) > 100000:
            raise ValueError("scan exceeds ray budget")
        origin = pose.position + mount.position.rotated(pose.theta)
        heading = pose.theta + mount.theta
        free, occupied, valid = set(), set(), 0
        for angle, distance, hit in zip(scan.angles, scan.ranges, scan.hits):
            if distance is None:
                continue
            valid += 1
            endpoint = (
                origin + Vector2(math.cos(heading + angle), math.sin(heading + angle)) * distance
            )
            free.update(self.ray_cells(origin, endpoint))
            endpoint_cell = self.cell(endpoint)
            if hit and endpoint_cell is not None:
                occupied.add(endpoint_cell)
        free.difference_update(occupied)
        c = self.config
        for cells, probability in ((free, c.free_probability), (occupied, c.occupied_probability)):
            if cells:
                ys, xs = zip(*sorted(cells))
                indices = np.array(ys), np.array(xs)
                increment = math.log(probability / (1 - probability))
                self._log_odds[indices] = np.clip(
                    self._log_odds[indices] + increment, -c.log_odds_limit, c.log_odds_limit
                )
                self._observed[indices] = True
        self._streams[scan.sensor] = (scan.frame, scan.sequence, scan.capture_time)
        return MapUpdate(
            scan.sensor, scan.sequence, scan.capture_time, valid, len(free), len(occupied)
        )

    def save(self, path: str | Path):
        """Portable data-only NPZ snapshot; not a resumable scan scheduler."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            format_version=np.array(1),
            config=np.array(self.config.model_dump_json()),
            log_odds=self._log_odds,
            observed=self._observed,
            states=self.states(),
        )
