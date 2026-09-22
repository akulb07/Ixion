"""Signed clearance and conservative continuous collision checks for disks.

The centre follows the actual configured kinematic path, including full arcs.
A 1-Lipschitz signed-distance bound certifies empty intervals. No fixed sampling
rate or endpoint-only shortcut is used. See docs/collision.md for error bounds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from roboforge.config import Circle, Environment, Rectangle
from roboforge.core import finite, nonnegative, positive
from roboforge.geometry import Pose2, Vector2
from roboforge.robotics import DifferentialDrive, WheelSpeeds


@dataclass(frozen=True, slots=True)
class Contact:
    """Nearest surface pair; normal points toward free space for this obstacle.

    Penetration is an individual constraint's separation depth. It is not a
    global minimum translation that resolves several simultaneous contacts.
    """

    object_id: str
    clearance: float
    point_on_obstacle: Vector2
    point_on_robot: Vector2
    normal: Vector2

    @property
    def penetration(self) -> float:
        return max(0.0, -self.clearance)


@dataclass(frozen=True, slots=True)
class CollisionReport:
    nearest: Contact
    contacts: tuple[Contact, ...]

    @property
    def clearance(self) -> float:
        return self.nearest.clearance

    @property
    def collision(self) -> bool:
        """Tangency counts as collision; no hidden tolerance in static queries."""
        return self.clearance <= 0

    @property
    def minimum_distance(self) -> float:
        return max(0.0, self.clearance)


@dataclass(frozen=True, slots=True)
class KinematicMotion:
    """Piecewise-constant drive motion for one timestep, parameterized by [0,1]."""

    start: Pose2
    drive: DifferentialDrive
    wheels: WheelSpeeds
    dt: float
    method: Literal["exact", "euler"] = "exact"
    encoder_wheels: WheelSpeeds | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "dt", nonnegative(self.dt, "motion duration"))
        if self.method not in ("exact", "euler"):
            raise ValueError("motion method must be 'exact' or 'euler'")
        if not isinstance(self.start, Pose2) or not isinstance(self.drive, DifferentialDrive):
            raise ValueError("motion requires a Pose2 and DifferentialDrive")
        if not isinstance(self.wheels, WheelSpeeds):
            raise ValueError("motion requires WheelSpeeds")
        if self.encoder_wheels is not None and not isinstance(self.encoder_wheels, WheelSpeeds):
            raise ValueError("encoder wheel rates must be WheelSpeeds")
        finite(self.drive.forward(self.wheels).linear * self.dt, "motion travel")

    @property
    def travel(self) -> float:
        """Centre path length in metres, also valid for the Euler straight path."""
        return abs(self.drive.forward(self.wheels).linear) * self.dt

    def pose_at(self, fraction: float) -> Pose2:
        fraction = finite(fraction, "motion fraction")
        if not 0 <= fraction <= 1:
            raise ValueError("motion fraction must be in [0, 1]")
        return self.drive.integrate(self.start, self.wheels, self.dt * fraction, self.method)


@dataclass(frozen=True, slots=True)
class SweepResult:
    reason: Literal["clear", "initial_contact", "contact", "conservative_contact"]
    safe_fraction: float
    interval: tuple[float, float] | None
    report: CollisionReport | None
    queries: int

    @property
    def blocked(self) -> bool:
        return self.reason != "clear"


def _contact(
    object_id: str,
    centre: Vector2,
    radius: float,
    signed_distance: float,
    point: Vector2,
    normal: Vector2,
) -> Contact:
    return Contact(
        object_id,
        finite(signed_distance - radius, "clearance"),
        point,
        centre - radius * normal,
        normal,
    )


def _circle_contact(centre: Vector2, radius: float, obstacle: Circle, index: int) -> Contact:
    offset = centre - Vector2(obstacle.x, obstacle.y)
    distance = offset.norm
    normal = offset / distance if distance > 0 else Vector2(1, 0)
    point = Vector2(obstacle.x, obstacle.y) + obstacle.radius * normal
    return _contact(f"obstacle:{index}", centre, radius, distance - obstacle.radius, point, normal)


def _rectangle_contact(centre: Vector2, radius: float, obstacle: Rectangle, index: int) -> Contact:
    left, bottom = obstacle.x, obstacle.y
    right, top = left + obstacle.width, bottom + obstacle.height
    point = Vector2(min(max(centre.x, left), right), min(max(centre.y, bottom), top))
    offset = centre - point
    distance = offset.norm
    if distance > 0:
        normal = offset / distance
    else:
        # Centre inside/on rectangle: nearest face determines signed depth.
        # Tie order left, right, bottom, top is stable, including centre/corners.
        gap, point, normal = min(
            [
                (centre.x - left, Vector2(left, centre.y), Vector2(-1, 0)),
                (right - centre.x, Vector2(right, centre.y), Vector2(1, 0)),
                (centre.y - bottom, Vector2(centre.x, bottom), Vector2(0, -1)),
                (top - centre.y, Vector2(centre.x, top), Vector2(0, 1)),
            ],
            key=lambda item: item[0],
        )
        distance = -gap
    return _contact(f"obstacle:{index}", centre, radius, distance, point, normal)


class CollisionWorld:
    """Read-only static geometry; boundaries are four inward-facing half-planes."""

    def __init__(self, environment: Environment) -> None:
        self.environment = environment

    def query(self, centre: Vector2, radius: float) -> CollisionReport:
        """Return signed footprint clearance, all contacts, nearest surface/normal."""
        radius = positive(radius, "footprint radius")
        if not isinstance(centre, Vector2):
            raise ValueError("footprint centre must be a Vector2")
        width, height = self.environment.width, self.environment.height
        distances = [
            _contact(
                "boundary:left", centre, radius, centre.x, Vector2(0, centre.y), Vector2(1, 0)
            ),
            _contact(
                "boundary:right",
                centre,
                radius,
                width - centre.x,
                Vector2(width, centre.y),
                Vector2(-1, 0),
            ),
            _contact(
                "boundary:bottom", centre, radius, centre.y, Vector2(centre.x, 0), Vector2(0, 1)
            ),
            _contact(
                "boundary:top",
                centre,
                radius,
                height - centre.y,
                Vector2(centre.x, height),
                Vector2(0, -1),
            ),
        ]
        for index, obstacle in enumerate(self.environment.obstacles):
            operation = _circle_contact if isinstance(obstacle, Circle) else _rectangle_contact
            distances.append(operation(centre, radius, obstacle, index))
        nearest = min(distances, key=lambda item: item.clearance)
        return CollisionReport(nearest, tuple(item for item in distances if item.clearance <= 0))

    def sweep(
        self,
        motion: KinematicMotion,
        radius: float,
        *,
        spatial_tolerance: float = 1e-6,
        max_queries: int = 100000,
    ) -> SweepResult:
        """Certify a path clear or return its earliest unresolved/contact interval.

        The returned safe_fraction is collision-free unless initially in contact.
        An unresolved leaf can be a near miss within spatial_tolerance; the reason
        explicitly distinguishes that conservative stop from a sampled collision.
        Query-budget exhaustion raises ValueError, never returns a clear path.
        """
        tolerance = positive(spatial_tolerance, "sweep spatial tolerance")
        radius = positive(radius, "footprint radius")
        if isinstance(max_queries, bool) or not isinstance(max_queries, int) or max_queries < 1:
            raise ValueError("max_queries must be a positive integer")
        travel = motion.travel
        scale = max(
            1.0,
            self.environment.width,
            self.environment.height,
            abs(motion.start.x),
            abs(motion.start.y),
            travel,
            radius,
        )
        guard = 64 * math.ulp(scale)
        if tolerance < 4 * guard:
            raise ValueError("sweep spatial tolerance is too small for the coordinate/travel scale")
        initial = self.query(motion.start.position, radius)
        queries = 1
        if initial.collision:
            return SweepResult("initial_contact", 0, (0, 0), initial, queries)
        if travel == 0:
            return SweepResult("clear", 1, None, None, queries)
        # Right intervals are pushed first; all earlier intervals are certified
        # before a candidate can be returned. This also handles whole loops.
        pending = [(0.0, 1.0)]
        while pending:
            if queries >= max_queries:
                raise ValueError(
                    "swept collision query budget exhausted; path was not certified clear"
                )
            lower, upper = pending.pop()
            middle = (lower + upper) / 2
            report = self.query(motion.pose_at(middle).position, radius)
            queries += 1
            half_travel = travel * (upper - lower) / 2
            if report.clearance - half_travel - guard > 0:
                continue
            if 2 * half_travel <= tolerance:
                reason = "contact" if report.collision else "conservative_contact"
                return SweepResult(reason, lower, (lower, upper), report, queries)
            if middle == lower or middle == upper:
                raise ValueError("sweep cannot refine further at floating-point precision")
            pending.append((middle, upper))
            pending.append((lower, middle))
        return SweepResult("clear", 1, None, None, queries)
