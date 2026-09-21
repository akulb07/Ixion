"""Analytical first-hit distances for static primitive geometry."""

import math
from dataclasses import dataclass

from roboforge.config import Circle, Environment
from roboforge.core import finite, positive
from roboforge.geometry import Vector2


@dataclass(frozen=True, slots=True)
class RayHit:
    distance: float
    object_id: str | None


def raycast(environment: Environment, origin: Vector2, angle: float, max_range: float) -> RayHit:
    """Return first surface along a unit ray; an origin inside a solid hits at zero."""
    angle, max_range = finite(angle, "ray angle"), positive(max_range, "ray range")
    direction = Vector2(math.cos(angle), math.sin(angle))
    width, height = environment.width, environment.height
    if not (0 <= origin.x <= width and 0 <= origin.y <= height):
        return RayHit(0, "boundary:outside")
    candidates = []
    for coordinate, component, lower, upper, low_name, high_name in [
        (origin.x, direction.x, 0, width, "left", "right"),
        (origin.y, direction.y, 0, height, "bottom", "top"),
    ]:
        if component > 0:
            candidates.append(((upper - coordinate) / component, f"boundary:{high_name}"))
        elif component < 0:
            candidates.append(((lower - coordinate) / component, f"boundary:{low_name}"))
    for index, obstacle in enumerate(environment.obstacles):
        hit = None
        if isinstance(obstacle, Circle):
            offset = origin - Vector2(obstacle.x, obstacle.y)
            c = offset.dot(offset) - obstacle.radius**2
            b = offset.dot(direction)
            discriminant = b * b - c
            if c <= 0:
                hit = 0.0
            elif discriminant >= 0:
                root = math.sqrt(discriminant)
                if -b - root >= 0:
                    hit = -b - root
        else:
            enter, leave = 0.0, math.inf
            for coordinate, component, lower, upper in [
                (origin.x, direction.x, obstacle.x, obstacle.x + obstacle.width),
                (origin.y, direction.y, obstacle.y, obstacle.y + obstacle.height),
            ]:
                if component == 0:
                    if not lower <= coordinate <= upper:
                        leave = -1
                else:
                    a, b = (lower - coordinate) / component, (upper - coordinate) / component
                    enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
            if enter <= leave:
                hit = enter
        if hit is not None:
            candidates.append((hit, f"obstacle:{index}"))
    distance, object_id = min(candidates, key=lambda item: item[0])
    return RayHit(distance, object_id) if distance <= max_range else RayHit(max_range, None)
