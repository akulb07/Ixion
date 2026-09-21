"""SE(2) geometry in metres and radians; column-vector convention."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from roboforge.core import finite, positive, wrap_angle


@dataclass(frozen=True, slots=True)
class Vector2:
    """Immutable finite 2D vector; frame and physical units belong to its caller."""

    x: float
    y: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", finite(self.x, "x"))
        object.__setattr__(self, "y", finite(self.y, "y"))

    def __add__(self, other: Vector2) -> Vector2:
        if not isinstance(other, Vector2):
            return NotImplemented
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector2) -> Vector2:
        if not isinstance(other, Vector2):
            return NotImplemented
        return Vector2(self.x - other.x, self.y - other.y)

    def __neg__(self) -> Vector2:
        return Vector2(-self.x, -self.y)

    def __mul__(self, scalar: float) -> Vector2:
        scalar = finite(scalar, "scale")
        return Vector2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> Vector2:
        return self * scalar

    def __truediv__(self, scalar: float) -> Vector2:
        scalar = finite(scalar, "divisor")
        if scalar == 0.0:
            raise ValueError("divisor must be nonzero")
        return Vector2(self.x / scalar, self.y / scalar)

    def dot(self, other: Vector2) -> float:
        """Euclidean inner product."""
        return finite(self.x * other.x + self.y * other.y, "dot product")

    def cross(self, other: Vector2) -> float:
        """Signed z component of the planar cross product."""
        return finite(self.x * other.y - self.y * other.x, "cross product")

    @property
    def norm(self) -> float:
        """Euclidean length, computed without unnecessary squaring overflow."""
        return finite(math.hypot(self.x, self.y), "norm")

    def normalized(self) -> Vector2:
        """Return unit direction; a zero vector has no direction."""
        if self.norm == 0.0:
            raise ValueError("cannot normalize a zero vector")
        return self / self.norm

    def rotated(self, angle: float) -> Vector2:
        """Rotate counterclockwise by angle radians about the origin."""
        angle = wrap_angle(angle)
        c, s = math.cos(angle), math.sin(angle)
        return Vector2(c * self.x - s * self.y, s * self.x + c * self.y)

    def as_array(self) -> NDArray[np.float64]:
        """Return a new array; mutating it cannot change this value object."""
        return np.array([self.x, self.y], dtype=np.float64)


@dataclass(frozen=True, slots=True)
class Pose2:
    """Planar position and canonical heading, expressed in a caller-known frame."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", finite(self.x, "pose x"))
        object.__setattr__(self, "y", finite(self.y, "pose y"))
        object.__setattr__(self, "theta", wrap_angle(self.theta))

    @property
    def position(self) -> Vector2:
        return Vector2(self.x, self.y)

    def as_array(self) -> NDArray[np.float64]:
        return np.array([self.x, self.y, self.theta], dtype=np.float64)


@dataclass(frozen=True, slots=True)
class Transform2:
    """T_target_source: p_target = R(theta) p_source + translation.

    Composition ``a @ b`` applies b first. It requires
    ``a.source_frame == b.target_frame`` and returns T_a.target_b.source.
    """

    target_frame: str
    source_frame: str
    translation: Vector2 = Vector2(0.0, 0.0)
    theta: float = 0.0

    def __post_init__(self) -> None:
        for name in ("target_frame", "source_frame"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a nonempty frame name")
        if not isinstance(self.translation, Vector2):
            raise ValueError("translation must be a Vector2")
        object.__setattr__(self, "theta", wrap_angle(self.theta))

    @classmethod
    def identity(cls, frame: str) -> Transform2:
        return cls(frame, frame)

    @classmethod
    def from_pose(cls, pose: Pose2, *, target_frame: str, source_frame: str) -> Transform2:
        """Interpret pose as the source frame's pose in the target frame."""
        return cls(target_frame, source_frame, pose.position, pose.theta)

    @property
    def matrix(self) -> NDArray[np.float64]:
        """New homogeneous 3x3 matrix; all translations are in metres."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return np.array(
            [
                [c, -s, self.translation.x],
                [s, c, self.translation.y],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    @classmethod
    def from_matrix(
        cls,
        matrix: ArrayLike,
        *,
        target_frame: str,
        source_frame: str,
        atol: float = 1e-10,
    ) -> Transform2:
        """Validate an SE(2) matrix, rejecting scaling, shear, and reflection.

        atol bounds floating-point residuals; accepted roundoff is projected to
        the angle representation. It is not a general matrix repair operation.
        """
        atol = positive(atol, "matrix tolerance")
        if atol > 1e-6:
            raise ValueError("matrix tolerance must not exceed 1e-6")
        try:
            raw = np.asarray(matrix)
            if raw.dtype.kind not in "fiu":
                raise ValueError("matrix entries must be real numbers")
            values = np.array(raw, dtype=np.float64)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("matrix must be a finite real 3x3 SE(2) matrix") from exc
        if values.shape != (3, 3) or not np.all(np.isfinite(values)):
            raise ValueError("matrix must be a finite real 3x3 SE(2) matrix")
        if not np.allclose(values[2], [0.0, 0.0, 1.0], atol=atol, rtol=0.0):
            raise ValueError("homogeneous matrix bottom row must be [0, 0, 1]")
        rotation = values[:2, :2]
        # Bound first, so malformed huge inputs cannot overflow in R.T @ R.
        if np.any(np.abs(rotation) > 1.0 + atol):
            raise ValueError("rotation block must be orthonormal with determinant +1")
        if not np.allclose(rotation.T @ rotation, np.eye(2), atol=atol, rtol=0.0):
            raise ValueError("rotation block must be orthonormal")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=atol, rel_tol=0.0):
            raise ValueError("rotation block must have determinant +1")
        return cls(
            target_frame,
            source_frame,
            Vector2(*values[:2, 2]),
            math.atan2(values[1, 0], values[0, 0]),
        )

    def apply_point(self, point: Vector2) -> Vector2:
        """Transform a position (homogeneous coordinate 1)."""
        return point.rotated(self.theta) + self.translation

    def apply_vector(self, vector: Vector2) -> Vector2:
        """Transform a free vector (homogeneous coordinate 0); no translation."""
        return vector.rotated(self.theta)

    def apply_pose(self, pose: Pose2) -> Pose2:
        """Express a source-frame pose in the target frame."""
        point = self.apply_point(pose.position)
        return Pose2(point.x, point.y, self.theta + pose.theta)

    def inverse(self) -> Transform2:
        """Return T_source_target with translation -R.T @ t."""
        return Transform2(
            self.source_frame,
            self.target_frame,
            (-self.translation).rotated(-self.theta),
            -self.theta,
        )

    def __matmul__(self, other: Transform2) -> Transform2:
        if not isinstance(other, Transform2):
            return NotImplemented
        if self.source_frame != other.target_frame:
            raise ValueError(
                f"cannot compose frames: {self.source_frame!r} != {other.target_frame!r}"
            )
        return Transform2(
            self.target_frame,
            other.source_frame,
            self.apply_point(other.translation),
            self.theta + other.theta,
        )
