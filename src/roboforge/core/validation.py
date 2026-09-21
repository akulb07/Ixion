"""Validate public numerical inputs before they enter robotics calculations."""

import math
from numbers import Real


def finite(value: float, name: str) -> float:
    """Return a finite real scalar, rejecting booleans and implicit strings."""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite real number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real number")
    return result


def positive(value: float, name: str) -> float:
    """Validate a strictly positive physical dimension."""
    result = finite(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return result


def nonnegative(value: float, name: str) -> float:
    """Validate a nonnegative duration or tolerance."""
    result = finite(value, name)
    if result < 0.0:
        raise ValueError(f"{name} must be greater than or equal to zero")
    return result


def wrap_angle(angle: float) -> float:
    """Canonicalize radians to [-pi, pi); +pi and -pi denote the same heading."""
    angle = finite(angle, "angle")
    # Remainder avoids cancellation from adding pi to a tiny input angle.
    wrapped = math.remainder(angle, math.tau)
    return -math.pi if wrapped == math.pi else wrapped
