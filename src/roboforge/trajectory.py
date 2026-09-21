"""Resample recorded ideal motion segments for display without inventing telemetry."""

import math

from roboforge.core import positive
from roboforge.geometry import Pose2
from roboforge.simulation import SimulationResult


def sample_trajectory(
    result: SimulationResult,
    *,
    distance_step: float = 0.05,
    angle_step: float = 0.1,
    max_samples: int = 100000,
) -> tuple[tuple[float, Pose2], ...]:
    """Reconstruct exact/Euler segment poses, distinct from the logged state samples.

    Controls/estimates/sensors are never inferred here. The cap fails explicitly
    instead of silently degrading a long or very high-speed trajectory's plot.
    """
    distance_step = positive(distance_step, "trajectory distance step")
    angle_step = positive(angle_step, "trajectory angle step")
    if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 1:
        raise ValueError("max_samples must be a positive integer")
    if len(result.motions) != len(result.states) - 1:
        raise ValueError("trajectory requires one recorded motion segment per state interval")
    samples = [(result.states[0].time, result.states[0].pose)]
    for index, motion in enumerate(result.motions):
        angle = abs(motion.drive.forward(motion.wheels).angular) * motion.dt
        count = max(1, math.ceil(motion.travel / distance_step), math.ceil(angle / angle_step))
        if len(samples) + count > max_samples:
            raise ValueError("trajectory sample budget exceeded; increase spacing or max_samples")
        start_time = result.states[index].time
        samples.extend(
            (start_time + motion.dt * fraction / count, motion.pose_at(fraction / count))
            for fraction in range(1, count + 1)
        )
    return tuple(samples)
