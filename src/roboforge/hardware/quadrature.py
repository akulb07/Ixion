"""Ideal quadrature edges reconstructed from sampled shaft angles."""

import math

from pydantic import Field

from roboforge.config import Nonnegative, Real, Schema
from roboforge.core import finite


class AngularSample(Schema):
    time_s: Nonnegative
    angle_rad: Real


class EncoderEdge(Schema):
    time_s: Nonnegative
    count: int
    a: bool
    b: bool


class EncoderTrace(Schema):
    cycles_per_revolution: int
    initial: EncoderEdge
    edges: tuple[EncoderEdge, ...]
    final_count: int


class QuadratureEncoder(Schema):
    """Cycles are per revolution of the supplied angle, not implicit motor turns."""

    cycles_per_revolution: int = Field(strict=True, gt=0, le=1_000_000)

    def trace(self, samples: tuple[AngularSample, ...], max_edges: int = 100_000) -> EncoderTrace:
        if not samples or len(samples) > 10_001:
            raise ValueError("encoder trace requires 1 to 10,001 angle samples")
        if type(max_edges) is not int or not 1 <= max_edges <= 100_000:
            raise ValueError("encoder edge budget must be an integer from 1 to 100,000")
        if any(b.time_s <= a.time_s for a, b in zip(samples, samples[1:])):
            raise ValueError("encoder sample times must strictly increase")
        scale = 4 * self.cycles_per_revolution / math.tau
        positions = tuple(finite(s.angle_rad * scale, "encoder position") for s in samples)
        # Keep adjacent integer thresholds distinguishable in floating point.
        if any(abs(p) >= 2**50 for p in positions):
            raise ValueError("encoder position exceeds supported numerical precision")
        counts = tuple(math.floor(p + 0.5) for p in positions)
        if sum(abs(b - a) for a, b in zip(counts, counts[1:])) > max_edges:
            raise ValueError("encoder trace exceeds edge budget")

        def edge(time, count):
            a, b = ((False, False), (True, False), (True, True), (False, True))[count % 4]
            return EncoderEdge(time_s=time, count=count, a=a, b=b)

        edges = []
        for index, (left, right) in enumerate(zip(samples, samples[1:])):
            start, end = counts[index : index + 2]
            if start == end:
                continue
            direction = 1 if end > start else -1
            p0, p1 = positions[index : index + 2]
            for count in range(start + direction, end + direction, direction):
                threshold = count - 0.5 * direction
                fraction = (threshold - p0) / (p1 - p0)
                time = left.time_s + fraction * (right.time_s - left.time_s)
                edges.append(edge(time, count))
        return EncoderTrace(
            cycles_per_revolution=self.cycles_per_revolution,
            initial=edge(samples[0].time_s, counts[0]),
            edges=tuple(edges),
            final_count=counts[-1],
        )
