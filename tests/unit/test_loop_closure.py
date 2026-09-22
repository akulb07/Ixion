import math

import numpy as np
import pytest

from roboforge.geometry import Pose2
from roboforge.loop_closure import LoopConfig, ScanKeyframe, detect_loops, optimize_scan_graph
from roboforge.mapping import GridConfig
from roboforge.sensors.readings import LidarReading


def scan(index, valid=True):
    angles = tuple(np.linspace(-math.pi, math.pi, 120, endpoint=False))
    return LidarReading(
        sensor="lidar",
        frame="lidar",
        sequence=index,
        capture_time=float(index),
        delivery_time=float(index),
        angles=angles,
        ranges=tuple(
            2 + 0.3 * math.sin(3 * a) + 0.2 * math.cos(2 * a) if valid else None for a in angles
        ),
        hits=(valid,) * len(angles),
        max_range=10,
    )


def test_loop_requires_temporal_gap_and_geometric_verification():
    frames = tuple(ScanKeyframe(scan(i), Pose2(3 + 0.01 * i, 3, 0.002 * i)) for i in range(6))
    assert detect_loops(frames, LoopConfig(minimum_separation=10)) == ()
    proposals = detect_loops(frames, LoopConfig(minimum_separation=5))
    assert len(proposals) == 1 and proposals[0].accepted
    np.testing.assert_allclose(proposals[0].match.pose.as_array(), [0, 0, 0], atol=1e-9)


def test_invalid_scan_cannot_close_loop():
    frames = (ScanKeyframe(scan(0), Pose2(3, 3)), ScanKeyframe(scan(1, False), Pose2(3.1, 3)))
    assert not detect_loops(frames, LoopConfig(minimum_separation=1))[0].accepted


def test_batch_correction_rebuilds_map_with_fixed_anchor():
    frames = tuple(ScanKeyframe(scan(i), Pose2(3 + 0.01 * i, 3, 0.002 * i)) for i in range(6))
    result, grid = optimize_scan_graph(
        frames, GridConfig(columns=120, rows=120), LoopConfig(minimum_separation=5)
    )
    assert result.graph.status == "converged"
    assert result.graph.poses[0] == frames[0].pose
    assert abs(result.graph.poses[-1].x - 3) < 0.01
    assert result.graph.costs[-1] < result.graph.costs[0]
    assert grid.observed.any()
    with pytest.raises(ValueError):
        optimize_scan_graph((frames[0], frames[0]), GridConfig())
