import math

import numpy as np
import pytest

from roboforge.geometry import Pose2
from roboforge.mapping import GridConfig
from roboforge.sensors.readings import LidarReading
from roboforge.slam import (
    IcpConfig,
    IncrementalSlam,
    compose,
    lidar_points,
    match_points,
    relative,
    transform_points,
)


def cloud():
    return np.random.default_rng(91).uniform(-2, 2, (100, 2))


def test_pose_composition_inverse():
    a, b = Pose2(3, -2, 0.7), Pose2(0.2, 0.3, -0.1)
    np.testing.assert_allclose(compose(a, relative(a, b)).as_array(), b.as_array(), atol=1e-14)


def test_icp_recovers_known_rigid_transform_without_reflection():
    source = cloud()
    truth = Pose2(0.12, -0.08, 0.06)
    target = transform_points(source, truth)
    match = match_points(source, target, config=IcpConfig(trim_fraction=1))
    assert match.status == "converged"
    np.testing.assert_allclose(match.pose.as_array(), truth.as_array(), atol=1e-10)
    assert match.rmse < 1e-10
    assert match == match_points(source, target, config=IcpConfig(trim_fraction=1))


def test_icp_outliers_and_correspondence_gate():
    source = cloud()
    target = transform_points(source, Pose2(0.05, -0.04, 0.02))
    source = np.vstack((source, [[20, 20], [50, 50]]))
    match = match_points(source, target)
    assert match.status == "converged" and match.rmse < 1e-9
    rejected = match_points(source, target + 100)
    assert rejected.status == "insufficient_correspondences"


def test_icp_degenerate_and_iteration_budget():
    line = np.column_stack((np.linspace(0, 2, 20), np.zeros(20)))
    assert match_points(line, line).status == "degenerate"
    assert (
        match_points(cloud(), cloud() + 0.12, config=IcpConfig(iterations=1)).status
        == "iteration_limit"
    )
    with pytest.raises(ValueError, match="budget"):
        match_points(np.zeros((2001, 2)), cloud())


def scan(sequence, invalid=False):
    angles = tuple(np.linspace(-math.pi, math.pi, 80, endpoint=False))
    ranges = tuple(2 + 0.3 * math.sin(3 * a) for a in angles)
    return LidarReading(
        sensor="lidar",
        frame="lidar",
        sequence=sequence,
        capture_time=float(sequence),
        delivery_time=float(sequence),
        angles=angles,
        ranges=(None,) * 80 if invalid else ranges,
        hits=(not invalid,) * 80,
        max_range=10,
    )


def test_frontend_alignment_rejection_and_copied_map_points():
    slam = IncrementalSlam(GridConfig(columns=120, rows=120, resolution=0.05))
    first = slam.update(scan(0), Pose2(3, 3), prior_time=0)
    assert first.status == "initialized"
    matched = slam.update(scan(1), Pose2(3.05, 3.02, 0.02), prior_time=1)
    assert matched.status == "matched"
    np.testing.assert_allclose(matched.pose.as_array(), [3, 3, 0], atol=1e-9)
    before = slam.grid.log_odds
    rejected = slam.update(scan(2, invalid=True), Pose2(3.05, 3.02, 0.02), prior_time=2)
    assert rejected.status == "rejected" and not rejected.map_updated
    np.testing.assert_array_equal(slam.grid.log_odds, before)
    points = slam.points
    points[:] = 999
    assert not (slam.points == 999).all()
    with pytest.raises(ValueError):
        slam.update(scan(2), Pose2(3, 3), prior_time=2)


def test_lidar_points_excludes_missing_and_no_hit():
    s = LidarReading(
        sensor="lidar",
        frame="lidar",
        sequence=0,
        capture_time=0,
        delivery_time=0,
        angles=(0, 1, 2),
        ranges=(2, None, 10),
        hits=(True, False, False),
        max_range=10,
    )
    np.testing.assert_allclose(lidar_points(s, Pose2(1, 0, math.pi / 2)), [[1, 2]], atol=1e-12)
