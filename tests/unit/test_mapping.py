import math

import numpy as np
import pytest

from roboforge.geometry import Pose2, Vector2
from roboforge.mapping import GridConfig, OccupancyGrid
from roboforge.sensors.readings import LidarReading


def scan(distance=2, hit=True, sequence=0, angles=(0,), **kwargs):
    return LidarReading(
        sensor="lidar",
        frame="lidar",
        sequence=sequence,
        capture_time=float(sequence),
        delivery_time=float(sequence),
        angles=angles,
        ranges=(distance,) * len(angles),
        hits=(hit,) * len(angles),
        max_range=10,
        **kwargs,
    )


def grid():
    return OccupancyGrid(GridConfig(columns=5, rows=5, resolution=1))


def test_hit_clears_ray_and_marks_endpoint_preserving_unknown():
    m = grid()
    update = m.update(scan(), Pose2(0.5, 0.5, 0), pose_time=0)
    np.testing.assert_array_equal(m.states()[0], [0, 0, 100, -1, -1])
    assert update.free_cells == 2 and update.occupied_cells == 1
    assert not m.observed[1:].any()


def test_max_range_and_dropout_do_not_invent_obstacles():
    m = grid()
    m.update(scan(hit=False), Pose2(0.5, 0.5), pose_time=0)
    assert (m.states()[0, :3] == 0).all()
    before = m.log_odds
    m.update(scan(distance=None, hit=False, sequence=1), Pose2(0.5, 0.5), pose_time=1)
    np.testing.assert_array_equal(m.log_odds, before)


def test_mount_translation_rotation_and_robot_rotation():
    m = grid()
    m.update(scan(), Pose2(2.5, 0.5, math.pi / 2), pose_time=0, mount=Pose2(1, 0, 0))
    assert m.states()[3, 2] == 100
    assert m.states()[1, 2] == 0


@pytest.mark.parametrize(
    "start,end,expected",
    [
        ((-2, 0.5), (7, 0.5), ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4))),
        ((0.5, 0.5), (3.5, 3.5), ((0, 0), (1, 1), (2, 2), (3, 3))),
        ((3.5, 0.5), (0.5, 0.5), ((0, 3), (0, 2), (0, 1), (0, 0))),
        ((5, 0), (5, 5), ()),
        ((0.5, 0.5), (0.5, 0.5), ()),
    ],
)
def test_exact_ray_traversal(start, end, expected):
    assert grid().ray_cells(Vector2(*start), Vector2(*end)) == expected


def test_outside_endpoint_does_not_mark_map_border_occupied():
    m = grid()
    m.update(scan(distance=7), Pose2(0.5, 0.5), pose_time=0)
    assert (m.states()[0] == 0).all()


def test_duplicate_ray_evidence_once_per_scan_and_saturation():
    m = grid()
    m.update(scan(angles=(0, 0, 0)), Pose2(0.5, 0.5), pose_time=0)
    assert m.probabilities[0, 2] == pytest.approx(0.7)
    for sequence in range(1, 100):
        m.update(scan(sequence=sequence), Pose2(0.5, 0.5), pose_time=sequence)
    assert m.log_odds.min() == -5 and m.log_odds.max() == 5
    snapshot = m.log_odds
    snapshot[:] = 999
    assert m.log_odds.max() == 5


def test_timestamp_frame_and_duplicate_rejections_are_atomic():
    m = grid()
    m.update(scan(), Pose2(0.5, 0.5), pose_time=0)
    before = m.log_odds
    for kwargs in ({"pose_time": 1}, {"pose_time": 0, "frame": "wrong"}, {"pose_time": 0}):
        with pytest.raises(ValueError):
            m.update(scan(), Pose2(0.5, 0.5), **kwargs)
    np.testing.assert_array_equal(before, m.log_odds)


def test_snapshot_export(tmp_path):
    m = grid()
    m.update(scan(), Pose2(0.5, 0.5), pose_time=0)
    path = tmp_path / "grid.npz"
    m.save(path)
    with np.load(path, allow_pickle=False) as saved:
        assert saved["format_version"] == 1
        assert GridConfig.model_validate_json(str(saved["config"])) == m.config
        np.testing.assert_array_equal(saved["log_odds"], m.log_odds)


def test_storage_budget():
    with pytest.raises(ValueError, match="budget"):
        GridConfig(rows=100000, columns=100000)
