from math import pi

import pytest

from roboforge.geometry import Pose2
from roboforge.odometry import EncoderOdometry
from roboforge.sensors.readings import EncoderReading


def reading(t, left, right, **kwargs):
    data = dict(
        sensor="encoders",
        frame="base",
        sequence=round(t * 100),
        capture_time=t,
        delivery_time=t,
        ticks_per_revolution=400,
        left_ticks=left,
        right_ticks=right,
    )
    return EncoderReading(**(data | kwargs))


def test_straight_reverse_and_prior():
    odom = EncoderOdometry(0.1, 0.4, Pose2(1, 2, pi / 2))
    assert odom.update(reading(0, 100, 100)).pose == Pose2(1, 2, pi / 2)
    estimate = odom.update(reading(1, 500, 500))
    assert estimate.pose.x == pytest.approx(1)
    assert estimate.pose.y == pytest.approx(2 + 0.2 * pi)
    reverse = odom.update(reading(2, 100, 100))
    assert reverse.pose.y == pytest.approx(2)
    assert reverse.distance == pytest.approx(0.4 * pi)
    assert reverse.twist.linear < 0


def test_quarter_arc():
    odom = EncoderOdometry(0.1, 0.4)
    odom.update(reading(0, 0, 0))
    estimate = odom.update(reading(1, 0, 400))
    assert estimate.pose.x == pytest.approx(0.2)
    assert estimate.pose.y == pytest.approx(0.2)
    assert estimate.pose.theta == pytest.approx(pi / 2)


def test_in_place_rotation():
    odom = EncoderOdometry(0.1, 0.4)
    odom.update(reading(0, 0, 0))
    estimate = odom.update(reading(1, -200, 200))
    assert estimate.pose.x == estimate.pose.y == estimate.distance == 0
    assert estimate.pose.theta == pytest.approx(pi / 2)


def test_dropout_bridge_and_initial_missing_sample():
    odom = EncoderOdometry(0.1, 0.4)
    assert odom.update(reading(0, None, 0)) is None
    assert odom.update(reading(1, 0, 0)).sample_interval == 0
    assert odom.update(reading(2, 100, None)) is None
    estimate = odom.update(reading(3, 400, 400))
    assert estimate.sample_interval == 2
    assert estimate.pose.x == pytest.approx(0.2 * pi)


@pytest.mark.parametrize(
    "changes",
    [
        {"sensor": "other"},
        {"frame": "other"},
        {"ticks_per_revolution": 200},
        {"capture_time": 0},
        {"sequence": 0},
    ],
)
def test_reject_bad_stream_without_changing_pose(changes):
    odom = EncoderOdometry(0.1, 0.4)
    odom.update(reading(0, 0, 0))
    with pytest.raises(ValueError):
        odom.update(reading(1, 400, 400, **changes))
    assert odom.pose == Pose2(0, 0, 0)


def test_delivery_latency_does_not_change_motion_estimate():
    fast, slow = EncoderOdometry(0.1, 0.4), EncoderOdometry(0.1, 0.4)
    for t in range(3):
        a = fast.update(reading(t, t * 100, t * 200))
        b = slow.update(reading(t, t * 100, t * 200, delivery_time=t + 5))
        assert a == b
