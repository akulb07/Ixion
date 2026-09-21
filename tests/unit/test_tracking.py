import math

import pytest

from roboforge.geometry import Pose2, Vector2
from roboforge.robotics import DifferentialDrive
from roboforge.tracking import PurePursuit


def test_straight_and_terminal_stop():
    follower = PurePursuit((Vector2(0, 0), Vector2(2, 0)))
    sample = follower.update(Pose2(0, 0, 0))
    assert sample.command.linear == 0.3 and sample.command.angular == 0
    terminal = follower.update(Pose2(2, 0, 0))
    assert terminal.reached and terminal.command.linear == terminal.command.angular == 0


def test_target_behind_rotates_in_place():
    follower = PurePursuit((Vector2(0, 0), Vector2(2, 0)))
    sample = follower.update(Pose2(0, 0, math.pi))
    assert sample.command.linear == 0
    assert abs(sample.command.angular) <= 1.5


def test_kinematic_tracking_reaches_goal_with_bounded_commands():
    follower = PurePursuit((Vector2(0, 0), Vector2(1, 0), Vector2(2, 1), Vector2(3, 1)))
    drive, pose = DifferentialDrive(0.05, 0.3), Pose2(0, -0.15, 0)
    previous_progress = 0
    for _ in range(3000):
        sample = follower.update(pose)
        assert sample.progress >= previous_progress
        assert 0 <= sample.command.linear <= 0.3
        assert abs(sample.command.angular) <= 1.5 + 1e-12
        previous_progress = sample.progress
        if sample.reached:
            break
        pose = drive.integrate(pose, drive.inverse(sample.command), 0.02)
    assert sample.reached and sample.goal_distance <= 0.05


def test_invalid_path():
    with pytest.raises(ValueError):
        PurePursuit((Vector2(0, 0), Vector2(0, 0)))
