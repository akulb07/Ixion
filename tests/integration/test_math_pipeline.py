"""Milestone 1 integration only: commands -> kinematics -> pose -> frames."""

import math
import unittest

import numpy as np

from roboforge.geometry import Pose2, Transform2, Vector2
from roboforge.robotics import BodyTwist2, DifferentialDrive


class MathPipelineTests(unittest.TestCase):
    def test_square_returns_to_origin(self):
        drive = DifferentialDrive(0.05, 0.3)
        pose = Pose2()
        for _ in range(4):
            pose = drive.integrate(pose, drive.inverse(BodyTwist2(0.5, 0)), 2)
            pose = drive.integrate(pose, drive.inverse(BodyTwist2(0, math.pi / 2)), 1)
        np.testing.assert_allclose(pose.as_array(), [0, 0, 0], atol=1e-12, rtol=0)

    def test_robot_and_sensor_frame_chain(self):
        drive = DifferentialDrive(0.05, 0.3)
        pose = drive.integrate(Pose2(), drive.inverse(BodyTwist2(1, 1)), math.pi / 2)
        world_base = Transform2.from_pose(pose, target_frame="world", source_frame="base")
        base_lidar = Transform2("base", "lidar", Vector2(0.2, 0))
        np.testing.assert_allclose(
            (world_base @ base_lidar).apply_point(Vector2(2, 0)).as_array(),
            [1, 3.2],
            atol=1e-12,
            rtol=0,
        )
        for name, offset in [
            ("left_wheel", Vector2(0, 0.15)),
            ("right_wheel", Vector2(0, -0.15)),
            ("imu", Vector2(0, 0)),
        ]:
            mounted = world_base @ Transform2("base", name, offset)
            self.assertEqual(mounted.source_frame, name)
            np.testing.assert_allclose(
                mounted.inverse().apply_point(mounted.translation).as_array(), [0, 0], atol=1e-12
            )
