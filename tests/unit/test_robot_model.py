"""Independent mass/inertia and mounting-frame checks."""

import math
import unittest

import numpy as np
from pydantic import ValidationError

from roboforge.config import RobotConfig
from roboforge.geometry import Pose2
from roboforge.robot import DifferentialDriveRobot


class RobotModelTests(unittest.TestCase):
    def test_uniform_body_and_wheel_mass_properties(self):
        properties = DifferentialDriveRobot(RobotConfig()).mass_properties
        self.assertAlmostEqual(properties.total_mass, 10.5)
        self.assertAlmostEqual(properties.body_yaw_inertia, 0.2)
        self.assertAlmostEqual(properties.wheel_spin_inertia, 0.0003125)
        self.assertAlmostEqual(properties.total_yaw_inertia, 0.2115625)

    def test_explicit_body_inertia_override(self):
        properties = DifferentialDriveRobot(RobotConfig(body_yaw_inertia=0.4)).mass_properties
        self.assertAlmostEqual(properties.body_yaw_inertia, 0.4)
        self.assertAlmostEqual(properties.total_yaw_inertia, 0.4115625)

    def test_default_frame_chain(self):
        frames = {
            item.source_frame: item
            for item in DifferentialDriveRobot(RobotConfig()).frame_transforms(
                Pose2(2, 3, math.pi / 2)
            )
        }
        self.assertEqual(set(frames), {"base", "left_wheel", "right_wheel", "lidar", "imu"})
        self.assertTrue(all(frame.target_frame == "world" for frame in frames.values()))
        np.testing.assert_allclose(
            frames["left_wheel"].translation.as_array(), [1.85, 3], atol=1e-14
        )
        np.testing.assert_allclose(
            frames["right_wheel"].translation.as_array(), [2.15, 3], atol=1e-14
        )
        np.testing.assert_allclose(frames["lidar"].translation.as_array(), [2, 3.1], atol=1e-14)

    def test_custom_mount_is_declarative(self):
        robot = DifferentialDriveRobot(
            RobotConfig(
                mounts=[{"name": "payload", "pose": {"x": 0.2, "y": -0.1, "theta": math.pi / 2}}]
            )
        )
        frame = robot.frame_transforms(Pose2())[-1]
        self.assertEqual(frame.source_frame, "payload")
        self.assertAlmostEqual(frame.theta, math.pi / 2)

    def test_invalid_physical_parameters_and_mounts(self):
        for field in ["body_mass", "wheel_mass", "body_yaw_inertia"]:
            for value in [-1, 0, True, float("nan")]:
                with self.subTest(field=field, value=value), self.assertRaises(ValidationError):
                    RobotConfig(**{field: value})
        for mounts in [
            [{"name": "base"}],
            [{"name": "imu"}, {"name": "imu"}],
            [{"name": "bad name"}],
        ]:
            with self.subTest(mounts=mounts), self.assertRaises(ValidationError):
                RobotConfig(mounts=mounts)
