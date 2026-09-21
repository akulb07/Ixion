"""Explicit analytical regression ranges, not implementation-generated goldens."""

import math
import unittest

import numpy as np

from roboforge.geometry import Pose2
from roboforge.robotics import DifferentialDrive, WheelSpeeds


class MotionRegressionTests(unittest.TestCase):
    def test_ten_second_straight_at_100_hz(self):
        drive, pose = DifferentialDrive(0.05, 0.3), Pose2()
        for _ in range(1000):
            pose = drive.integrate(pose, WheelSpeeds(10, 10), 0.01)
        np.testing.assert_allclose(pose.as_array(), [5, 0, 0], atol=1e-12, rtol=0)

    def test_full_circle(self):
        drive, pose = DifferentialDrive(0.05, 0.3), Pose2()
        for _ in range(2000):
            pose = drive.integrate(pose, WheelSpeeds(2, 4), 3 * math.tau / 2000)
        np.testing.assert_allclose(pose.as_array(), [0, 0, 0], atol=1e-11, rtol=0)

    def test_partition_invariance(self):
        drive, start, wheels = DifferentialDrive(0.05, 0.3), Pose2(2, 3, 0.2), WheelSpeeds(-2, 7)
        whole = drive.integrate(start, wheels, 10)
        steps = start
        for _ in range(1000):
            steps = drive.integrate(steps, wheels, 0.01)
        np.testing.assert_allclose(steps.as_array(), whole.as_array(), atol=1e-12, rtol=0)

    def test_repeatability(self):
        drive = DifferentialDrive(0.05, 0.3)

        def run():
            pose = Pose2()
            trajectory = []
            for i in range(300):
                pose = drive.integrate(pose, WheelSpeeds(2 + i / 300, 4), 0.01)
                trajectory.append(pose.as_array())
            return np.array(trajectory)

        np.testing.assert_array_equal(run(), run())
