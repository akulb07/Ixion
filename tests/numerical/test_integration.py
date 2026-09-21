"""Convergence against independent closed-form trajectories."""

import math
import unittest

import numpy as np

from roboforge.geometry import Pose2
from roboforge.robotics import DifferentialDrive, WheelSpeeds


class IntegratorTests(unittest.TestCase):
    def test_euler_step_uses_old_heading(self):
        drive = DifferentialDrive(0.05, 0.3)
        result = drive.integrate(Pose2(), WheelSpeeds(-1, 5), 0.1, "euler")
        np.testing.assert_allclose(result.as_array(), [0.01, 0, 0.1], atol=1e-14, rtol=0)

    def test_euler_first_order_convergence(self):
        drive, wheels = DifferentialDrive(0.05, 0.3), WheelSpeeds(-1, 5)
        expected = np.array([0.1 * math.sin(1), 0.1 * (1 - math.cos(1))])
        errors = []
        for steps in (20, 40, 80):
            pose = Pose2()
            for _ in range(steps):
                pose = drive.integrate(pose, wheels, 1 / steps, "euler")
            errors.append(np.linalg.norm(pose.position.as_array() - expected))
        for coarse, fine in zip(errors, errors[1:]):
            self.assertGreater(coarse / fine, 1.99)
            self.assertLess(coarse / fine, 2.01)

    def test_exact_error_across_step_sizes(self):
        drive, wheels = DifferentialDrive(0.05, 0.3), WheelSpeeds(-1, 5)
        expected = [0.1 * math.sin(1), 0.1 * (1 - math.cos(1)), 1]
        for steps in (1, 20, 40, 80):
            pose = Pose2()
            for _ in range(steps):
                pose = drive.integrate(pose, wheels, 1 / steps)
            np.testing.assert_allclose(pose.as_array(), expected, atol=1e-12, rtol=0)

    def test_invalid_integrator_even_at_zero_dt(self):
        with self.assertRaisesRegex(ValueError, "integration method"):
            DifferentialDrive(0.05, 0.3).integrate(Pose2(), WheelSpeeds(0, 0), 0, "rk99")
