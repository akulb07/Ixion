"""Kinematic tests use analytical or independent matrix references."""

import math
import unittest

import numpy as np

from roboforge.geometry import Pose2
from roboforge.robotics import BodyTwist2, DifferentialDrive, WheelSpeeds


class DifferentialDriveTests(unittest.TestCase):
    def setUp(self):
        self.drive = DifferentialDrive(0.05, 0.30)

    def test_forward_straight(self):
        twist = self.drive.forward(WheelSpeeds(10, 10))
        self.assertAlmostEqual(twist.linear, 0.5)
        self.assertEqual(twist.angular, 0)

    def test_forward_spin(self):
        twist = self.drive.forward(WheelSpeeds(-3, 3))
        self.assertEqual(twist.linear, 0)
        self.assertAlmostEqual(twist.angular, 1)

    def test_forward_turns_and_stationary(self):
        for wheels, expected in [
            (WheelSpeeds(2, 4), (0.15, 1 / 3)),
            (WheelSpeeds(4, 2), (0.15, -1 / 3)),
            (WheelSpeeds(0, 0), (0, 0)),
        ]:
            with self.subTest(wheels=wheels):
                twist = self.drive.forward(wheels)
                np.testing.assert_allclose([twist.linear, twist.angular], expected, atol=1e-14)

    def test_inverse_known_values(self):
        wheels = self.drive.inverse(BodyTwist2(0.5, 1))
        self.assertAlmostEqual(wheels.left, 7)
        self.assertAlmostEqual(wheels.right, 13)

    def test_seeded_inverse_against_linear_system(self):
        rng = np.random.default_rng(231)
        matrix = np.array([[0.025, 0.025], [-1 / 6, 1 / 6]])
        for _ in range(100):
            desired = rng.uniform(-2, 2, 2)
            wheels = self.drive.inverse(BodyTwist2(*desired))
            np.testing.assert_allclose(
                [wheels.left, wheels.right], np.linalg.solve(matrix, desired), atol=1e-12, rtol=0
            )
            actual = self.drive.forward(wheels)
            np.testing.assert_allclose([actual.linear, actual.angular], desired, atol=1e-12, rtol=0)

    def test_world_derivative(self):
        np.testing.assert_allclose(
            self.drive.derivative(Pose2(2, 3, math.pi / 2), WheelSpeeds(10, 10)),
            [0, 0.5, 0],
            atol=1e-15,
        )

    def test_straight_displacement(self):
        np.testing.assert_allclose(
            self.drive.integrate(Pose2(), WheelSpeeds(10, 10), 10).as_array(), [5, 0, 0], atol=1e-14
        )

    def test_reverse_at_rotated_heading(self):
        np.testing.assert_allclose(
            self.drive.integrate(Pose2(2, 3, math.pi / 2), WheelSpeeds(-10, -10), 2).as_array(),
            [2, 2, math.pi / 2],
            atol=1e-14,
        )

    def test_pure_rotation(self):
        result = self.drive.integrate(Pose2(2, 3), WheelSpeeds(-3, 3), math.pi / 2)
        np.testing.assert_allclose(result.as_array(), [2, 3, math.pi / 2], atol=1e-14)

    def test_left_and_right_quarter_circles(self):
        # v=.15, |omega|=1/3 => radius=.45, dt=3*pi/2.
        for wheels, sign in [(WheelSpeeds(2, 4), 1), (WheelSpeeds(4, 2), -1)]:
            with self.subTest(sign=sign):
                result = self.drive.integrate(Pose2(), wheels, 3 * math.pi / 2)
                np.testing.assert_allclose(
                    result.as_array(), [0.45, sign * 0.45, sign * math.pi / 2], atol=1e-14, rtol=0
                )

    def test_stationary_and_zero_time(self):
        pose = Pose2(1, -2, 0.4)
        self.assertEqual(self.drive.integrate(pose, WheelSpeeds(0, 0), 10), pose)
        self.assertEqual(self.drive.integrate(pose, WheelSpeeds(2, 9), 0), pose)

    def test_near_zero_angular_velocity(self):
        result = self.drive.integrate(Pose2(), WheelSpeeds(10, 10 + 1e-12), 2)
        self.assertAlmostEqual(result.x, 1, places=12)
        self.assertGreater(result.y, 0)
        self.assertLess(result.y, 1e-12)
        self.assertGreater(result.theta, 0)

    def test_general_arc_against_closed_form(self):
        start = Pose2(1.7, -0.2, 0.37)
        wheels = WheelSpeeds(-1, 5)
        v, omega, dt = 0.1, 1.0, 2.3
        expected = [
            start.x + v / omega * (math.sin(start.theta + omega * dt) - math.sin(start.theta)),
            start.y - v / omega * (math.cos(start.theta + omega * dt) - math.cos(start.theta)),
            start.theta + omega * dt,
        ]
        np.testing.assert_allclose(
            self.drive.integrate(start, wheels, dt).as_array(), expected, atol=1e-14, rtol=0
        )

    def test_dimensions_rejected_with_useful_errors(self):
        for radius, separation in [(0, 0.3), (-0.05, 0.3), (0.05, 0), (0.05, -0.3)]:
            with (
                self.subTest(radius=radius, separation=separation),
                self.assertRaisesRegex(ValueError, "greater than zero"),
            ):
                DifferentialDrive(radius, separation)
        with self.assertRaisesRegex(ValueError, "wheel separation"):
            DifferentialDrive(0.05, float("nan"))

    def test_invalid_rates_and_timestep(self):
        for value in [float("nan"), float("inf"), True, "1"]:
            for constructor in [WheelSpeeds, BodyTwist2]:
                with (
                    self.subTest(value=value, constructor=constructor),
                    self.assertRaises(ValueError),
                ):
                    constructor(value, 0)
                with (
                    self.subTest(value=value, constructor=constructor),
                    self.assertRaises(ValueError),
                ):
                    constructor(0, value)
        for dt in [-1, float("nan"), float("inf"), True]:
            with self.subTest(dt=dt), self.assertRaisesRegex(ValueError, "timestep"):
                self.drive.integrate(Pose2(), WheelSpeeds(1, 1), dt)

    def test_finite_inputs_that_overflow_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            self.drive.integrate(Pose2(), WheelSpeeds(1e308, 1e308), 1e308)
        with self.assertRaisesRegex(ValueError, "finite"):
            DifferentialDrive(1e308, 0.3).forward(WheelSpeeds(10, 10))
