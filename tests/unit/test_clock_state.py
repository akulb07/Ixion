"""Clock ownership and derived ground-truth velocities."""

import math
import unittest
from dataclasses import FrozenInstanceError

from roboforge.config import RobotConfig
from roboforge.geometry import Pose2
from roboforge.robot import DifferentialDriveRobot, RobotState
from roboforge.robotics import BodyTwist2, WheelSpeeds
from roboforge.simulation import SimulationClock


class ClockStateTests(unittest.TestCase):
    def test_clock_uses_integer_ticks(self):
        clock = SimulationClock(0.01)
        self.assertEqual(clock.frequency, 100)
        for _ in range(10000):
            clock = clock.advanced()
        self.assertEqual(clock.tick, 10000)
        self.assertEqual(clock.time, 100)

    def test_clock_validation(self):
        for dt in [0, -1, float("nan"), float("inf")]:
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                SimulationClock(dt)
        for tick in [-1, 1.5, True]:
            with self.subTest(tick=tick), self.assertRaises(ValueError):
                SimulationClock(0.01, tick)
        with self.assertRaises(FrozenInstanceError):
            SimulationClock(0.01).tick = 1

    def test_state_velocities_are_world_frame(self):
        state = RobotState(Pose2(1, 2, math.pi / 2), WheelSpeeds(1, 1), BodyTwist2(0.5, 0.2), 3)
        self.assertAlmostEqual(state.vx, 0)
        self.assertAlmostEqual(state.vy, 0.5)
        self.assertEqual(state.omega, 0.2)
        with self.assertRaises(FrozenInstanceError):
            state.time = 4

    def test_state_validation(self):
        with self.assertRaisesRegex(ValueError, "state time"):
            RobotState(Pose2(), WheelSpeeds(0, 0), BodyTwist2(0, 0), -1)
        with self.assertRaisesRegex(ValueError, "Pose2"):
            RobotState((0, 0, 0), WheelSpeeds(0, 0), BodyTwist2(0, 0), 0)

    def test_robot_rejects_inconsistent_clock_time(self):
        robot = DifferentialDriveRobot(RobotConfig())
        with self.assertRaisesRegex(ValueError, "previous timestamp"):
            robot.step(robot.initial_state(), WheelSpeeds(1, 1), 0.01, 2, "exact")
