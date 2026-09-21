"""Display reconstruction must preserve within-step arcs and multiple turns."""

import math
import unittest

import numpy as np

from roboforge.config import RunConfig
from roboforge.simulation import Simulator
from roboforge.trajectory import sample_trajectory


class TrajectoryTests(unittest.TestCase):
    def test_full_loop_contains_arc_not_zero_length_chord(self):
        result = Simulator(
            RunConfig(
                robot={"wheel_radius": 1, "wheel_separation": 2, "initial_pose": {"x": 5, "y": 5}},
                simulation={"dt": math.tau},
                commands=[{"left": 0, "right": 2, "steps": 1}],
            )
        ).run()
        samples = sample_trajectory(result)
        self.assertGreater(len(samples), 100)
        for _, pose in samples:
            self.assertAlmostEqual(math.hypot(pose.x - 5, pose.y - 6), 1, places=12)
        self.assertGreater(max(pose.x for _, pose in samples), 5.99)
        np.testing.assert_allclose(
            samples[-1][1].as_array(), result.states[-1].pose.as_array(), atol=1e-12
        )

    def test_large_spin_can_be_unwrapped_from_recorded_motion(self):
        result = Simulator(
            RunConfig(simulation={"dt": 30}, commands=[{"left": -1, "right": 1, "steps": 1}])
        ).run()
        angles = np.unwrap([pose.theta for _, pose in sample_trajectory(result)])
        self.assertAlmostEqual(angles[-1], 10, places=12)

    def test_sampling_budget_and_parameters(self):
        result = Simulator(RunConfig(commands=[{"left": 1, "right": 1, "steps": 1}])).run()
        with self.assertRaisesRegex(ValueError, "budget"):
            sample_trajectory(result, max_samples=1)
        with self.assertRaises(ValueError):
            sample_trajectory(result, distance_step=0)

    def test_contact_at_start_keeps_single_sample(self):
        result = Simulator(
            RunConfig(
                simulation={"collision": {"mode": "stop"}},
                commands=[{"left": 1, "right": 1, "steps": 1}],
            )
        ).run()
        self.assertEqual(len(sample_trajectory(result)), 1)
