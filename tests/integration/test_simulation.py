"""Complete Milestone 1 flow: configuration -> robot -> clock -> result -> files."""

import contextlib
import csv
import io
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from roboforge.cli import main
from roboforge.config import RunConfig, load_config
from roboforge.core import wrap_angle
from roboforge.io import save_result
from roboforge.simulation import Simulator
from roboforge.visualization import plot_trajectory

ROOT = Path(__file__).resolve().parents[2]


class SimulationTests(unittest.TestCase):
    def test_configured_straight_then_spin(self):
        result = Simulator(load_config(ROOT / "configs" / "foundation.yaml")).run()
        self.assertEqual(len(result.states), 2001)
        np.testing.assert_allclose(
            result.states[1000].pose.as_array(), [1.5, 1, 0], atol=1e-12, rtol=0
        )
        np.testing.assert_allclose(
            result.states[-1].pose.as_array(), [1.5, 1, wrap_angle(10 / 3)], atol=1e-12, rtol=0
        )
        self.assertEqual(result.states[1000].time, 10)
        self.assertEqual(result.states[-1].time, 20)
        self.assertEqual(result.states[0].twist.linear, 0)
        self.assertEqual(result.states[1000].wheels.left, 1)
        self.assertEqual(result.states[1001].wheels.left, -1)

    def test_repeat_same_instance_and_fresh_instance(self):
        config = load_config(ROOT / "configs" / "foundation.yaml")
        simulator = Simulator(config)
        first = simulator.run()
        self.assertEqual(first, simulator.run())
        self.assertEqual(first, Simulator(config).run())

    def test_curved_motion_via_config(self):
        config = RunConfig(
            robot={"initial_pose": {"x": 1, "y": 1}},
            simulation={"dt": 0.01},
            commands=[{"left": -1, "right": 5, "steps": 100}],
        )
        result = Simulator(config).run()
        np.testing.assert_allclose(
            result.states[-1].pose.as_array(),
            [1 + 0.1 * math.sin(1), 1 + 0.1 * (1 - math.cos(1)), 1],
            atol=1e-12,
            rtol=0,
        )

    def test_stationary_stays_fixed(self):
        config = RunConfig(commands=[{"left": 0, "right": 0, "steps": 100}])
        result = Simulator(config).run()
        self.assertTrue(all(state.pose == result.states[0].pose for state in result.states))

    def test_save_reload_and_csv(self):
        result = Simulator(RunConfig(commands=[{"left": 1, "right": 1, "steps": 10}])).run()
        with tempfile.TemporaryDirectory() as temp:
            directory = save_result(result, temp)
            restored = Simulator(load_config(directory / "config.json")).run()
            self.assertEqual(restored, result)
            metadata = json.loads((directory / "metadata.json").read_text())
            self.assertEqual(metadata["seed"], 42)
            self.assertFalse(metadata["collision_enabled"])
            with (directory / "trajectory.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 11)
            self.assertEqual(float(rows[-1]["time_s"]), 0.1)
            self.assertAlmostEqual(float(rows[-1]["x_m"]), 0.005)

    def test_cli_success_and_actionable_failure(self):
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(["validate", str(ROOT / "configs" / "foundation.yaml")]), 0)
            self.assertEqual(
                main(["simulate", str(ROOT / "configs" / "foundation.yaml"), "--output", temp]), 0
            )
            self.assertTrue((Path(temp) / "trajectory.csv").is_file())
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                self.assertEqual(main(["validate", str(Path(temp) / "missing.yaml")]), 2)
            self.assertIn("Invalid configuration", stderr.getvalue())

    def test_plot_exports_real_result(self):
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            self.skipTest("optional matplotlib not installed")
        result = Simulator(load_config(ROOT / "configs" / "foundation.yaml")).run()
        with tempfile.TemporaryDirectory() as temp:
            path = plot_trajectory(result, Path(temp) / "trajectory.png")
            self.assertEqual(path.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            self.assertGreater(path.stat().st_size, 10000)
