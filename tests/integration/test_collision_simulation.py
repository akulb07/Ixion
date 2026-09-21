"""Config -> swept world -> deterministic terminal state -> export/CLI."""

import contextlib
import io
import json
import math
import tempfile
import unittest
from pathlib import Path

from roboforge.cli import main
from roboforge.config import RunConfig, load_config
from roboforge.io import save_result
from roboforge.physics import CollisionWorld
from roboforge.simulation import SimulationClock, Simulator

ROOT = Path(__file__).resolve().parents[2]


class CollisionSimulationTests(unittest.TestCase):
    def test_zero_fraction_stop_preserves_previous_executed_motion(self):
        config = RunConfig(
            robot={"initial_pose": {"x": 1, "y": 1}},
            environment={"width": 2.2, "height": 3},
            simulation={"dt": 1, "collision": {"mode": "stop"}},
            commands=[
                {"left": 19.99999, "right": 19.99999, "steps": 1},
                {"left": 100, "right": 100, "steps": 1},
            ],
        )
        result = Simulator(config).run()
        self.assertEqual(result.status, "collision")
        self.assertEqual(result.states[-1].time, 1)
        self.assertEqual(len(result.motions), 1)
        self.assertEqual(result.states[-1].wheels.left, 0)
        self.assertEqual(result.motions[-1].wheels.left, 19.99999)
        self.assertAlmostEqual(result.motions[-1].pose_at(1).x, result.states[-1].pose.x)

    def test_thin_wall_stops_with_partial_clock_and_zero_velocity(self):
        result = Simulator(load_config(ROOT / "configs" / "collision_wall.yaml")).run()
        self.assertEqual(result.status, "collision")
        self.assertEqual(len(result.states), 2)
        state = result.states[-1]
        self.assertAlmostEqual(state.time, 0.95, delta=1e-6)
        self.assertAlmostEqual(state.pose.x, 4.8, delta=1e-6)
        self.assertEqual(
            (state.wheels.left, state.wheels.right, state.vx, state.vy, state.omega),
            (0, 0, 0, 0, 0),
        )
        self.assertEqual(result.collisions[0].attempted_tick, 1)
        self.assertEqual(result.collisions[0].requested_wheels.left, 80)
        self.assertGreater(
            CollisionWorld(result.config.environment)
            .query(state.pose.position, result.config.robot.footprint_radius)
            .clearance,
            0,
        )

    def test_arc_contact_inside_full_loop(self):
        result = Simulator(load_config(ROOT / "configs" / "collision_loop.yaml")).run()
        self.assertEqual(result.status, "collision")
        expected = math.pi / 2 - 2 * math.asin(0.15)
        self.assertAlmostEqual(result.states[-1].time, expected, delta=2e-6)
        self.assertAlmostEqual(result.states[-1].pose.theta, expected, delta=2e-6)

    def test_clear_enabled_path_completes(self):
        config = RunConfig(
            robot={"initial_pose": {"x": 1, "y": 1}},
            simulation={"collision": {"mode": "stop"}},
            commands=[{"left": 1, "right": 1, "steps": 20}],
        )
        result = Simulator(config).run()
        self.assertEqual(result.status, "completed")
        self.assertEqual(result.collisions, ())
        self.assertEqual(len(result.states), 21)

    def test_initial_footprint_overlap_is_recorded(self):
        config = RunConfig(
            robot={"initial_pose": {"x": 0.1, "y": 1}},
            simulation={"collision": {"mode": "stop"}},
            commands=[{"left": 0, "right": 0, "steps": 20}],
        )
        result = Simulator(config).run()
        self.assertEqual(result.status, "collision")
        self.assertEqual(len(result.states), 1)
        self.assertEqual(result.states[0].time, 0)
        self.assertEqual(result.collisions[0].reason, "initial_contact")
        self.assertAlmostEqual(result.collisions[0].report.clearance, -0.1)

    def test_later_tick_collision_keeps_clock_consistent(self):
        config = RunConfig(
            robot={"initial_pose": {"x": 1, "y": 1}},
            environment={"width": 3, "height": 3},
            simulation={"dt": 1, "collision": {"mode": "stop"}},
            commands=[{"left": 20, "right": 20, "steps": 5}],
        )
        result = Simulator(config).run()
        self.assertEqual(result.collisions[0].attempted_tick, 2)
        self.assertAlmostEqual(result.states[-1].time, 1.8, delta=2e-6)
        times = [state.time for state in result.states]
        self.assertEqual(times[:2], [0, 1])
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))

    def test_collision_mode_disabled_preserves_original_behavior(self):
        config = load_config(ROOT / "configs" / "collision_wall.yaml").model_dump()
        config["simulation"]["collision"]["mode"] = "disabled"
        result = Simulator(RunConfig.model_validate(config)).run()
        self.assertEqual(result.status, "completed")
        self.assertEqual(result.states[-1].pose.x, 9)

    def test_collision_run_reruns_exactly(self):
        simulator = Simulator(load_config(ROOT / "configs" / "collision_loop.yaml"))
        self.assertEqual(simulator.run(), simulator.run())

    def test_export_records_uncertainty_and_can_be_reproduced(self):
        result = Simulator(load_config(ROOT / "configs" / "collision_wall.yaml")).run()
        with tempfile.TemporaryDirectory() as temp:
            directory = save_result(result, temp)
            meta = json.loads((directory / "metadata.json").read_text())
            events = json.loads((directory / "collisions.json").read_text())
            self.assertEqual(meta["format_version"], 2)
            self.assertEqual(meta["status"], "collision")
            self.assertEqual(meta["collision_count"], 1)
            self.assertAlmostEqual(meta["mass_properties"]["total_mass"], 10.5)
            self.assertEqual(events[0]["requested_wheels"]["left"], 80)
            self.assertEqual(events[0]["reason"], result.collisions[0].reason)
            self.assertEqual(Simulator(load_config(directory / "config.json")).run(), result)

    def test_cli_collision_exit_code_and_diagnostics(self):
        with tempfile.TemporaryDirectory() as temp:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = main(
                    ["simulate", str(ROOT / "configs" / "collision_wall.yaml"), "--output", temp]
                )
            self.assertEqual(code, 3)
            self.assertIn("obstacle:0", output.getvalue())
            self.assertTrue((Path(temp) / "collisions.json").is_file())

    def test_cli_validation_rejects_initial_contact(self):
        config = RunConfig(
            simulation={"collision": {"mode": "stop"}},
            commands=[{"left": 1, "right": 1, "steps": 1}],
        )
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "initial_contact.json"
            path.write_text(config.model_dump_json(), encoding="utf-8")
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                code = main(["validate", str(path)])
            self.assertEqual(code, 2)
            self.assertIn("initial footprint", stderr.getvalue())

    def test_substep_clock_validation(self):
        self.assertEqual(SimulationClock(0.1, 10).time_at_fraction(0.5), 1.05)
        for fraction in [-1, 2, True, float("nan")]:
            with self.subTest(fraction=fraction), self.assertRaises(ValueError):
                SimulationClock(0.1).time_at_fraction(fraction)
