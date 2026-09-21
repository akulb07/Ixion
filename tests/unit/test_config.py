"""Configuration failures must be explicit before simulation begins."""

import json
import tempfile
import unittest
from pathlib import Path

from pydantic import ValidationError

from roboforge.config import Environment, RobotConfig, RunConfig, load_config


def basic_config(**kwargs):
    return RunConfig(commands=[{"left": 1, "right": 1, "steps": 10}], **kwargs)


class ConfigTests(unittest.TestCase):
    def test_unquoted_scientific_notation_roundtrips_but_quoted_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "scientific.yaml"
            template = "simulation: {collision: {spatial_tolerance: %s}}\ncommands: [{left: 1, right: 1, steps: 1}]"
            path.write_text(template % "1e-6", encoding="utf-8")
            self.assertEqual(load_config(path).simulation.collision.spatial_tolerance, 1e-6)
            path.write_text(template % "'1e-6'", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "spatial_tolerance"):
                load_config(path)

    def test_valid_defaults_and_immutability(self):
        config = basic_config()
        self.assertEqual(config.robot.wheel_radius, 0.05)
        self.assertIsInstance(config.commands, tuple)
        with self.assertRaises(ValidationError):
            config.robot.wheel_radius = 0.1

    def test_invalid_dimensions(self):
        for field in ["wheel_radius", "wheel_separation", "footprint_radius"]:
            for value in [0, -1, float("nan"), float("inf"), True, "0.05"]:
                with (
                    self.subTest(field=field, value=value),
                    self.assertRaisesRegex(ValidationError, field),
                ):
                    RobotConfig(**{field: value})

    def test_unknown_fields_are_not_silently_ignored(self):
        with self.assertRaisesRegex(ValidationError, "wheel_raduis"):
            RobotConfig(wheel_raduis=0.05)
        with self.assertRaisesRegex(ValidationError, "integrater"):
            basic_config(simulation={"integrater": "exact"})

    def test_invalid_step_counts_and_seed(self):
        for steps in [0, -1, 1.5, True, "10"]:
            with self.subTest(steps=steps), self.assertRaises(ValidationError):
                RunConfig(commands=[{"left": 1, "right": 1, "steps": steps}])
        for seed in [-1, True, "42"]:
            with self.subTest(seed=seed), self.assertRaises(ValidationError):
                basic_config(seed=seed)

    def test_invalid_run_settings(self):
        for simulation in [{"dt": 0}, {"dt": -1}, {"dt": float("inf")}, {"integrator": "invalid"}]:
            with self.subTest(simulation=simulation), self.assertRaises(ValidationError):
                basic_config(simulation=simulation)
        with self.assertRaises(ValidationError):
            RunConfig(commands=[])
        with self.assertRaises(ValidationError):
            basic_config(schema_version=2)

    def test_environment_geometry(self):
        environment = Environment(
            width=5,
            height=5,
            obstacles=[
                {"type": "rectangle", "x": 0, "y": 0, "width": 2, "height": 1},
                {"type": "circle", "x": 4, "y": 4, "radius": 1},
            ],
        )
        self.assertEqual(len(environment.obstacles), 2)
        self.assertIsInstance(environment.obstacles, tuple)

    def test_invalid_obstacles_and_initial_pose(self):
        for obstacle in [
            {"type": "triangle", "x": 1, "y": 1},
            {"type": "circle", "x": 0, "y": 0, "radius": 1},
            {"type": "rectangle", "x": 0, "y": 0, "width": -1, "height": 1},
            {"type": "rectangle", "x": 4, "y": 4, "width": 2, "height": 2},
        ]:
            with self.subTest(obstacle=obstacle), self.assertRaises(ValidationError):
                Environment(width=5, height=5, obstacles=[obstacle])
        with self.assertRaisesRegex(ValidationError, "initial robot centre"):
            basic_config(robot={"initial_pose": {"x": -1, "y": 0}})

    def test_yaml_json_roundtrip(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "config.json"
            config = basic_config()
            path.write_text(config.model_dump_json(), encoding="utf-8")
            self.assertEqual(load_config(path), config)
            path.write_text("commands:\n  - {left: 1, right: 1, steps: 10}\n", encoding="utf-8")
            self.assertEqual(load_config(path), config)

    def test_bad_files_report_path_and_reason(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "bad.yaml"
            for text in [
                "",
                "commands: [",
                "commands: []\ncommands: []",
                "x: !!python/object:builtins.object {}",
                "1: value",
            ]:
                path.write_text(text, encoding="utf-8")
                with self.subTest(text=text), self.assertRaisesRegex(ValueError, "bad.yaml"):
                    load_config(path)
            with self.assertRaisesRegex(ValueError, "missing.yaml"):
                load_config(Path(temp) / "missing.yaml")

    def test_nonfinite_json_value_rejected(self):
        data = basic_config().model_dump()
        data["robot"]["wheel_radius"] = float("nan")
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "config.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "wheel_radius"):
                load_config(path)
