import math
import unittest

import numpy as np

from roboforge.config import Environment, NoiseConfig, RunConfig
from roboforge.geometry import Vector2
from roboforge.sensors.noise import NoiseProcess
from roboforge.sensors.raycast import raycast
from roboforge.sensors.suite import SensorSuite
from roboforge.simulation import Simulator


class RayTests(unittest.TestCase):
    def test_cardinal_wall_ranges(self):
        env = Environment(width=10, height=8)
        for angle, expected in [(0, 8), (math.pi / 2, 5), (math.pi, 2), (-math.pi / 2, 3)]:
            self.assertAlmostEqual(raycast(env, Vector2(2, 3), angle, 20).distance, expected)

    def test_circle_rectangle_tangent_inside_and_occlusion(self):
        env = Environment(
            obstacles=[
                {"type": "circle", "x": 5, "y": 5, "radius": 1},
                {"type": "rectangle", "x": 7, "y": 4, "width": 1, "height": 2},
            ]
        )
        self.assertEqual(raycast(env, Vector2(1, 5), 0, 20).distance, 3)
        self.assertAlmostEqual(raycast(env, Vector2(1, 6), 0, 20).distance, 4)
        self.assertEqual(raycast(env, Vector2(5, 5), 0, 20).distance, 0)
        self.assertEqual(raycast(env, Vector2(7.5, 5), 0, 20).distance, 0)
        self.assertEqual(raycast(env, Vector2(9, 5), math.pi, 20).object_id, "obstacle:1")

    def test_max_range_no_hit(self):
        hit = raycast(Environment(), Vector2(5, 5), 0, 2)
        self.assertEqual((hit.distance, hit.object_id), (2, None))


class NoiseTests(unittest.TestCase):
    def test_seeded_distribution_and_bias(self):
        process = NoiseProcess(NoiseConfig(stddev=0.2, bias=0.5), 42, "test")
        values = np.array([process.sample(1, 0.01) for _ in range(20000)])
        self.assertAlmostEqual(float(values.mean()), 1.5, delta=0.006)
        self.assertAlmostEqual(float(values.std()), 0.2, delta=0.006)

    def test_repeat_and_dropout(self):
        a, b = [NoiseProcess(NoiseConfig(stddev=0.2, bias_walk=0.1), 42, "same") for _ in range(2)]
        self.assertEqual(
            [a.sample(0, 0.1) for _ in range(100)], [b.sample(0, 0.1) for _ in range(100)]
        )
        self.assertIsNone(NoiseProcess(NoiseConfig(dropout=1), 1, "drop").sample(0, 1))


class SensorTests(unittest.TestCase):
    def config(self, sensors, **kwargs):
        return RunConfig(
            robot={"initial_pose": {"x": 2, "y": 2}},
            commands=[{"left": 1, "right": 1, "steps": 100}],
            sensors=sensors,
            **kwargs,
        )

    def test_encoder_quantization_and_timestamps(self):
        result = Simulator(self.config([{"type": "encoder", "rate_hz": 10}])).run()
        self.assertEqual(len(result.readings), 11)
        last = result.readings[-1]
        self.assertEqual(last.capture_time, 1)
        self.assertEqual(last.left_ticks, round(2048 / math.tau))
        self.assertEqual(last.left_ticks, last.right_ticks)

    def test_sensor_rate_above_simulation_rate(self):
        result = Simulator(self.config([{"type": "encoder", "rate_hz": 250}])).run()
        self.assertEqual(len(result.readings), 251)
        self.assertEqual(result.readings[-1].capture_time, 1)

    def test_sensor_order_does_not_change_noise(self):
        encoder = {"type": "encoder", "noise": {"stddev": 0.01}}
        a = Simulator(self.config([encoder])).run()
        b = Simulator(self.config([{"type": "imu"}, encoder])).run()
        self.assertEqual(a.readings, tuple(r for r in b.readings if r.kind == "encoder"))

    def test_lidar_mount_and_single_forward_ray(self):
        result = Simulator(self.config([{"type": "lidar", "rays": 1, "rate_hz": 1}])).run()
        self.assertAlmostEqual(result.readings[0].ranges[0], 7.9)
        self.assertAlmostEqual(result.readings[-1].ranges[0], 7.85)
        self.assertTrue(result.readings[-1].hits[0])

    def test_invalid_lidar_returns_are_explicit(self):
        result = Simulator(
            self.config([{"type": "lidar", "rays": 4, "noise": {"dropout": 1}}])
        ).run()
        self.assertEqual(result.readings[0].ranges, (None,) * 4)
        self.assertFalse(any(result.readings[0].hits))

    def test_imu_steady_turn_and_centripetal_acceleration(self):
        config = RunConfig(
            robot={"initial_pose": {"x": 3, "y": 3}},
            sensors=[{"type": "imu", "rate_hz": 10}],
            commands=[{"left": 17, "right": 23, "steps": 100}],
        )
        last = Simulator(config).run().readings[-1]
        self.assertAlmostEqual(last.gyro_z, 1)
        self.assertAlmostEqual(last.acceleration_x, 0)
        self.assertAlmostEqual(last.acceleration_y, 1)

    def test_latency_delivery_is_causal_and_once(self):
        suite = SensorSuite(self.config([{"type": "encoder", "rate_hz": 10, "latency": 0.2}]))
        suite.capture_initial()
        self.assertEqual(suite.deliver(0.1), ())
        self.assertEqual(len(suite.deliver(0.2)), 1)
        self.assertEqual(suite.deliver(0.2), ())

    def test_capture_repeatability_and_no_truth_fields(self):
        config = self.config(
            [
                {"type": "encoder", "noise": {"stddev": 0.01}},
                {"type": "imu", "gyro_noise": {"stddev": 0.02}},
            ]
        )
        self.assertEqual(Simulator(config).run(), Simulator(config).run())
        self.assertNotIn("pose", Simulator(config).run().readings[0].model_dump())

    def test_invalid_sensor_configuration(self):
        for sensors in [
            [{"type": "encoder", "rate_hz": 0}],
            [{"type": "lidar", "min_range": 3, "max_range": 2}],
            [{"type": "imu", "frame": "missing"}],
            [{"type": "encoder"}, {"type": "encoder"}],
        ]:
            with self.subTest(sensors=sensors), self.assertRaises(ValueError):
                self.config(sensors)
