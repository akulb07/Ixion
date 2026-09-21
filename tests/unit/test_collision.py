"""Analytical signed-distance, witness-point and swept-path references."""

import math
import unittest

import numpy as np

from roboforge.config import Environment
from roboforge.geometry import Pose2, Vector2
from roboforge.physics import CollisionWorld, KinematicMotion
from roboforge.robotics import DifferentialDrive, WheelSpeeds


def room(*obstacles):
    return CollisionWorld(Environment(width=10, height=10, obstacles=obstacles))


def line(start=Pose2(1, 5), speed=4, duration=2):
    return KinematicMotion(start, DifferentialDrive(1, 2), WheelSpeeds(speed, speed), duration)


class StaticCollisionTests(unittest.TestCase):
    def test_circle_separation_tangency_and_overlap(self):
        world = room({"type": "circle", "x": 5, "y": 5, "radius": 1})
        for x, expected in [(7, 0.5), (6.5, 0), (6.25, -0.25)]:
            with self.subTest(x=x):
                report = world.query(Vector2(x, 5), 0.5)
                self.assertAlmostEqual(report.clearance, expected)
                self.assertEqual(report.collision, expected <= 0)
                self.assertEqual(report.nearest.object_id, "obstacle:0")
                self.assertEqual(report.nearest.point_on_obstacle, Vector2(6, 5))
                self.assertEqual(report.nearest.normal, Vector2(1, 0))
                self.assertAlmostEqual(report.nearest.penetration, max(-expected, 0))

    def test_coincident_circle_has_deterministic_normal(self):
        report = room({"type": "circle", "x": 5, "y": 5, "radius": 1}).query(Vector2(5, 5), 0.5)
        self.assertEqual(report.clearance, -1.5)
        self.assertEqual(report.nearest.normal, Vector2(1, 0))
        self.assertEqual(report.nearest.point_on_robot, Vector2(4.5, 5))

    def test_rectangle_faces_corner_and_inside(self):
        world = room({"type": "rectangle", "x": 4, "y": 4, "width": 2, "height": 2})
        cases = [
            (Vector2(3, 5), 0.5, Vector2(-1, 0)),
            (Vector2(5, 7), 0.5, Vector2(0, 1)),
            (Vector2(3, 3), math.sqrt(2) - 0.5, Vector2(-1 / math.sqrt(2), -1 / math.sqrt(2))),
            (Vector2(5, 5), -1.5, Vector2(-1, 0)),
            (Vector2(4, 5), -0.5, Vector2(-1, 0)),
        ]
        for centre, expected, normal in cases:
            with self.subTest(centre=centre):
                report = world.query(centre, 0.5)
                self.assertAlmostEqual(report.clearance, expected)
                np.testing.assert_allclose(
                    report.nearest.normal.as_array(), normal.as_array(), atol=1e-14
                )
                pair = report.nearest.point_on_robot - report.nearest.point_on_obstacle
                np.testing.assert_allclose(
                    pair.as_array(), (expected * normal).as_array(), atol=1e-14
                )

    def test_rectangle_corner_is_rounded_not_square_inflation(self):
        world = room({"type": "rectangle", "x": 4, "y": 4, "width": 2, "height": 2})
        # Inside the expanded AABB, but outside the actual rounded corner.
        report = world.query(Vector2(3.6, 3.6), 0.5)
        self.assertFalse(report.collision)
        self.assertAlmostEqual(report.clearance, math.hypot(0.4, 0.4) - 0.5)

    def test_all_world_boundaries(self):
        for centre, name, normal in [
            (Vector2(0.25, 5), "left", Vector2(1, 0)),
            (Vector2(9.75, 5), "right", Vector2(-1, 0)),
            (Vector2(5, 0.25), "bottom", Vector2(0, 1)),
            (Vector2(5, 9.75), "top", Vector2(0, -1)),
        ]:
            with self.subTest(name=name):
                report = room().query(centre, 0.5)
                self.assertEqual(report.nearest.object_id, f"boundary:{name}")
                self.assertEqual(report.clearance, -0.25)
                self.assertEqual(report.nearest.normal, normal)

    def test_boundary_tangency_outside_and_multiple_contacts(self):
        self.assertTrue(room().query(Vector2(0.5, 5), 0.5).collision)
        report = room().query(Vector2(-1, -1), 0.5)
        self.assertEqual(report.minimum_distance, 0)
        self.assertEqual(report.clearance, -1.5)
        self.assertEqual(
            [contact.object_id for contact in report.contacts], ["boundary:left", "boundary:bottom"]
        )

    def test_multiple_overlaps_preserve_all_constraints(self):
        world = room(
            {"type": "circle", "x": 5, "y": 5, "radius": 1},
            {"type": "circle", "x": 5.5, "y": 5, "radius": 1},
        )
        report = world.query(Vector2(5.25, 5), 0.5)
        self.assertEqual(len(report.contacts), 2)
        self.assertEqual(report.nearest.object_id, "obstacle:0")

    def test_invalid_query_parameters(self):
        for radius in [0, -1, float("nan"), True, "1"]:
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                room().query(Vector2(5, 5), radius)
        with self.assertRaisesRegex(ValueError, "Vector2"):
            room().query((5, 5), 0.2)

    def test_seeded_signed_distance_is_lipschitz(self):
        world = room(
            {"type": "rectangle", "x": 2, "y": 3, "width": 3, "height": 2},
            {"type": "circle", "x": 7, "y": 7, "radius": 1},
        )
        rng = np.random.default_rng(617)
        for _ in range(200):
            a, b = Vector2(*rng.uniform(-1, 11, 2)), Vector2(*rng.uniform(-1, 11, 2))
            self.assertLessEqual(
                abs(world.query(a, 0.2).clearance - world.query(b, 0.2).clearance),
                (a - b).norm + 1e-12,
            )


class SweptCollisionTests(unittest.TestCase):
    def test_thin_wall_between_clear_endpoints(self):
        world = room({"type": "rectangle", "x": 5, "y": 4, "width": 0.01, "height": 2})
        motion = line()
        self.assertFalse(world.query(motion.start.position, 0.2).collision)
        self.assertFalse(world.query(motion.pose_at(1).position, 0.2).collision)
        sweep = world.sweep(motion, 0.2)
        self.assertTrue(sweep.blocked)
        self.assertAlmostEqual(sweep.safe_fraction, (4.8 - 1) / 8, delta=1e-6 / 8)
        self.assertGreater(
            world.query(motion.pose_at(sweep.safe_fraction).position, 0.2).clearance, 0
        )
        self.assertLessEqual(motion.travel * (sweep.interval[1] - sweep.interval[0]), 1e-6)

    def test_circle_entry_time(self):
        world = room({"type": "circle", "x": 5, "y": 5, "radius": 0.5})
        sweep = world.sweep(line(), 0.25)
        self.assertTrue(sweep.blocked)
        self.assertAlmostEqual(sweep.safe_fraction, (4.25 - 1) / 8, delta=1e-6 / 8)

    def test_boundary_crossing_and_reverse_motion(self):
        sweep = room().sweep(line(Pose2(1, 5), -2, 1), 0.2)
        self.assertTrue(sweep.blocked)
        self.assertEqual(sweep.report.nearest.object_id, "boundary:left")
        self.assertAlmostEqual(sweep.safe_fraction, 0.4, delta=1e-6)

    def test_tangent_sweep_is_not_missed(self):
        world = room({"type": "circle", "x": 5, "y": 5, "radius": 0.5})
        sweep = world.sweep(line(Pose2(1, 6)), 0.5, spatial_tolerance=1e-5)
        self.assertTrue(sweep.blocked)
        # At tangency, time uncertainty scales as sqrt(spatial tolerance).
        self.assertLessEqual(abs(sweep.safe_fraction - 0.5), 0.001)
        self.assertLessEqual(abs(sweep.report.clearance), 1e-5)

    def test_clear_near_miss_is_certified(self):
        world = room({"type": "circle", "x": 5, "y": 5, "radius": 0.5})
        sweep = world.sweep(line(Pose2(1, 6.01)), 0.5)
        self.assertFalse(sweep.blocked)
        self.assertEqual(sweep.safe_fraction, 1)

    def test_full_loop_with_same_start_and_end_is_checked(self):
        world = room({"type": "circle", "x": 6, "y": 6, "radius": 0.1})
        motion = KinematicMotion(Pose2(5, 5), DifferentialDrive(1, 2), WheelSpeeds(0, 2), math.tau)
        sweep = world.sweep(motion, 0.1)
        expected = (math.pi / 2 - 2 * math.asin(0.1)) / math.tau
        self.assertTrue(sweep.blocked)
        self.assertAlmostEqual(sweep.safe_fraction, expected, delta=1e-6 / motion.travel)

    def test_arc_does_not_use_straight_chord(self):
        world = room({"type": "circle", "x": 5, "y": 6, "radius": 0.1})
        motion = KinematicMotion(Pose2(5, 5), DifferentialDrive(1, 2), WheelSpeeds(0, 2), math.pi)
        self.assertFalse(world.sweep(motion, 0.1).blocked)

    def test_euler_sweep_matches_its_straight_update(self):
        world = room({"type": "rectangle", "x": 5.4, "y": 4.8, "width": 0.01, "height": 0.4})
        motion = KinematicMotion(
            Pose2(5, 5), DifferentialDrive(1, 2), WheelSpeeds(0, 2), 1, "euler"
        )
        self.assertTrue(world.sweep(motion, 0.1).blocked)

    def test_stationary_and_spin_only(self):
        for wheels in [WheelSpeeds(0, 0), WheelSpeeds(-1, 1)]:
            motion = KinematicMotion(Pose2(5, 5), DifferentialDrive(1, 2), wheels, 10)
            sweep = room().sweep(motion, 0.5)
            self.assertFalse(sweep.blocked)
            self.assertEqual(sweep.queries, 1)

    def test_initial_contact_returns_immediately(self):
        sweep = room().sweep(line(Pose2(0.1, 5)), 0.2)
        self.assertEqual(sweep.reason, "initial_contact")
        self.assertEqual(sweep.safe_fraction, 0)
        self.assertEqual(sweep.interval, (0, 0))

    def test_query_budget_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "not certified clear"):
            room().sweep(line(), 0.2, max_queries=1)

    def test_invalid_parameters_and_unresolvable_tolerance(self):
        for kwargs in [
            {"spatial_tolerance": 0},
            {"spatial_tolerance": 1e-30},
            {"max_queries": True},
            {"max_queries": 0},
        ]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                room().sweep(line(), 0.2, **kwargs)
        for fraction in [-1, 2, float("nan")]:
            with self.subTest(fraction=fraction), self.assertRaises(ValueError):
                line().pose_at(fraction)
        with self.assertRaises(ValueError):
            KinematicMotion(Pose2(), DifferentialDrive(1, 2), WheelSpeeds(1, 1), -1)

    def test_repeatability(self):
        world = room({"type": "circle", "x": 5, "y": 5, "radius": 0.5})
        self.assertEqual(world.sweep(line(), 0.2), world.sweep(line(), 0.2))
