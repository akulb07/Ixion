"""Analytical and seeded algebraic checks for planar geometry."""

import math
import unittest
from dataclasses import FrozenInstanceError

import numpy as np

from roboforge.core import finite, nonnegative, positive, wrap_angle
from roboforge.geometry import Pose2, Transform2, Vector2


class ScalarTests(unittest.TestCase):
    def test_nonfinite_and_nonreal_inputs(self):
        for value in [float("nan"), float("inf"), -float("inf"), True, "1", 1j, None, 10**400]:
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "finite real"):
                finite(value, "input")

    def test_physical_dimension_validation(self):
        for value in [0, -1]:
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "greater than zero"):
                positive(value, "wheel radius")
        self.assertEqual(nonnegative(0, "dt"), 0)
        with self.assertRaisesRegex(ValueError, "greater than or equal"):
            nonnegative(-1, "dt")

    def test_angle_boundary_and_periodicity(self):
        self.assertEqual(wrap_angle(math.pi), -math.pi)
        self.assertEqual(wrap_angle(-math.pi), -math.pi)
        self.assertEqual(wrap_angle(0), 0)
        for turns in range(-20, 21):
            self.assertAlmostEqual(wrap_angle(0.3 + turns * math.tau), 0.3, places=12)

    def test_tiny_angle_preserved(self):
        self.assertEqual(wrap_angle(1e-18), 1e-18)
        self.assertEqual(wrap_angle(-1e-18), -1e-18)


class VectorTests(unittest.TestCase):
    def test_arithmetic(self):
        a, b = Vector2(3, 4), Vector2(-1, 2)
        self.assertEqual(a + b, Vector2(2, 6))
        self.assertEqual(a - b, Vector2(4, 2))
        self.assertEqual(-a, Vector2(-3, -4))
        self.assertEqual(2 * a, Vector2(6, 8))
        self.assertEqual(a / 2, Vector2(1.5, 2))
        self.assertEqual(a.dot(b), 5)
        self.assertEqual(a.cross(b), 10)
        self.assertEqual(b.cross(a), -10)

    def test_norm_and_unit_vector(self):
        self.assertEqual(Vector2(3, 4).norm, 5)
        self.assertAlmostEqual(Vector2(3, 4).normalized().norm, 1)
        self.assertTrue(math.isfinite(Vector2(1e200, 1e200).norm))

    def test_undefined_operations(self):
        with self.assertRaisesRegex(ValueError, "zero vector"):
            Vector2(0, 0).normalized()
        with self.assertRaisesRegex(ValueError, "nonzero"):
            Vector2(1, 0) / 0
        with self.assertRaisesRegex(ValueError, "finite"):
            Vector2(1, 0) * float("inf")

    def test_rotation_orientation(self):
        np.testing.assert_allclose(
            Vector2(1, 0).rotated(math.pi / 2).as_array(), [0, 1], atol=1e-15
        )
        np.testing.assert_allclose(
            Vector2(1, 0).rotated(-math.pi / 2).as_array(), [0, -1], atol=1e-15
        )

    def test_immutable_and_array_copy(self):
        vector = Vector2(3, 4)
        with self.assertRaises(FrozenInstanceError):
            vector.x = 9
        array = vector.as_array()
        array[0] = 99
        self.assertEqual(vector.x, 3)

    def test_invalid_components(self):
        for value in [float("nan"), float("inf"), "2", True]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                Vector2(value, 0)


class PoseTests(unittest.TestCase):
    def test_origin_and_wrapping(self):
        self.assertEqual(Pose2(), Pose2(0, 0, 0))
        self.assertEqual(Pose2(1, 2, 3 * math.pi).theta, -math.pi)
        self.assertEqual(Pose2(1, 2).position, Vector2(1, 2))

    def test_immutable_and_array_copy(self):
        pose = Pose2(1, 2, 0.3)
        with self.assertRaises(FrozenInstanceError):
            pose.theta = 0
        pose.as_array()[0] = 7
        self.assertEqual(pose.x, 1)

    def test_invalid_pose(self):
        for args in [(float("nan"), 0, 0), (0, float("inf"), 0), (0, 0, True)]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                Pose2(*args)


class TransformTests(unittest.TestCase):
    def setUp(self):
        self.transform = Transform2("world", "base", Vector2(2, 3), math.pi / 2)

    def test_point_versus_vector(self):
        np.testing.assert_allclose(self.transform.apply_point(Vector2(1, 0)).as_array(), [2, 4])
        np.testing.assert_allclose(
            self.transform.apply_vector(Vector2(1, 0)).as_array(), [0, 1], atol=1e-15
        )

    def test_homogeneous_matrix(self):
        np.testing.assert_allclose(
            self.transform.matrix, [[0, -1, 2], [1, 0, 3], [0, 0, 1]], atol=1e-15
        )

    def test_transform_pose(self):
        result = self.transform.apply_pose(Pose2(1, 0, math.pi / 2))
        np.testing.assert_allclose(result.as_array(), [2, 4, -math.pi])

    def test_composition_known_answer(self):
        base_lidar = Transform2("base", "lidar", Vector2(0.2, 0), -math.pi / 2)
        world_lidar = self.transform @ base_lidar
        self.assertEqual((world_lidar.target_frame, world_lidar.source_frame), ("world", "lidar"))
        np.testing.assert_allclose(world_lidar.apply_point(Vector2(1, 0)).as_array(), [3, 3.2])
        np.testing.assert_allclose(
            world_lidar.matrix, self.transform.matrix @ base_lidar.matrix, atol=1e-14
        )

    def test_frame_mismatch(self):
        with self.assertRaisesRegex(ValueError, "cannot compose frames"):
            self.transform @ Transform2("imu", "lidar")

    def test_inverse_and_identity(self):
        inverse = self.transform.inverse()
        self.assertEqual((inverse.target_frame, inverse.source_frame), ("base", "world"))
        np.testing.assert_allclose((self.transform @ inverse).matrix, np.eye(3), atol=1e-14)
        np.testing.assert_allclose((inverse @ self.transform).matrix, np.eye(3), atol=1e-14)
        self.assertEqual(Transform2.identity("base").apply_point(Vector2(3, 4)), Vector2(3, 4))

    def test_pose_and_matrix_roundtrip(self):
        transform = Transform2.from_pose(
            Pose2(2, 3, 0.7), target_frame="world", source_frame="base"
        )
        restored = Transform2.from_matrix(
            transform.matrix, target_frame="world", source_frame="base"
        )
        np.testing.assert_allclose(restored.matrix, transform.matrix, atol=1e-14)

    def test_matrix_copy(self):
        self.transform.matrix[0, 2] = 100
        self.assertEqual(self.transform.translation.x, 2)

    def test_reject_non_rigid_matrices(self):
        invalid = [
            np.eye(2),
            np.ones((3, 3)),
            np.diag([-1, 1, 1]),
            np.diag([2, 1, 1]),
            [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]],
            np.full((3, 3), np.nan),
            np.eye(3, dtype=complex),
            [["1", "0", "0"], ["0", "1", "0"], ["0", "0", "1"]],
            [[1], [2, 3]],
            np.diag([1e300, 1, 1]),
        ]
        for matrix in invalid:
            with self.subTest(matrix=matrix), self.assertRaises(ValueError):
                Transform2.from_matrix(matrix, target_frame="a", source_frame="b")

    def test_matrix_tolerance(self):
        matrix = np.eye(3)
        matrix[0, 0] += 1e-12
        Transform2.from_matrix(matrix, target_frame="a", source_frame="b")
        with self.assertRaises(ValueError):
            Transform2.from_matrix(matrix, target_frame="a", source_frame="b", atol=1e-14)
        for tolerance in [0, -1, float("nan"), 0.1]:
            with self.subTest(tolerance=tolerance), self.assertRaises(ValueError):
                Transform2.from_matrix(
                    np.eye(3), target_frame="a", source_frame="b", atol=tolerance
                )

    def test_invalid_frame_and_translation(self):
        for args in [("", "base"), ("world", "  "), (None, "base"), ("a", "b", (1, 2))]:
            with self.subTest(args=args), self.assertRaises(ValueError):
                Transform2(*args)

    def test_seeded_composition_and_inverse_properties(self):
        rng = np.random.default_rng(42)
        for _ in range(100):
            a = Transform2.from_pose(
                Pose2(*rng.uniform(-10, 10, 3)), target_frame="world", source_frame="base"
            )
            b = Transform2.from_pose(
                Pose2(*rng.uniform(-10, 10, 3)), target_frame="base", source_frame="lidar"
            )
            p = Vector2(*rng.uniform(-10, 10, 2))
            np.testing.assert_allclose(
                (a @ b).apply_point(p).as_array(),
                a.apply_point(b.apply_point(p)).as_array(),
                atol=1e-12,
                rtol=0,
            )
            np.testing.assert_allclose(
                a.inverse().apply_point(a.apply_point(p)).as_array(),
                p.as_array(),
                atol=1e-12,
                rtol=0,
            )
