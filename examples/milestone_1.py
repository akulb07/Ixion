"""One-command, deterministic demonstration; prints machine-readable JSON.

Run from a source checkout with: python examples/milestone_1.py
This demonstrates mathematical primitives, not a simulation engine.
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np

from roboforge import __version__
from roboforge.geometry import Pose2, Transform2, Vector2
from roboforge.robotics import BodyTwist2, DifferentialDrive, WheelSpeeds


def demonstrate() -> dict:
    drive = DifferentialDrive(0.05, 0.30)
    origin = Pose2()
    scenarios = [
        ("straight_10s", WheelSpeeds(10, 10), 10, [5, 0, 0]),
        ("spin_90deg", WheelSpeeds(-3, 3), math.pi / 2, [0, 0, math.pi / 2]),
        ("left_quarter_circle", WheelSpeeds(2, 4), 3 * math.pi / 2, [0.45, 0.45, math.pi / 2]),
        ("right_quarter_circle", WheelSpeeds(4, 2), 3 * math.pi / 2, [0.45, -0.45, -math.pi / 2]),
        ("stationary", WheelSpeeds(0, 0), 10, [0, 0, 0]),
    ]
    results = {}
    for name, wheels, duration, expected in scenarios:
        actual = drive.integrate(origin, wheels, duration).as_array()
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=0)
        results[name] = {
            "wheel_rates_rad_s": [wheels.left, wheels.right],
            "duration_s": duration,
            "pose": actual.tolist(),
            "expected_pose": expected,
            "max_absolute_component_error": float(np.max(np.abs(actual - expected))),
        }
    pose = Pose2()
    for _ in range(4):
        pose = drive.integrate(pose, drive.inverse(BodyTwist2(0.5, 0)), 2)
        pose = drive.integrate(pose, drive.inverse(BodyTwist2(0, math.pi / 2)), 1)
    np.testing.assert_allclose(pose.as_array(), [0, 0, 0], atol=1e-12, rtol=0)
    world_base = Transform2.from_pose(
        Pose2(2, 3, math.pi / 2), target_frame="world", source_frame="base"
    )
    base_lidar = Transform2("base", "lidar", Vector2(0.2, 0))
    point = (world_base @ base_lidar).apply_point(Vector2(1, 0))
    np.testing.assert_allclose(point.as_array(), [2, 4.2], atol=1e-12, rtol=0)
    return {
        "roboforge_version": __version__,
        "numpy_version": np.__version__,
        "randomness": "none",
        "units": {"position": "m", "heading": "rad", "time": "s"},
        "robot": {"wheel_radius_m": 0.05, "wheel_separation_m": 0.30},
        "scenarios": results,
        "square_final_pose": pose.as_array().tolist(),
        "square_position_error_m": pose.position.norm,
        "frame_example": {
            "T_world_base": world_base.matrix.tolist(),
            "T_base_lidar": base_lidar.matrix.tolist(),
            "lidar_point_in_world_m": point.as_array().tolist(),
        },
        "vector_example": {
            "vector": [3, 4],
            "norm": Vector2(3, 4).norm,
            "unit_vector": Vector2(3, 4).normalized().as_array().tolist(),
        },
    }


if __name__ == "__main__":
    print(json.dumps(demonstrate(), indent=2, allow_nan=False))
