"""Cross-layer collision safety with delayed encoder feedback and actuator limits."""

from roboforge.config import RunConfig
from roboforge.geometry import Vector2
from roboforge.physics import CollisionWorld
from roboforge.simulation import Simulator


def test_feedback_collision_preserves_truth_measurement_and_request_boundaries():
    config = RunConfig.model_validate(
        {
            "robot": {"initial_pose": {"x": 1, "y": 1}},
            "environment": {
                "width": 4,
                "height": 4,
                "obstacles": [{"type": "rectangle", "x": 2, "y": 0, "width": 0.01, "height": 4}],
            },
            "simulation": {"collision": {"mode": "stop"}},
            "commands": [{"left": 20, "right": 20, "steps": 500}],
            "sensors": [{"type": "encoder", "rate_hz": 50, "latency": 0.03}],
            "wheel_controller": {},
            "actuators": {"delay_steps": 2, "left": {"max_speed": 5}, "right": {"max_speed": 5}},
        }
    )
    result = Simulator(config).run()
    assert result.status == "collision"
    final = result.states[-1]
    assert final.wheels.left == final.wheels.right == 0
    assert result.collisions[0].requested_wheels.left == 20
    assert result.actuator_samples[-1].applied.left == 5
    world = CollisionWorld(config.environment)
    assert all(not world.query(Vector2(s.pose.x, s.pose.y), 0.2).collision for s in result.states)
    assert all(r.capture_time <= final.time + 1e-12 for r in result.readings)
    assert all(s.capture_time + 0.03 <= s.time + 1e-12 for s in result.control_samples)
    assert len(result.motions) == len(result.states) - 1
    assert result == Simulator(config).run()
