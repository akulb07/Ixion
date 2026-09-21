"""Validate swept wall/arc contacts and export reproducible collision demos."""

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.config import load_config
from roboforge.io import save_result
from roboforge.physics import CollisionWorld, KinematicMotion
from roboforge.robot import DifferentialDriveRobot
from roboforge.robotics import WheelSpeeds
from roboforge.simulation import Simulator
from roboforge.visualization import plot_trajectory


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_2")
    args = parser.parse_args()
    summary = {}
    references = {"wall": 0.95, "loop": math.pi / 2 - 2 * math.asin(0.15)}
    for name, expected_time in references.items():
        config = load_config(ROOT / "configs" / f"collision_{name}.yaml")
        robot = DifferentialDriveRobot(config.robot)
        command = config.commands[0]
        motion = KinematicMotion(
            config.robot.initial_pose.to_pose(),
            robot.kinematics,
            WheelSpeeds(command.left, command.right),
            config.simulation.dt,
        )
        world = CollisionWorld(config.environment)
        endpoints_clear = all(
            not world.query(motion.pose_at(f).position, config.robot.footprint_radius).collision
            for f in (0, 1)
        )
        if not endpoints_clear:
            raise AssertionError("Demo must have clear endpoints to demonstrate swept checks")
        result = Simulator(config).run()
        if result.status != "collision":
            raise AssertionError("The swept path must hit the configured obstacle")
        final = result.states[-1]
        tolerance = config.simulation.collision.spatial_tolerance
        # Crossings are transverse in these examples: normal speed ~1 or 4 m/s.
        if abs(final.time - expected_time) > 2 * tolerance:
            raise AssertionError("Stop time differs from the analytical contact beyond tolerance")
        clearance = world.query(final.pose.position, config.robot.footprint_radius).clearance
        if not 0 < clearance < 2 * tolerance:
            raise AssertionError("Robot must stop just before contact without penetration")
        if result != Simulator(config).run():
            raise AssertionError("Repeated collision runs must be identical")
        destination = args.output / name
        save_result(result, destination)
        plot_trajectory(result, destination / "trajectory.png")
        summary[name] = {
            "expected_first_contact_s": expected_time,
            "actual_safe_stop_s": final.time,
            "absolute_time_error_s": abs(final.time - expected_time),
            "safe_stop_pose": final.pose.as_array().tolist(),
            "clearance_m": clearance,
            "reason": result.collisions[0].reason,
            "unresolved_interval_s": result.collisions[0].interval,
            "sweep_queries": result.collisions[0].queries,
            "endpoints_clear": endpoints_clear,
            "deterministic_repeat": True,
        }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Artifacts: {args.output.resolve()}")


if __name__ == "__main__":
    main()
