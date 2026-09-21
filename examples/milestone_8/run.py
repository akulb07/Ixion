"""Known-map A* navigation using delivered encoder odometry and Pure Pursuit."""

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.actuators import WheelActuators
from roboforge.config import RunConfig
from roboforge.geometry import Vector2
from roboforge.odometry import EncoderOdometry
from roboforge.physics import CollisionWorld, KinematicMotion
from roboforge.planning import grid_plan
from roboforge.robot import DifferentialDriveRobot
from roboforge.sensors.readings import EncoderReading
from roboforge.sensors.suite import SensorSuite
from roboforge.tracking import PurePursuit


def run():
    config = RunConfig.model_validate(
        {
            "name": "encoder_navigation",
            "robot": {"initial_pose": {"x": 1, "y": 1}, "footprint_radius": 0.15},
            "environment": {
                "width": 5,
                "height": 5,
                "obstacles": [{"type": "rectangle", "x": 2, "y": 0, "width": 0.1, "height": 3.5}],
            },
            "commands": [{"left": 0, "right": 0, "steps": 1}],
            "simulation": {"dt": 0.02},
            "sensors": [{"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 65536}],
            "actuators": {"left": {"time_constant": 0.05}, "right": {"time_constant": 0.05}},
        }
    )
    plan = grid_plan(config.environment, Vector2(1, 1), Vector2(4, 1), 0.4, 0.25)
    assert plan.status == "success"
    tracker = PurePursuit(plan.path, lookahead=0.25, max_speed=0.25)
    robot, sensors = DifferentialDriveRobot(config.robot), SensorSuite(config)
    actuators = WheelActuators(config.actuators)
    odometry = EncoderOdometry(0.05, 0.3, config.robot.initial_pose.to_pose())
    world = CollisionWorld(config.environment)
    state, estimate = robot.initial_state(), config.robot.initial_pose.to_pose()
    sensors.capture_initial()
    history = []
    dt = config.simulation.dt
    for tick in range(4000):
        time = tick * dt
        for reading in sensors.deliver(time):
            if isinstance(reading, EncoderReading):
                measured = odometry.update(reading)
                if measured is not None:
                    estimate = measured.pose
        tracking = tracker.update(estimate)
        history.append(
            {
                "time": time,
                "truth": asdict(state.pose),
                "estimate": asdict(estimate),
                "tracking": asdict(tracking),
            }
        )
        if tracking.reached:
            break
        applied = actuators.step(robot.kinematics.inverse(tracking.command), dt, time).applied
        motion = KinematicMotion(state.pose, robot.kinematics, applied, dt)
        if world.sweep(motion, config.robot.footprint_radius).blocked:
            raise AssertionError("navigation demonstration collided")
        sensors.advance(motion, start_time=time)
        state = robot.step(state, applied, dt, (tick + 1) * dt, "exact")
    assert tracking.reached, "navigation did not reach the goal within budget"
    return config, plan, history


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_8")
    args = parser.parse_args()
    config, plan, history = run()
    assert (config, plan, history) == run()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "config.json").write_text(config.model_dump_json(indent=2), encoding="utf-8")
    (args.output / "navigation.json").write_text(json.dumps(history, indent=2), encoding="utf-8")
    (args.output / "plan.json").write_text(json.dumps(asdict(plan), indent=2), encoding="utf-8")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.add_patch(Rectangle((2, 0), 0.1, 3.5, color="gray"))
    ax.plot(
        [p.x for p in plan.path], [p.y for p in plan.path], "--", label="A* path (0.4 m clearance)"
    )
    ax.plot(
        [h["truth"]["x"] for h in history],
        [h["truth"]["y"] for h in history],
        label="Executed path",
    )
    ax.set(
        xlim=(0, 5),
        ylim=(0, 5),
        xlabel="x [m]",
        ylabel="y [m]",
        title="RoboForge · encoder-guided Pure Pursuit",
    )
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output / "navigation.png", dpi=150)
    plt.close(fig)
    summary = {
        "reached": True,
        "collision_free": True,
        "deterministic_repeat": True,
        "duration_s": history[-1]["time"],
        "estimated_goal_error_m": history[-1]["tracking"]["goal_distance"],
    }
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
