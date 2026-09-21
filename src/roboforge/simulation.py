"""Synchronous fixed-step simulation; independent of plotting and wall-clock time."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from roboforge.config import RunConfig
from roboforge.core import finite, positive
from roboforge.geometry import Pose2
from roboforge.physics import CollisionReport, CollisionWorld, KinematicMotion
from roboforge.robot import DifferentialDriveRobot, RobotState
from roboforge.robotics import BodyTwist2, WheelSpeeds


@dataclass(frozen=True, slots=True)
class SimulationClock:
    """Time = integer tick * dt, avoiding accumulated floating-point additions."""

    dt: float
    tick: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "dt", positive(self.dt, "clock timestep"))
        if isinstance(self.tick, bool) or not isinstance(self.tick, int) or self.tick < 0:
            raise ValueError("clock tick must be a nonnegative integer")
        finite(self.tick * self.dt, "clock time")

    @property
    def time(self) -> float:
        return self.tick * self.dt

    @property
    def frequency(self) -> float:
        return finite(1.0 / self.dt, "simulation frequency")

    def advanced(self) -> SimulationClock:
        return SimulationClock(self.dt, self.tick + 1)

    def time_at_fraction(self, fraction: float) -> float:
        """Timestamp inside the next step, for a terminal collision interruption."""
        fraction = finite(fraction, "clock fraction")
        if not 0 <= fraction <= 1:
            raise ValueError("clock fraction must be in [0, 1]")
        return finite((self.tick + fraction) * self.dt, "clock time")


@dataclass(frozen=True, slots=True)
class CollisionEvent:
    """Terminal contact diagnostics; candidate report can differ from safe stop."""

    attempted_tick: int
    time: float
    interval: tuple[float, float]
    sample_time: float
    candidate_pose: Pose2
    reason: str
    requested_wheels: WheelSpeeds
    report: CollisionReport
    queries: int


@dataclass(frozen=True, slots=True)
class SimulationResult:
    config: RunConfig
    states: tuple[RobotState, ...]
    status: Literal["completed", "collision"] = "completed"
    collisions: tuple[CollisionEvent, ...] = ()
    motions: tuple[KinematicMotion, ...] = ()


class Simulator:
    """Run prescribed wheel commands with optional swept collision stopping.

    Default collision mode is disabled for v1-config compatibility. Stop mode
    halts conservatively; it is not a force/contact-response dynamics solver.
    Every run constructs fresh state and clock for deterministic repeatability.
    """

    def __init__(self, config: RunConfig) -> None:
        self.config = config

    def run(self) -> SimulationResult:
        robot = DifferentialDriveRobot(self.config.robot)
        clock = SimulationClock(self.config.simulation.dt)
        state = robot.initial_state()
        states = [state]
        motions = []
        collision = self.config.simulation.collision
        world = CollisionWorld(self.config.environment) if collision.mode == "stop" else None
        for command in self.config.commands:
            wheels = WheelSpeeds(command.left, command.right)
            for _ in range(command.steps):
                motion = KinematicMotion(
                    state.pose,
                    robot.kinematics,
                    wheels,
                    clock.dt,
                    self.config.simulation.integrator,
                )
                if world is not None:
                    sweep = world.sweep(
                        motion,
                        self.config.robot.footprint_radius,
                        spatial_tolerance=collision.spatial_tolerance,
                        max_queries=collision.max_queries,
                    )
                    if sweep.blocked:
                        lower, upper = sweep.interval
                        middle = (lower + upper) / 2
                        stop_time = clock.time_at_fraction(sweep.safe_fraction)
                        stopped = RobotState(
                            motion.pose_at(sweep.safe_fraction),
                            WheelSpeeds(0, 0),
                            BodyTwist2(0, 0),
                            stop_time,
                        )
                        if stopped.time == states[-1].time:
                            states[-1] = stopped
                        else:
                            states.append(stopped)
                            motions.append(
                                KinematicMotion(
                                    state.pose,
                                    robot.kinematics,
                                    wheels,
                                    clock.dt * sweep.safe_fraction,
                                    self.config.simulation.integrator,
                                )
                            )
                        event = CollisionEvent(
                            clock.tick + 1,
                            stop_time,
                            (clock.time_at_fraction(lower), clock.time_at_fraction(upper)),
                            clock.time_at_fraction(middle),
                            motion.pose_at(middle),
                            sweep.reason,
                            wheels,
                            sweep.report,
                            sweep.queries,
                        )
                        return SimulationResult(
                            self.config, tuple(states), "collision", (event,), tuple(motions)
                        )
                clock = clock.advanced()
                state = robot.step(
                    state, wheels, clock.dt, clock.time, self.config.simulation.integrator
                )
                states.append(state)
                motions.append(motion)
        return SimulationResult(self.config, tuple(states), motions=tuple(motions))
