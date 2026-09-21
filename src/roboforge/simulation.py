"""Synchronous fixed-step simulation; independent of plotting and wall-clock time."""

from __future__ import annotations

from dataclasses import dataclass

from roboforge.config import RunConfig
from roboforge.core import finite, positive
from roboforge.robot import DifferentialDriveRobot, RobotState
from roboforge.robotics import WheelSpeeds


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


@dataclass(frozen=True, slots=True)
class SimulationResult:
    config: RunConfig
    states: tuple[RobotState, ...]


class Simulator:
    """Run prescribed wheel commands in a passive environment.

    Obstacles/bounds do not stop the robot in Milestone 1. Each run constructs
    fresh state and clock, so repeated calls on the same instance are identical.
    """

    def __init__(self, config: RunConfig) -> None:
        self.config = config

    def run(self) -> SimulationResult:
        robot = DifferentialDriveRobot(self.config.robot)
        clock = SimulationClock(self.config.simulation.dt)
        state = robot.initial_state()
        states = [state]
        for command in self.config.commands:
            wheels = WheelSpeeds(command.left, command.right)
            for _ in range(command.steps):
                clock = clock.advanced()
                state = robot.step(
                    state, wheels, clock.dt, clock.time, self.config.simulation.integrator
                )
                states.append(state)
        return SimulationResult(self.config, tuple(states))
