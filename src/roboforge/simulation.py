"""Synchronous fixed-step simulation; independent of plotting and wall-clock time."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Callable, Literal

from roboforge.actuators import ActuatorSample, WheelActuators
from roboforge.config import RunConfig
from roboforge.control import EncoderWheelController, WheelControlSample
from roboforge.core import finite, positive
from roboforge.faults import FaultEngine, FaultEvent
from roboforge.geometry import Pose2
from roboforge.physics import CollisionReport, CollisionWorld, KinematicMotion
from roboforge.robot import DifferentialDriveRobot, RobotState
from roboforge.robotics import BodyTwist2, WheelSpeeds
from roboforge.sensors import SensorReading
from roboforge.sensors.suite import SensorSuite


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
    readings: tuple[SensorReading, ...] = ()
    actuator_samples: tuple[ActuatorSample, ...] = ()
    control_samples: tuple[WheelControlSample, ...] = ()
    fault_events: tuple[FaultEvent, ...] = ()


class SimulationCancelled(Exception):
    """Cooperative cancellation at a simulation step boundary."""


class Simulator:
    """Run prescribed wheel commands with optional swept collision stopping.

    Default collision mode is disabled for v1-config compatibility. Stop mode
    halts conservatively; it is not a force/contact-response dynamics solver.
    Every run constructs fresh state and clock for deterministic repeatability.
    """

    def __init__(self, config: RunConfig) -> None:
        self.config = config

    def run(self, should_cancel: Callable[[], bool] | None = None) -> SimulationResult:
        if should_cancel is not None and should_cancel():
            raise SimulationCancelled("simulation cancelled")
        robot = DifferentialDriveRobot(self.config.robot)
        clock = SimulationClock(self.config.simulation.dt)
        state = robot.initial_state()
        states = [state]
        motions = []
        actuators = WheelActuators(self.config.actuators)
        actuator_samples = []
        faults = FaultEngine(self.config)
        sensors = SensorSuite(self.config) if self.config.sensors else None
        controller = (
            EncoderWheelController(self.config.wheel_controller)
            if self.config.wheel_controller
            else None
        )
        if sensors:
            sensors.capture_initial()
        collision = self.config.simulation.collision
        world = CollisionWorld(self.config.environment) if collision.mode == "stop" else None
        for command in self.config.commands:
            requested = WheelSpeeds(command.left, command.right)
            for _ in range(command.steps):
                if should_cancel is not None and should_cancel():
                    raise SimulationCancelled("simulation cancelled")
                actuator_request = (
                    controller.update(clock.time, requested, sensors.deliver(clock.time))
                    if controller
                    else requested
                )
                actuator_sample = actuators.step(
                    faults.command(actuator_request, clock.time), clock.dt, clock.time
                )
                shaft = faults.wheels(actuator_sample.applied, clock.time, "actuator_saturation")
                actuator_sample = replace(
                    actuator_sample,
                    requested=actuator_request,
                    applied=shaft,
                    speed_limited=(
                        actuator_sample.speed_limited[0]
                        or shaft.left != actuator_sample.applied.left,
                        actuator_sample.speed_limited[1]
                        or shaft.right != actuator_sample.applied.right,
                    ),
                )
                actuators.applied = shaft
                actuator_samples.append(actuator_sample)
                wheels = faults.wheels(shaft, clock.time, "wheel_slip")
                encoder_wheels = shaft if shaft != wheels else None
                motion = KinematicMotion(
                    state.pose,
                    robot.kinematics,
                    wheels,
                    clock.dt,
                    self.config.simulation.integrator,
                    encoder_wheels,
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
                                    encoder_wheels,
                                )
                            )
                            if sensors:
                                sensors.advance(motions[-1], start_time=clock.time)
                        event = CollisionEvent(
                            clock.tick + 1,
                            stop_time,
                            (clock.time_at_fraction(lower), clock.time_at_fraction(upper)),
                            clock.time_at_fraction(middle),
                            motion.pose_at(middle),
                            sweep.reason,
                            requested,
                            sweep.report,
                            sweep.queries,
                        )
                        return SimulationResult(
                            self.config,
                            tuple(states),
                            "collision",
                            (event,),
                            tuple(motions),
                            tuple(
                                sorted(sensors.readings, key=lambda r: (r.capture_time, r.sensor))
                            )
                            if sensors
                            else (),
                            tuple(actuator_samples),
                            tuple(controller.samples) if controller else (),
                            faults.events(stop_time),
                        )
                if sensors:
                    sensors.advance(motion, start_time=clock.time)
                clock = clock.advanced()
                state = robot.step(
                    state, wheels, clock.dt, clock.time, self.config.simulation.integrator
                )
                if shaft != wheels:
                    state = replace(state, wheels=shaft)
                states.append(state)
                motions.append(motion)
        return SimulationResult(
            self.config,
            tuple(states),
            motions=tuple(motions),
            readings=tuple(sorted(sensors.readings, key=lambda r: (r.capture_time, r.sensor)))
            if sensors
            else (),
            actuator_samples=tuple(actuator_samples),
            control_samples=tuple(controller.samples) if controller else (),
            fault_events=faults.events(clock.time),
        )
