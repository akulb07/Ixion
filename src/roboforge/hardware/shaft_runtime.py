"""Incremental shaft dynamics for host-side control experiments."""

from roboforge.config import Schema
from roboforge.core import finite, positive

from .battery import BatteryState
from .driver import DriverInputs
from .powertrain import MotorChannel, Powertrain
from .shaft_dynamics import ShaftLoad, ShaftSample, run_shaft_bench


class ShaftStep(Schema):
    start: ShaftSample
    end: ShaftSample
    inputs: tuple[DriverInputs, ...]


class ShaftRuntime:
    """Advance one explicit-Euler interval with commands held over that interval.

    Failed steps leave all runtime state unchanged. No history is retained;
    callers decide what to record and when to run their control logic.
    """

    def __init__(
        self,
        system: Powertrain,
        state: BatteryState,
        loads: tuple[ShaftLoad, ...],
        vcc_v: float = 3.3,
        auxiliary_current_a: float = 0,
    ):
        if len(loads) != len(system.channels):
            raise ValueError("provide exactly one shaft load per motor channel")
        system.operating_point(state, vcc_v, auxiliary_current_a)
        self._system = system
        self._state = state
        self._loads = loads
        self._vcc = vcc_v
        self._auxiliary = auxiliary_current_a
        self._time = 0.0
        self._angles = (0.0,) * len(loads)

    @property
    def time_s(self) -> float:
        return self._time

    @property
    def battery_state(self) -> BatteryState:
        return self._state

    @property
    def system(self) -> Powertrain:
        return self._system

    def step(self, duration_s: float, inputs: tuple[DriverInputs, ...] | None = None) -> ShaftStep:
        duration = positive(duration_s, "runtime step")
        end_time = finite(self._time + duration, "runtime time")
        if end_time <= self._time:
            raise ValueError("runtime step is too small to advance simulation time")
        commands = tuple(c.inputs for c in self._system.channels) if inputs is None else inputs
        if len(commands) != len(self._system.channels):
            raise ValueError("provide one driver command per motor channel")
        channels = tuple(
            MotorChannel(motor=c.motor, inputs=command, output_speed_rad_s=c.output_speed_rad_s)
            for c, command in zip(self._system.channels, commands)
        )
        commanded = Powertrain(
            battery=self._system.battery, driver=self._system.driver, channels=channels
        )
        local = run_shaft_bench(
            commanded, self._state, self._loads, duration, duration, self._vcc, self._auxiliary
        )

        def absolute(sample, time):
            return ShaftSample(
                time_s=time,
                state=sample.state,
                displacement_rad=tuple(
                    finite(a + b, "runtime displacement")
                    for a, b in zip(self._angles, sample.displacement_rad)
                ),
                speed_rad_s=sample.speed_rad_s,
                acceleration_rad_s2=sample.acceleration_rad_s2,
                point=sample.point,
            )

        start = absolute(local.samples[0], self._time)
        end = absolute(local.samples[-1], end_time)
        updated = Powertrain(
            battery=commanded.battery,
            driver=commanded.driver,
            channels=tuple(
                MotorChannel(motor=c.motor, inputs=c.inputs, output_speed_rad_s=speed)
                for c, speed in zip(channels, end.speed_rad_s)
            ),
        )
        result = ShaftStep(start=start, end=end, inputs=commands)
        self._system, self._state = updated, end.state
        self._time, self._angles = end_time, end.displacement_rad
        return result
