"""Incremental symmetric rolling, with commands held during each interval."""

from roboforge.config import Schema
from roboforge.core import finite, nonnegative, positive

from .battery import BatteryState
from .driver import DriverInputs
from .powertrain import MotorChannel, Powertrain
from .rolling import RollingBody, RollingSample, rolling_load, rolling_sample, run_rolling_bench
from .shaft_dynamics import ShaftSample
from .voltage_limit import VoltageLimitEvent, locate_voltage_limit


class RollingStep(Schema):
    start: RollingSample
    end: RollingSample
    inputs: tuple[DriverInputs, ...]


class VoltageLimitedStep(Schema):
    step: RollingStep | None
    event: VoltageLimitEvent | None


class RollingRuntime:
    """Two equal wheels on a constant grade. A rejected step changes no state."""

    def __init__(
        self,
        system: Powertrain,
        state: BatteryState,
        body: RollingBody,
        *,
        vcc_v: float = 3.3,
        auxiliary_current_a: float = 0,
    ):
        if len(system.channels) != 2 or system.channels[0] != system.channels[1]:
            raise ValueError(
                "straight-line runtime requires two identical motor channels and speeds"
            )
        vcc = finite(vcc_v, "driver VCC")
        auxiliary = nonnegative(auxiliary_current_a, "auxiliary current")
        point = system.operating_point(state, vcc, auxiliary)
        load = rolling_load(body)
        initial = rolling_sample(
            body,
            ShaftSample(
                time_s=0,
                state=state,
                displacement_rad=(0, 0),
                speed_rad_s=tuple(c.output_speed_rad_s for c in system.channels),
                acceleration_rad_s2=tuple(
                    finite(
                        (c.motor.output_torque_nm - load.load_torque_nm) / load.inertia_kg_m2,
                        "shaft acceleration",
                    )
                    for c in point.channels
                ),
                point=point,
            ),
        )
        self._system = system
        self._body = body
        self._sample = initial
        self._vcc_v = vcc
        self._auxiliary_current_a = auxiliary

    @property
    def time_s(self) -> float:
        return self._sample.shaft.time_s

    @property
    def sample(self) -> RollingSample:
        return self._sample

    @property
    def system(self) -> Powertrain:
        return self._system

    def step(
        self, duration_s: float, inputs: tuple[DriverInputs, ...] | None = None
    ) -> RollingStep:
        duration = positive(duration_s, "rolling runtime step")
        end_time = finite(self.time_s + duration, "rolling runtime time")
        if end_time <= self.time_s:
            raise ValueError("rolling step is too small to advance simulation time")
        commands = tuple(c.inputs for c in self._system.channels) if inputs is None else inputs
        if len(commands) != 2:
            raise ValueError("provide exactly two rolling driver commands")
        commanded = Powertrain(
            battery=self._system.battery,
            driver=self._system.driver,
            channels=tuple(
                MotorChannel(motor=c.motor, inputs=command, output_speed_rad_s=c.output_speed_rad_s)
                for c, command in zip(self._system.channels, commands)
            ),
        )
        local = run_rolling_bench(
            commanded,
            self._sample.shaft.state,
            self._body,
            duration,
            duration,
            self._vcc_v,
            self._auxiliary_current_a,
        )

        def absolute(sample, time):
            angles = tuple(
                finite(a + b, "rolling displacement")
                for a, b in zip(self._sample.shaft.displacement_rad, sample.shaft.displacement_rad)
            )
            return RollingSample(
                shaft=sample.shaft.model_copy(update={"time_s": time, "displacement_rad": angles}),
                distance_m=finite(self._sample.distance_m + sample.distance_m, "rolling distance"),
                speed_m_s=sample.speed_m_s,
                acceleration_m_s2=sample.acceleration_m_s2,
                contact_force_per_wheel_n=sample.contact_force_per_wheel_n,
                traction_limit_per_wheel_n=sample.traction_limit_per_wheel_n,
            )

        start = absolute(local.samples[0], self.time_s)
        end = absolute(local.samples[-1], end_time)
        updated = Powertrain(
            battery=commanded.battery,
            driver=commanded.driver,
            channels=tuple(
                MotorChannel(motor=c.motor, inputs=c.inputs, output_speed_rad_s=speed)
                for c, speed in zip(commanded.channels, end.shaft.speed_rad_s)
            ),
        )
        result = RollingStep(start=start, end=end, inputs=commands)
        self._system, self._sample = updated, end
        return result

    def step_until_voltage_limit(
        self,
        duration_s: float,
        threshold_v: float,
        inputs: tuple[DriverInputs, ...] | None = None,
    ) -> VoltageLimitedStep:
        """Advance at most one interval, ending at a declared supply boundary."""
        duration = positive(duration_s, "voltage-limited rolling step")
        if finite(self.time_s + duration, "voltage-limited end time") <= self.time_s:
            raise ValueError("rolling step is too small to advance simulation time")
        commands = tuple(c.inputs for c in self._system.channels) if inputs is None else inputs
        if len(commands) != 2:
            raise ValueError("provide exactly two rolling driver commands")
        commanded = self._system.model_copy(
            update={
                "channels": tuple(
                    c.model_copy(update={"inputs": command})
                    for c, command in zip(self._system.channels, commands)
                )
            }
        )
        if commanded.channels[0] != commanded.channels[1]:
            raise ValueError(
                "straight-line runtime requires two identical motor channels and speeds"
            )
        event = locate_voltage_limit(
            commanded,
            self._sample.shaft.state,
            (rolling_load(self._body),) * 2,
            duration,
            threshold_v,
            time_s=self.time_s,
            vcc_v=self._vcc_v,
            auxiliary_current_a=self._auxiliary_current_a,
        )
        elapsed = duration if event is None else event.time_s - self.time_s
        if event is not None and event.phase == "physics_step" and elapsed <= 0:
            raise ValueError(
                "voltage crossing is below simulation time resolution; reduce the step"
            )
        step = None if elapsed <= 0 else self.step(elapsed, commands)
        return VoltageLimitedStep(step=step, event=event)
