"""Incremental symmetric rolling, with commands held during each interval."""

from roboforge.config import Schema
from roboforge.core import finite, positive

from .battery import BatteryState
from .driver import DriverInputs
from .powertrain import MotorChannel, Powertrain
from .rolling import RollingBody, RollingSample, rolling_load, rolling_sample, run_rolling_bench
from .shaft_dynamics import ShaftSample


class RollingStep(Schema):
    start: RollingSample
    end: RollingSample
    inputs: tuple[DriverInputs, ...]


class RollingRuntime:
    """Two equal wheels on a constant grade. A rejected step changes no state."""

    def __init__(self, system: Powertrain, state: BatteryState, body: RollingBody):
        if len(system.channels) != 2 or system.channels[0] != system.channels[1]:
            raise ValueError(
                "straight-line runtime requires two identical motor channels and speeds"
            )
        point = system.operating_point(state)
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
            commanded, self._sample.shaft.state, self._body, duration, duration
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
