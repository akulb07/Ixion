"""Independent output-shaft inertia bench, with shared electrical supply."""

import math

from roboforge.config import Nonnegative, Positive, Real, Schema
from roboforge.core import finite, positive

from .battery import BatteryState
from .powertrain import MotorChannel, Powertrain, PowertrainPoint


class ShaftLoad(Schema):
    """Effective output-axis inertia and signed resisting torque.

    Torque is subtracted from motor torque, even at zero or negative speed.
    This is a constant applied load, not a dry-friction or wheel-contact model.
    Inertia includes all rotating parts referred to the output axis.
    """

    inertia_kg_m2: Positive
    load_torque_nm: Real = 0


class ShaftSample(Schema):
    time_s: Nonnegative
    state: BatteryState
    displacement_rad: tuple[Real, ...]
    speed_rad_s: tuple[Real, ...]
    acceleration_rad_s2: tuple[Real, ...]
    point: PowertrainPoint


class ShaftTrace(Schema):
    samples: tuple[ShaftSample, ...]
    loads: tuple[ShaftLoad, ...]
    duration_s: Positive
    max_step_s: Positive


def run_shaft_bench(
    system: Powertrain,
    state: BatteryState,
    loads: tuple[ShaftLoad, ...],
    duration_s: float,
    step_s: float,
    vcc_v: float = 3.3,
    auxiliary_current_a: float = 0,
) -> ShaftTrace:
    """Explicit Euler for speed and charge; commands remain fixed.

    Invalid electrical states abort the whole run. No protection, regeneration,
    chassis dynamics or timestep stability is inferred. Reduce the step and check
    convergence for each setup; only simulation time is used.
    """
    duration = positive(duration_s, "bench duration")
    step = positive(step_s, "bench step")
    count = max(1, math.ceil(finite(duration / step, "bench step count")))
    if count > 10_000:
        raise ValueError("shaft bench exceeds 10,000-step budget")
    if len(loads) != len(system.channels):
        raise ValueError("provide exactly one shaft load per motor channel")
    samples = []
    current = system
    time = 0.0
    displacement = (0.0,) * len(system.channels)
    for index in range(count + 1):
        point = current.operating_point(state, vcc_v, auxiliary_current_a)
        acceleration = tuple(
            finite(
                (channel.motor.output_torque_nm - load.load_torque_nm) / load.inertia_kg_m2,
                "shaft acceleration",
            )
            for channel, load in zip(point.channels, loads)
        )
        speeds = tuple(c.output_speed_rad_s for c in current.channels)
        samples.append(
            ShaftSample(
                time_s=time,
                state=state,
                displacement_rad=displacement,
                speed_rad_s=speeds,
                acceleration_rad_s2=acceleration,
                point=point,
            )
        )
        if index == count:
            break
        end = min(duration, (index + 1) * step)
        elapsed = end - time
        if elapsed <= 0:
            break
        channels = tuple(
            MotorChannel(
                motor=c.motor,
                inputs=c.inputs,
                output_speed_rad_s=finite(speed + alpha * elapsed, "shaft speed"),
            )
            for c, speed, alpha in zip(current.channels, speeds, acceleration)
        )
        displacement = tuple(
            finite(angle + speed * elapsed, "shaft displacement")
            for angle, speed in zip(displacement, speeds)
        )
        state = system.battery.discharge(state, point.battery.current_a, elapsed).state
        current = Powertrain(battery=system.battery, driver=system.driver, channels=channels)
        time = end
    return ShaftTrace(samples=tuple(samples), loads=loads, duration_s=duration, max_step_s=step)
