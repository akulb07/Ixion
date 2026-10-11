"""Symmetric straight-line rolling on level ground, with a traction validity check."""

from pydantic import Field

from roboforge.config import Nonnegative, Positive, Probability, Real, Schema
from roboforge.core import finite, positive

from .battery import BatteryState
from .powertrain import Powertrain
from .shaft_dynamics import ShaftLoad, ShaftSample, run_shaft_bench


class RollingBody(Schema):
    total_mass_kg: Positive
    wheel_radius_m: Positive
    rotating_inertia_per_wheel_kg_m2: Nonnegative
    friction_coefficient: Nonnegative
    driven_weight_fraction: Probability = Field(gt=0)
    gravity_m_s2: Positive = 9.80665


class RollingSample(Schema):
    shaft: ShaftSample
    distance_m: Real
    speed_m_s: Real
    acceleration_m_s2: Real
    contact_force_per_wheel_n: Real
    traction_limit_per_wheel_n: Nonnegative


class RollingTrace(Schema):
    body: RollingBody
    samples: tuple[RollingSample, ...]
    duration_s: Positive
    max_step_s: Positive


def run_rolling_bench(
    system: Powertrain,
    state: BatteryState,
    body: RollingBody,
    duration_s: float,
    step_s: float,
    vcc_v: float = 3.3,
    auxiliary_current_a: float = 0,
) -> RollingTrace:
    """Integrate two identical channels under a no-slip straight-line constraint.

    Weight on the driven wheels is supplied explicitly and shared equally. Caster
    drag, load transfer, incline, turning and slip dynamics are not modeled.
    Distance starts at zero. Commands and initial speed come from the channels.
    """
    if len(system.channels) != 2 or system.channels[0] != system.channels[1]:
        raise ValueError("straight-line bench requires two identical motor channels and speeds")
    radius = body.wheel_radius_m
    # Per wheel: torque = J*alpha + force*r; 2*force = mass*r*alpha.
    inertia = positive(
        body.rotating_inertia_per_wheel_kg_m2 + body.total_mass_kg * radius**2 / 2,
        "effective rolling inertia",
    )
    limit = finite(
        body.friction_coefficient
        * body.total_mass_kg
        * body.gravity_m_s2
        * body.driven_weight_fraction
        / 2,
        "traction limit",
    )
    trace = run_shaft_bench(
        system,
        state,
        (ShaftLoad(inertia_kg_m2=inertia),) * 2,
        duration_s,
        step_s,
        vcc_v,
        auxiliary_current_a,
    )
    samples = []
    for sample in trace.samples:
        acceleration = finite(radius * sample.acceleration_rad_s2[0], "body acceleration")
        force = finite(body.total_mass_kg * acceleration / 2, "wheel contact force")
        if abs(force) > limit:
            raise ValueError(
                f"traction exceeded at {sample.time_s:g} s: requires {abs(force):g} N "
                f"per wheel, limit {limit:g} N; wheel slip is not modeled"
            )
        samples.append(
            RollingSample(
                shaft=sample,
                distance_m=finite(radius * sample.displacement_rad[0], "body distance"),
                speed_m_s=finite(radius * sample.speed_rad_s[0], "body speed"),
                acceleration_m_s2=acceleration,
                contact_force_per_wheel_n=force,
                traction_limit_per_wheel_n=limit,
            )
        )
    return RollingTrace(
        body=body, samples=tuple(samples), duration_s=trace.duration_s, max_step_s=trace.max_step_s
    )
