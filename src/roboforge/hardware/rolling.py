"""Symmetric straight-line rolling on a constant grade, with a traction check."""

import math

from pydantic import Field, model_serializer

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
    grade_rad: Real = Field(default=0, gt=-math.pi / 2, lt=math.pi / 2)

    @model_serializer(mode="wrap")
    def preserve_level_encoding(self, handler):
        data = handler(self)
        if self.grade_rad == 0:
            data.pop("grade_rad", None)
        return data


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


def rolling_inertia(body: RollingBody) -> float:
    # Per wheel: torque = J*alpha + force*r; 2*force = mass*r*alpha.
    return positive(
        body.rotating_inertia_per_wheel_kg_m2 + body.total_mass_kg * body.wheel_radius_m**2 / 2,
        "effective rolling inertia",
    )


def grade_force_n(body: RollingBody) -> float:
    """Gravity opposing positive travel; a positive grade rises along the lane."""
    return finite(body.total_mass_kg * body.gravity_m_s2 * math.sin(body.grade_rad), "grade force")


def rolling_load(body: RollingBody) -> ShaftLoad:
    return ShaftLoad(
        inertia_kg_m2=rolling_inertia(body),
        load_torque_nm=finite(grade_force_n(body) * body.wheel_radius_m / 2, "grade torque"),
    )


def rolling_sample(body: RollingBody, sample: ShaftSample) -> RollingSample:
    limit = finite(
        body.friction_coefficient
        * body.total_mass_kg
        * body.gravity_m_s2
        * math.cos(body.grade_rad)
        * body.driven_weight_fraction
        / 2,
        "traction limit",
    )
    acceleration = finite(body.wheel_radius_m * sample.acceleration_rad_s2[0], "body acceleration")
    force = finite(
        (body.total_mass_kg * acceleration + grade_force_n(body)) / 2, "wheel contact force"
    )
    if abs(force) > limit:
        raise ValueError(
            f"traction exceeded at {sample.time_s:g} s: requires {abs(force):g} N "
            f"per wheel, limit {limit:g} N; wheel slip is not modeled"
        )
    return RollingSample(
        shaft=sample,
        distance_m=finite(body.wheel_radius_m * sample.displacement_rad[0], "body distance"),
        speed_m_s=finite(body.wheel_radius_m * sample.speed_rad_s[0], "body speed"),
        acceleration_m_s2=acceleration,
        contact_force_per_wheel_n=force,
        traction_limit_per_wheel_n=limit,
    )


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
    drag, load transfer, turning and slip dynamics are not modeled. Constant
    grade contributes gravity load and reduces normal force. Distance is along
    the lane, not its horizontal projection.
    Distance starts at zero. Commands and initial speed come from the channels.
    """
    if len(system.channels) != 2 or system.channels[0] != system.channels[1]:
        raise ValueError("straight-line bench requires two identical motor channels and speeds")
    trace = run_shaft_bench(
        system,
        state,
        (rolling_load(body),) * 2,
        duration_s,
        step_s,
        vcc_v,
        auxiliary_current_a,
    )
    return RollingTrace(
        body=body,
        samples=tuple(rolling_sample(body, s) for s in trace.samples),
        duration_s=trace.duration_s,
        max_step_s=trace.max_step_s,
    )
