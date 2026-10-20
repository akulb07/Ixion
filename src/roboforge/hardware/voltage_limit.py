"""Locate a declared voltage boundary within one explicit-Euler interval."""

import math
from typing import Literal

from pydantic import Field

from roboforge.config import Nonnegative, Real, Schema
from roboforge.core import finite, nonnegative, positive

from .battery import BatteryState
from .driver import DriverInputs
from .powertrain import Powertrain
from .shaft_dynamics import ShaftLoad


class VoltageLimitEvent(Schema):
    phase: Literal["command_boundary", "physics_step"]
    time_s: Nonnegative
    threshold_v: Real = Field(gt=2.5, le=13.5)
    voltage_v: Real
    bracket_end_time_s: Nonnegative
    bracket_end_voltage_v: Real
    time_tolerance_s: Nonnegative
    motor_inputs: tuple[DriverInputs, ...] = Field(min_length=1, max_length=2)


def locate_voltage_limit(
    system: Powertrain,
    state: BatteryState,
    loads: tuple[ShaftLoad, ...],
    duration_s: float,
    threshold_v: float,
    *,
    time_s: float = 0,
    vcc_v: float = 3.3,
    auxiliary_current_a: float = 0,
) -> VoltageLimitEvent | None:
    """Check command application, then OCV knots, then bracket the first crossing.

    Speeds and charge follow the same frozen initial derivatives as one Euler
    step. No driver is evaluated below its supported supply range. The returned
    interior time is the last above-threshold side of a bounded bracket, not an
    exact continuous-time crossing or a simulated electrical disconnection.
    """
    duration = positive(duration_s, "voltage-limit interval")
    threshold = finite(threshold_v, "minimum pack voltage")
    time_s = nonnegative(time_s, "voltage-limit start time")
    vcc_v = finite(vcc_v, "driver VCC")
    if not 2.7 <= vcc_v <= 5.5:
        raise ValueError("voltage-limit estimate requires driver VCC 2.7–5.5 V")
    if not 2.5 < threshold <= 13.5:
        raise ValueError("minimum pack voltage must be above 2.5 V and at most 13.5 V")
    if len(loads) != len(system.channels):
        raise ValueError("provide one load per voltage-limit channel")
    initial_voltage = system.algebraic_supply_voltage(state.soc, auxiliary_current_a)
    if initial_voltage <= threshold:
        return VoltageLimitEvent(
            phase="command_boundary",
            time_s=time_s,
            threshold_v=threshold,
            voltage_v=initial_voltage,
            bracket_end_time_s=time_s,
            bracket_end_voltage_v=initial_voltage,
            time_tolerance_s=0,
            motor_inputs=tuple(c.inputs for c in system.channels),
        )
    point = system.operating_point(state, vcc_v, auxiliary_current_a)
    acceleration = tuple(
        finite(
            (c.motor.output_torque_nm - load.load_torque_nm) / load.inertia_kg_m2,
            "voltage-limit shaft acceleration",
        )
        for c, load in zip(point.channels, loads)
    )
    current = point.battery.current_a
    capacity = system.battery.capacity_ah
    end = duration
    if current > 0:
        depletion = finite(state.soc * capacity * 3600 / current, "depletion time")
        end = min(end, depletion)
    if end <= 0:
        return None  # The ordinary runtime will report unsupported depletion.

    def voltage(elapsed):
        used = finite(current * elapsed / 3600, "preview discharged charge")
        # Probe no later than charge exhaustion. max only handles roundoff at
        # that boundary, matching BatteryModel.discharge's SOC calculation.
        soc = max(0.0, finite(state.soc - used / capacity, "preview SOC"))
        channels = tuple(
            c.model_copy(
                update={
                    "output_speed_rad_s": finite(
                        c.output_speed_rad_s + a * elapsed, "preview shaft speed"
                    )
                }
            )
            for c, a in zip(system.channels, acceleration)
        )
        candidate = system.model_copy(update={"channels": channels})
        return candidate.algebraic_supply_voltage(soc, auxiliary_current_a)

    probes = {end}
    if current > 0:
        for knot in system.battery.ocv_curve:
            elapsed = finite((state.soc - knot.soc) * capacity * 3600 / current, "OCV knot time")
            if 0 < elapsed < end:
                probes.add(elapsed)
    lower, lower_voltage = 0.0, initial_voltage
    for upper in sorted(probes):
        upper_voltage = voltage(upper)
        if upper_voltage <= threshold:
            tolerance = max(1e-12, duration * 1e-9, math.ulp(time_s + duration) * 4)
            for _ in range(80):
                if upper - lower <= tolerance and lower > 0:
                    break
                middle = (lower + upper) / 2
                value = voltage(middle)
                if value <= threshold:
                    upper, upper_voltage = middle, value
                else:
                    lower, lower_voltage = middle, value
            if upper - lower > tolerance:
                raise ValueError(
                    "voltage crossing could not be bracketed within the iteration budget"
                )
            absolute_lower = finite(time_s + lower, "limit time")
            if absolute_lower <= time_s:
                raise ValueError(
                    "voltage crossing is below simulation time resolution; reduce the step"
                )
            return VoltageLimitEvent(
                phase="physics_step",
                time_s=absolute_lower,
                threshold_v=threshold,
                voltage_v=lower_voltage,
                bracket_end_time_s=finite(time_s + upper, "limit bracket end"),
                bracket_end_voltage_v=upper_voltage,
                time_tolerance_s=tolerance,
                motor_inputs=tuple(c.inputs for c in system.channels),
            )
        lower, lower_voltage = upper, upper_voltage
    return None
