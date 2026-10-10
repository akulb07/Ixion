"""Discharge-only pack model: tabulated OCV, ohmic sag and coulomb counting."""

from typing import Literal

from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Positive, Probability, Schema
from roboforge.core import finite, nonnegative

from .project import Component


class OCVPoint(Schema):
    soc: Probability
    voltage_v: Positive


class BatteryState(Schema):
    soc: Probability = 1
    peak_current_a: Nonnegative = 0


class BatteryPoint(Schema):
    soc: Probability
    open_circuit_voltage_v: Positive
    terminal_voltage_v: Nonnegative
    current_a: Nonnegative
    power_w: Nonnegative
    internal_loss_w: Nonnegative
    charge_limited_runtime_s: Nonnegative | None
    diagnostics: tuple[str, ...] = ()


class BatteryStep(Schema):
    duration_s: Nonnegative
    discharged_ah: Nonnegative
    state: BatteryState
    start: BatteryPoint
    end: BatteryPoint


class BatteryModel(Schema):
    """All ratings are for the whole pack, never multiplied by cell counts.

    Chemistry is descriptive; the supplied OCV table controls voltage. No
    chemistry-specific default discharge curve or charging behavior is implied.
    """

    chemistry: Literal["LiPo", "Li-ion"]
    capacity_ah: Positive
    internal_resistance_ohm: Nonnegative
    ocv_curve: tuple[OCVPoint, ...] = Field(min_length=2, max_length=128)
    max_continuous_current_a: Positive | None = None
    max_burst_current_a: Positive | None = None

    @model_validator(mode="after")
    def consistent_curve(self):
        if self.ocv_curve[0].soc != 0 or self.ocv_curve[-1].soc != 1:
            raise ValueError("OCV curve must cover SOC zero through one")
        if any(
            a.soc >= b.soc or a.voltage_v > b.voltage_v
            for a, b in zip(self.ocv_curve, self.ocv_curve[1:])
        ):
            raise ValueError("OCV curve requires increasing SOC and nondecreasing voltage")
        if (
            self.max_continuous_current_a is not None
            and self.max_burst_current_a is not None
            and self.max_burst_current_a < self.max_continuous_current_a
        ):
            raise ValueError("burst current rating cannot be below continuous rating")
        return self

    def open_circuit_voltage(self, soc: float) -> float:
        """Piecewise-linear interpolation of the user-supplied pack OCV table."""
        charge = BatteryState(soc=soc).soc
        for a, b in zip(self.ocv_curve, self.ocv_curve[1:]):
            if charge <= b.soc:
                return finite(
                    a.voltage_v + (b.voltage_v - a.voltage_v) * (charge - a.soc) / (b.soc - a.soc),
                    "OCV",
                )
        return self.ocv_curve[-1].voltage_v

    def operating_point(self, state: BatteryState, current_a: float) -> BatteryPoint:
        """Estimate a prescribed discharge current; warnings do not clamp it."""
        current = nonnegative(current_a, "discharge current (charging is unsupported)")
        ocv = self.open_circuit_voltage(state.soc)
        voltage = finite(ocv - current * self.internal_resistance_ohm, "terminal voltage")
        if voltage < 0:
            raise ValueError("requested current collapses terminal voltage below zero")
        diagnostics = []
        if state.soc == 0 and current > 0:
            raise ValueError("battery is depleted; positive discharge current is unavailable")
        for label, limit in (
            ("continuous", self.max_continuous_current_a),
            ("burst", self.max_burst_current_a),
        ):
            if limit is None:
                diagnostics.append(f"{label}_current_rating_unknown")
            elif current > limit:
                diagnostics.append(f"{label}_current_exceeded")
        runtime = (
            None
            if current == 0
            else finite(state.soc * self.capacity_ah / current * 3600, "charge-limited runtime")
        )
        return BatteryPoint(
            soc=state.soc,
            open_circuit_voltage_v=ocv,
            terminal_voltage_v=voltage,
            current_a=current,
            power_w=voltage * current,
            internal_loss_w=current * current * self.internal_resistance_ohm,
            charge_limited_runtime_s=runtime,
            diagnostics=tuple(diagnostics),
        )

    def discharge(self, state: BatteryState, current_a: float, duration_s: float) -> BatteryStep:
        """Return a new state for constant current; reject steps past depletion.

        At SOC zero the end point has zero current (ideal charge exhaustion).
        Failed steps leave the immutable original state intact.
        """
        current = nonnegative(current_a, "discharge current")
        duration = nonnegative(duration_s, "discharge duration")
        used = finite(current * duration / 3600, "discharged charge")
        available = finite(state.soc * self.capacity_ah, "remaining charge")
        if used > available:
            raise ValueError(
                "step exceeds remaining charge; reduce duration to the depletion boundary"
            )
        next_state = BatteryState(
            soc=max(0.0, state.soc - used / self.capacity_ah),
            peak_current_a=max(state.peak_current_a, current) if duration else state.peak_current_a,
        )
        if self.open_circuit_voltage(next_state.soc) - current * self.internal_resistance_ohm < 0:
            raise ValueError("requested current collapses voltage during the step")
        return BatteryStep(
            duration_s=duration,
            discharged_ah=used,
            state=next_state,
            start=self.operating_point(state, current),
            end=self.operating_point(next_state, 0 if next_state.soc == 0 else current),
        )

    @classmethod
    def from_component(cls, component: Component) -> "BatteryModel":
        """Use explicit empty/full pack OCV inputs for a two-point approximation."""
        chemistries = {"lipo": "LiPo", "lipo-2s": "LiPo", "li-ion": "Li-ion"}
        if component.category != "battery" or component.type not in chemistries:
            raise ValueError("select a lipo, lipo-2s or li-ion battery component")
        parameters = {p.name: p for p in component.parameters}

        def read(name, unit, optional=False):
            parameter = parameters.get(name)
            if parameter is None and optional:
                return None
            if parameter is None or parameter.unit != unit:
                raise ValueError(f"battery requires {name} with unit {unit}")
            return parameter.value

        return cls(
            chemistry=chemistries[component.type],
            capacity_ah=read("capacity", "Ah"),
            internal_resistance_ohm=read("internal_resistance", "ohm"),
            ocv_curve=(
                OCVPoint(soc=0, voltage_v=read("empty_ocv", "V")),
                OCVPoint(soc=1, voltage_v=read("full_ocv", "V")),
            ),
            max_continuous_current_a=read("max_continuous_current", "A", True),
            max_burst_current_a=read("max_burst_current", "A", True),
        )
