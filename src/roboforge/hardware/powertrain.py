"""Shared-battery algebraic solve for one or two averaged driver channels."""

import math

from pydantic import Field

from roboforge.config import Nonnegative, Real, Schema
from roboforge.core import finite, nonnegative, positive

from .battery import BatteryModel, BatteryPoint, BatteryState
from .driver import DriverInputs, DriverPoint, TB6612Driver
from .motor import DCMotor


class MotorChannel(Schema):
    motor: DCMotor
    inputs: DriverInputs
    output_speed_rad_s: Real


class PowertrainPoint(Schema):
    battery: BatteryPoint
    channels: tuple[DriverPoint, ...]
    auxiliary_current_a: Nonnegative
    auxiliary_power_w: Nonnegative


class PowertrainSample(Schema):
    time_s: Nonnegative
    state: BatteryState
    point: PowertrainPoint


class DischargeTrace(Schema):
    """Fixed-speed explicit-Euler discharge; not a mechanical trajectory."""

    samples: tuple[PowertrainSample, ...]
    duration_s: Nonnegative
    max_step_s: Nonnegative


class Powertrain(Schema):
    """An estimate at fixed shaft speeds, not a dynamic robot simulation.

    Auxiliary current is pack-side current. No regulator conversion is implied.
    The externally supplied VCC value is not automatically an auxiliary load.
    """

    battery: BatteryModel
    driver: TB6612Driver = TB6612Driver()
    channels: tuple[MotorChannel, ...] = Field(min_length=1, max_length=2)

    def discharge_trace(
        self,
        state: BatteryState,
        duration_s: float,
        step_s: float,
        vcc_v: float = 3.3,
        auxiliary_current_a: float = 0,
    ) -> DischargeTrace:
        """Integrate charge with current held over each step and re-solved afterward.

        Returns only a complete trace. Depletion, undervoltage or regeneration
        aborts with ValueError rather than fabricating the rest of a mission.
        Uses simulation time only, at most 10,000 steps, and a shortened last step.
        """
        duration = positive(duration_s, "trace duration")
        step = positive(step_s, "trace step")
        ratio = finite(duration / step, "trace step count")
        count = max(1, math.ceil(ratio))
        if count > 10_000:
            raise ValueError("discharge trace exceeds 10,000-step budget")
        samples = []
        point = self.operating_point(state, vcc_v, auxiliary_current_a)
        samples.append(PowertrainSample(time_s=0, state=state, point=point))
        for index in range(count):
            end = min(duration, (index + 1) * step)
            elapsed = end - samples[-1].time_s
            if elapsed <= 0:
                break  # Do not duplicate a rounded end sample.
            updated = self.battery.discharge(state, point.battery.current_a, elapsed)
            state = updated.state
            point = self.operating_point(state, vcc_v, auxiliary_current_a)
            samples.append(PowertrainSample(time_s=end, state=state, point=point))
        return DischargeTrace(samples=tuple(samples), duration_s=duration, max_step_s=step)

    def operating_point(
        self, state: BatteryState, vcc_v: float = 3.3, auxiliary_current_a: float = 0
    ) -> PowertrainPoint:
        """Solve pack voltage and signed channel supply currents simultaneously.

        For each driven channel, I_supply = a*(a*V - E)/(Rm+Rd),
        where a is signed duty. With I_total = A*V+B and V=OCV-Rb*I_total,
        V=(OCV-Rb*B)/(1+Rb*A). Brake/coast channels draw no ideal supply
        current. Regenerative channels are rejected until sink paths are modeled.
        """
        auxiliary = nonnegative(auxiliary_current_a, "auxiliary current")
        conductance, offset = 0.0, auxiliary
        for channel in self.channels:
            _, fraction = channel.inputs.resolve()
            if fraction is None or fraction == 0:
                continue
            motor = channel.motor
            rotor_speed = finite(channel.output_speed_rad_s * motor.gear_ratio, "rotor speed")
            emf = finite(motor.motor_constant_si * rotor_speed, "back EMF")
            resistance = finite(
                motor.winding_resistance_ohm + self.driver.bridge_resistance_ohm,
                "channel resistance",
            )
            conductance = finite(conductance + fraction * fraction / resistance, "conductance")
            offset = finite(offset - fraction * emf / resistance, "current offset")
        resistance = self.battery.internal_resistance_ohm
        ocv = self.battery.open_circuit_voltage(state.soc)
        voltage = finite(
            (ocv - resistance * offset) / (1 + resistance * conductance), "coupled voltage"
        )
        channels = tuple(
            self.driver.evaluate(c.motor, c.inputs, voltage, vcc_v, c.output_speed_rad_s)
            for c in self.channels
        )
        if any(c.supply_current_a < 0 for c in channels):
            raise ValueError("regenerative channel requires an unimplemented supply sink model")
        current = finite(auxiliary + sum(c.supply_current_a for c in channels), "pack current")
        battery = self.battery.operating_point(state, current)
        return PowertrainPoint(
            battery=battery,
            channels=channels,
            auxiliary_current_a=auxiliary,
            auxiliary_power_w=voltage * auxiliary,
        )
