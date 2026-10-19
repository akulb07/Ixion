"""Resolve a two-terminal motor's driver channel and electrical polarity."""

from typing import Literal

from roboforge.config import Schema

from .driver import DriverInputs
from .project import Identifier, RobotProject
from .validation import check_project


class MotorConnection(Schema):
    motor: Identifier
    driver: Identifier
    channel: Literal["A", "B"]
    polarity: Literal[-1, 1]

    def motor_inputs(self, driver_inputs: DriverInputs) -> DriverInputs:
        """Transform commands into the motor P-to-N voltage convention.

        Swapping direction inputs preserves brake/coast/standby and reverses
        driven voltage. Speeds and torque remain in the motor's positive frame.
        """
        if self.polarity == 1:
            return driver_inputs
        return DriverInputs(
            standby=driver_inputs.standby,
            in1=driver_inputs.in2,
            in2=driver_inputs.in1,
            pwm_duty=driver_inputs.pwm_duty,
        )


def resolve_rover_wheels(project: RobotProject, motors: tuple[str, str]) -> list[str]:
    """Require two distinct wheel shafts, each driven by exactly one motor."""
    if motors[0] == motors[1]:
        raise ValueError("select two distinct rover motors")
    parts = {c.id: c for c in project.components}
    wheels = []
    for motor in motors:
        links = [p for p in project.assembly.links if p.source == motor and p.kind == "shaft"]
        if len(links) != 1 or parts[links[0].target].category != "wheel":
            raise ValueError("each rover motor requires exactly one shaft link to a wheel")
        wheel = links[0].target
        if sum(p.target == wheel and p.kind == "shaft" for p in project.assembly.links) != 1:
            raise ValueError("each rover wheel must have exactly one driving shaft")
        wheels.append(wheel)
    if len(set(wheels)) != 2:
        raise ValueError("rover motors must drive distinct wheels")
    return wheels


def resolve_motor_connection(project: RobotProject, motor: str) -> MotorConnection:
    components = {c.id: c for c in project.components}
    component = components.get(motor)
    if component is None or component.category != "motor":
        raise ValueError("motor connection requires an existing motor component")
    errors = [d.code for d in check_project(project) if d.severity == "ERROR"]
    if errors:
        raise ValueError("project wiring has errors: " + ", ".join(sorted(set(errors))))
    pins = {p.id: p for p in component.pins}
    nets = {(e.component, e.pin): n for n in project.nets for e in n.endpoints}
    terminals = []
    for terminal in ("P", "N"):
        pin = pins.get(terminal)
        if pin is None or pin.kind != "passive" or "MOTOR_OUTPUT" not in pin.capabilities:
            raise ValueError("motor must have passive P/N terminals with MOTOR_OUTPUT capability")
        net = nets.get((motor, terminal))
        if net is None or len(net.endpoints) != 2:
            raise ValueError("motor terminals require direct two-endpoint driver nets")
        other = next((e for e in net.endpoints if e.component != motor), None)
        if other is None:
            raise ValueError("motor terminals cannot be shorted together")
        driver = components[other.component]
        driver_pin = next(p for p in driver.pins if p.id == other.pin)
        if (
            driver.category != "motor_driver"
            or driver.type != "tb6612fng"
            or driver_pin.kind != "output"
            or "MOTOR_OUTPUT" not in driver_pin.capabilities
        ):
            raise ValueError("motor terminal must connect to a TB6612 motor output")
        terminals.append(other)
    positive, negative = terminals
    if positive.component != negative.component:
        raise ValueError("motor terminals must use the same driver")
    pairs = {
        ("AO1", "AO2"): ("A", 1),
        ("AO2", "AO1"): ("A", -1),
        ("BO1", "BO2"): ("B", 1),
        ("BO2", "BO1"): ("B", -1),
    }
    pair = pairs.get((positive.pin, negative.pin))
    if pair is None:
        raise ValueError("motor terminals must use one complete driver channel")
    return MotorConnection(
        motor=motor, driver=positive.component, channel=pair[0], polarity=pair[1]
    )
