"""Small composable graph rules; not an electrical safety certification."""

from typing import Literal, Protocol

from roboforge.config import Schema

from .project import RobotProject


class Diagnostic(Schema):
    severity: Literal["INFO", "PASS", "WARNING", "ERROR"]
    code: str
    title: str
    message: str
    component: str | None = None
    suggestion: str
    context: tuple[str, ...] = ()


class ValidationRule(Protocol):
    """A rule consumes a resolved graph without modifying or simulating it."""

    def evaluate(self, project: RobotProject) -> tuple[Diagnostic, ...]: ...


def pin_kinds(project: RobotProject) -> dict[tuple[str, str], str]:
    """Resolve valid digital assignments without treating I2C as push-pull."""
    pins = {(c.id, p.id): p for c in project.components for p in c.pins}
    kinds = {key: pin.kind for key, pin in pins.items()}
    for assignment in project.assignments:
        key = (assignment.endpoint.component, assignment.endpoint.pin)
        if assignment.function not in pins[key].capabilities:
            continue
        if pins[key].kind == "bidirectional":
            if assignment.function in {"DIGITAL_OUT", "PWM"}:
                kinds[key] = "output"
            elif assignment.function in {"DIGITAL_IN", "INTERRUPT"}:
                kinds[key] = "input"
    return kinds


class WiringRule:
    """Detect absent required pins and explicit source conflicts."""

    def evaluate(self, project: RobotProject) -> tuple[Diagnostic, ...]:
        pins = {(c.id, p.id): p for c in project.components for p in c.pins}
        connected = {(e.component, e.pin) for n in project.nets for e in n.endpoints}
        diagnostics = []
        for key, pin in pins.items():
            if pin.required and key not in connected:
                diagnostics.append(
                    Diagnostic(
                        severity="ERROR",
                        code="required_pin_unconnected",
                        title="Required pin is unconnected",
                        message=f"{key[0]}.{key[1]} needs a connection.",
                        component=key[0],
                        suggestion="Connect the pin to its intended signal or supply net.",
                        context=(key[1],),
                    )
                )
        resolved = pin_kinds(project)
        for net in project.nets:
            kinds = [resolved[(e.component, e.pin)] for e in net.endpoints]
            sources = sum(kind in {"output", "power_out"} for kind in kinds)
            if sources > 1 or (sources and "ground" in kinds):
                diagnostics.append(
                    Diagnostic(
                        severity="ERROR",
                        code="net_source_conflict",
                        title="Conflicting net sources",
                        message=f"{net.id} joins explicit outputs or an output to ground.",
                        suggestion="Separate push-pull sources; model open-drain pins as bidirectional.",
                        context=(net.id,),
                    )
                )
        return tuple(diagnostics)


class I2CRule:
    """Check addresses, declared memberships and line capabilities."""

    def evaluate(self, project: RobotProject) -> tuple[Diagnostic, ...]:
        nets = {n.id: n for n in project.nets}
        pins = {(c.id, p.id): p for c in project.components for p in c.pins}
        diagnostics = []
        for bus in project.i2c_buses:
            addresses = [d.address for d in bus.devices]
            if len(set(addresses)) != len(addresses):
                diagnostics.append(
                    Diagnostic(
                        severity="ERROR",
                        code="i2c_duplicate_address",
                        title="I2C address collision",
                        message=f"{bus.id} has multiple devices at the same address.",
                        suggestion="Change an address or use a separate bus/multiplexer.",
                        context=(bus.id,),
                    )
                )
            members = {bus.controller, *(d.component for d in bus.devices)}
            for net_id, capability in ((bus.sda_net, "I2C_SDA"), (bus.scl_net, "I2C_SCL")):
                endpoints = nets[net_id].endpoints
                capable = {
                    e.component
                    for e in endpoints
                    if capability in pins[(e.component, e.pin)].capabilities
                }
                invalid = [
                    e
                    for e in endpoints
                    if not {capability, "POWER"} & set(pins[(e.component, e.pin)].capabilities)
                ]
                if not members <= capable or invalid:
                    diagnostics.append(
                        Diagnostic(
                            severity="ERROR",
                            code="i2c_line_membership",
                            title="I2C line mismatch",
                            message=f"{net_id} does not connect all members through {capability} pins or has incompatible pins.",
                            suggestion="Check controller/device pin assignments on both bus lines.",
                            context=(bus.id, net_id),
                        )
                    )
        return tuple(diagnostics)


def check_project(
    project: RobotProject, rules: tuple[ValidationRule, ...] | None = None
) -> tuple[Diagnostic, ...]:
    """Run graph rules and explicitly report unassessed engineering domains."""
    from .electrical import ElectricalRule, GroundReferenceRule, PinAssignmentRule

    active = (
        (WiringRule(), I2CRule(), PinAssignmentRule(), GroundReferenceRule(), ElectricalRule())
        if rules is None
        else rules
    )
    return tuple(d for rule in active for d in rule.evaluate(project)) + (
        Diagnostic(
            severity="WARNING",
            code="engineering_checks_pending",
            title="Engineering checks not implemented",
            message="Declared electrical limits are checked where provided; supply activation, current, thermal, bus timing, firmware and mechanical readiness remain unassessed.",
            suggestion="Verify part variants and electrical ratings; hardware simulation is not available yet.",
        ),
    )
