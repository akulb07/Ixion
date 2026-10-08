"""Static checks of declared electrical envelopes, not circuit simulation."""

from .project import RobotProject
from .validation import Diagnostic, pin_kinds


class PinAssignmentRule:
    """Check requested peripheral functions and reject impossible directions."""

    def evaluate(self, project: RobotProject) -> tuple[Diagnostic, ...]:
        pins = {(c.id, p.id): p for c in project.components for p in c.pins}
        connected = {(e.component, e.pin) for n in project.nets for e in n.endpoints}
        diagnostics = []
        for assignment in project.assignments:
            key = (assignment.endpoint.component, assignment.endpoint.pin)
            pin = pins[key]
            function = assignment.function
            allowed = {"bidirectional"}
            if function in {"DIGITAL_OUT", "PWM"}:
                allowed.add("output")
            elif function in {"DIGITAL_IN", "INTERRUPT"}:
                allowed.add("input")
            if function not in pin.capabilities or pin.kind not in allowed:
                diagnostics.append(
                    Diagnostic(
                        severity="ERROR",
                        code="pin_function_unsupported",
                        title="Unsupported pin function",
                        message=f"{key[0]}.{key[1]} cannot provide {function}.",
                        component=key[0],
                        suggestion="Select a pin with the required function and direction.",
                        context=(key[1], function),
                    )
                )
            if key not in connected:
                diagnostics.append(
                    Diagnostic(
                        severity="ERROR",
                        code="assigned_pin_unconnected",
                        title="Assigned pin is unconnected",
                        message=f"{key[0]}.{key[1]} is assigned but has no net.",
                        component=key[0],
                        suggestion="Connect this peripheral to the intended device.",
                        context=(key[1],),
                    )
                )
        return tuple(diagnostics)


class ElectricalRule:
    """Compare a single explicit source against every declared receiver on a net.

    Floating supplies fail. Multiple sources are left to WiringRule. Open-drain
    buses and unassigned GPIO directions remain unknown, rather than inferred.
    Rails describe output envelopes, not regulator enable or load behavior.
    """

    def evaluate(self, project: RobotProject) -> tuple[Diagnostic, ...]:
        pins = {(c.id, p.id): p for c in project.components for p in c.pins}
        kinds = pin_kinds(project)
        diagnostics = []
        for net in project.nets:
            keys = [(e.component, e.pin) for e in net.endpoints]
            sources = [key for key in keys if kinds[key] in {"power_out", "output"}]
            sinks = [key for key in keys if kinds[key] in {"power_in", "input"}]
            if not sinks:
                continue
            if len(sources) != 1:
                if not sources and any(kinds[key] == "input" for key in sinks):
                    diagnostics.append(self._diagnostic(net.id, "signal_drive_unresolved", "WARNING",
                        "No explicit signal driver is resolved; open-drain buses and unassigned GPIOs need separate analysis."))
                if not sources and any(kinds[key] == "power_in" for key in sinks):
                    diagnostics.append(
                        self._diagnostic(
                            net.id,
                            "power_source_missing",
                            "ERROR",
                            "A power input has no explicit source on its net.",
                        )
                    )
                continue
            source = pins[sources[0]]
            for key in sinks:
                sink = pins[key]
                supplied, accepted = source.driven_voltage, sink.accepted_voltage
                context = (net.id, f"{sources[0][0]}.{sources[0][1]}", f"{key[0]}.{key[1]}")
                if supplied is None or accepted is None:
                    diagnostics.append(
                        self._diagnostic(
                            net.id,
                            "voltage_limits_unknown",
                            "WARNING",
                            "Source or receiver voltage envelope is missing.",
                            context,
                        )
                    )
                elif (
                    supplied.minimum_v < accepted.minimum_v
                    or supplied.maximum_v > accepted.maximum_v
                ):
                    diagnostics.append(
                        self._diagnostic(
                            net.id,
                            "voltage_range_mismatch",
                            "ERROR",
                            f"Source range [{supplied.minimum_v}, {supplied.maximum_v}] V exceeds receiver range [{accepted.minimum_v}, {accepted.maximum_v}] V.",
                            context,
                        )
                    )
                if kinds[sources[0]] == "output" and kinds[key] == "input":
                    output, input_ = source.output_levels, sink.input_levels
                    if output is None or input_ is None:
                        diagnostics.append(
                            self._diagnostic(
                                net.id,
                                "logic_levels_unknown",
                                "WARNING",
                                "Logic thresholds or output guarantees are missing.",
                                context,
                            )
                        )
                    elif (
                        output.low_max_v > input_.low_max_v or output.high_min_v < input_.high_min_v
                    ):
                        diagnostics.append(
                            self._diagnostic(
                                net.id,
                                "logic_level_mismatch",
                                "ERROR",
                                "Output low/high guarantees do not meet receiver thresholds.",
                                context,
                            )
                        )
        return tuple(diagnostics)

    @staticmethod
    def _diagnostic(net: str, code: str, severity: str, message: str, context=()) -> Diagnostic:
        return Diagnostic(
            severity=severity,
            code=code,
            title="Electrical net check",
            message=f"{net}: {message}",
            suggestion="Check exact-part limits across supply, load and temperature; add or correct the declared envelopes.",
            context=context or (net,),
        )
