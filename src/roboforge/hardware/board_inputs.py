"""Resolve ideal encoder inputs through project nets to one MCU."""

from collections.abc import Callable, Mapping

from roboforge.config import Nonnegative, Schema

from .gpio_replay import EncoderGPIOReplay
from .project import Identifier, RobotProject
from .quadrature import EncoderTrace
from .validation import check_project


class EncoderBinding(Schema):
    component: Identifier
    a_pin: Identifier = "A"
    b_pin: Identifier = "B"


class BoardInputChange(Schema):
    time_s: Nonnegative
    mcu: str
    pin: str
    level: bool


class BoardInputReplay:
    """Only directly wired encoder inputs; no voltage or CPU simulation."""

    def __init__(
        self,
        project: RobotProject,
        mcu: str,
        traces: Mapping[str, EncoderTrace],
        bindings: Mapping[str, EncoderBinding],
    ):
        if set(traces) != set(bindings):
            raise ValueError("each encoder trace needs exactly one binding")
        components = {c.id: c for c in project.components}
        if mcu not in components or components[mcu].category != "mcu":
            raise ValueError("input replay requires an existing MCU component")
        self.diagnostics = check_project(project)
        errors = [d.code for d in self.diagnostics if d.severity == "ERROR"]
        if errors:
            raise ValueError("project wiring has errors: " + ", ".join(sorted(set(errors))))
        nets = {(e.component, e.pin): net for net in project.nets for e in net.endpoints}
        assignments = {
            (a.endpoint.component, a.endpoint.pin): a.function for a in project.assignments
        }
        self._pins = {}
        self._functions = {}
        for channel, binding in bindings.items():
            component = components.get(binding.component)
            if component is None or component.category != "encoder":
                raise ValueError("binding must name an encoder component")
            for signal, source_pin in (("a", binding.a_pin), ("b", binding.b_pin)):
                pin = next((p for p in component.pins if p.id == source_pin), None)
                if pin is None or pin.kind != "output" or "DIGITAL_OUT" not in pin.capabilities:
                    raise ValueError("encoder binding requires digital output pins")
                net = nets.get((component.id, source_pin))
                if net is None or len(net.endpoints) != 2:
                    raise ValueError("encoder replay requires a direct two-endpoint net")
                targets = [e for e in net.endpoints if e.component == mcu]
                if len(targets) != 1:
                    raise ValueError("encoder signal must connect directly to the selected MCU")
                target = targets[0]
                function = assignments.get((mcu, target.pin))
                if function not in {"DIGITAL_IN", "INTERRUPT"}:
                    raise ValueError("encoder destination must be assigned DIGITAL_IN or INTERRUPT")
                if target.pin in self._pins:
                    raise ValueError("encoder bindings reuse an MCU pin")
                self._pins[target.pin] = (channel, signal)
                self._functions[target.pin] = function
        self._mcu = mcu
        self._replay = EncoderGPIOReplay(traces)

    @property
    def time_s(self) -> float:
        return self._replay.time_s

    @property
    def pin_map(self) -> dict[str, tuple[str, str]]:
        return dict(self._pins)

    def digital_read(self, pin: str) -> bool:
        channel, signal = self._pins[pin]
        return self._replay.read(channel, signal)

    def attach_interrupt(self, pin: str, callback: Callable[[BoardInputChange], None]) -> None:
        if self._functions[pin] != "INTERRUPT":
            raise ValueError("pin must have an INTERRUPT assignment for callbacks")
        if not callable(callback):
            raise ValueError("interrupt callback must be callable")
        channel, signal = self._pins[pin]

        def dispatch(event):
            if event.pin == signal:
                callback(
                    BoardInputChange(time_s=event.time_s, mcu=self._mcu, pin=pin, level=event.level)
                )

        self._replay.on_change(channel, dispatch)

    def run_until(self, time_s: float) -> int:
        return self._replay.run_until(time_s)
