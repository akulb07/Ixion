"""Project-wired HC-SR04 logical GPIO, optionally through an explicit ideal buffer."""

from collections.abc import Callable

from roboforge.config import Schema
from roboforge.core import finite, nonnegative, positive

from .project import Identifier, RobotProject
from .signal_clock import SignalClock, SignalEdge
from .ultrasonic import EchoMeasurement, UltrasonicConfig, UltrasonicSensor
from .validation import check_project


class UltrasonicWiring(Schema):
    mcu: Identifier
    sensor: Identifier
    trigger_pin: Identifier
    echo_pin: Identifier
    trigger_net: Identifier
    echo_nets: tuple[Identifier, ...]
    ideal_echo_interface: Identifier | None


def resolve_ultrasonic_wiring(
    project: RobotProject, mcu: str, sensor: str, ideal_echo_interface: str | None
) -> UltrasonicWiring:
    """Resolve direct trigger and direct/one-buffer echo paths; no voltage conversion."""
    parts = {c.id: c for c in project.components}
    if mcu not in parts or parts[mcu].category != "mcu":
        raise ValueError("ultrasonic GPIO requires an existing MCU")
    if (
        sensor not in parts
        or parts[sensor].category != "sensor"
        or parts[sensor].type != "hc-sr04-style"
    ):
        raise ValueError("ultrasonic GPIO requires an hc-sr04-style sensor")
    errors = {d.code for d in check_project(project) if d.severity == "ERROR"}
    if errors:
        raise ValueError("project wiring has errors: " + ", ".join(sorted(errors)))
    pins = {(c.id, p.id): p for c in project.components for p in c.pins}
    nets = {(e.component, e.pin): n for n in project.nets for e in n.endpoints}
    assignments = {(a.endpoint.component, a.endpoint.pin): a.function for a in project.assignments}

    def require_pin(component, pin, kind, capability):
        model = pins.get((component, pin))
        if model is None or model.kind != kind or capability not in model.capabilities:
            raise ValueError(f"{component}.{pin} must be a {kind} with {capability}")

    def other(component, pin):
        net = nets.get((component, pin))
        if net is None or len(net.endpoints) != 2:
            raise ValueError("ultrasonic GPIO requires two-endpoint signal nets")
        return net, next(e for e in net.endpoints if (e.component, e.pin) != (component, pin))

    require_pin(sensor, "TRIG", "input", "DIGITAL_IN")
    require_pin(sensor, "ECHO", "output", "DIGITAL_OUT")
    trigger_net, trigger = other(sensor, "TRIG")
    echo_net, echo = other(sensor, "ECHO")
    echo_nets = [echo_net.id]
    if ideal_echo_interface is not None:
        interface = parts.get(ideal_echo_interface)
        if (
            interface is None
            or interface.category != "interface"
            or interface.type not in {"level-interface-placeholder", "ideal-digital-buffer"}
        ):
            raise ValueError("selected ideal echo interface is not a supported logical buffer")
        require_pin(interface.id, "IN", "input", "DIGITAL_IN")
        require_pin(interface.id, "OUT", "output", "DIGITAL_OUT")
        if (echo.component, echo.pin) != (interface.id, "IN"):
            raise ValueError("sensor ECHO must connect directly to the selected interface IN")
        output_net, echo = other(interface.id, "OUT")
        echo_nets.append(output_net.id)
    for endpoint, functions, capability in (
        (trigger, {"DIGITAL_OUT"}, "DIGITAL_OUT"),
        (echo, {"DIGITAL_IN", "INTERRUPT"}, "DIGITAL_IN"),
    ):
        if endpoint.component != mcu:
            raise ValueError("ultrasonic signal must reach the selected MCU")
        pin = pins[(mcu, endpoint.pin)]
        direction_ok = pin.kind in (
            {"output", "bidirectional"}
            if capability == "DIGITAL_OUT"
            else {"input", "bidirectional"}
        )
        if (
            not direction_ok
            or capability not in pin.capabilities
            or assignments.get((mcu, endpoint.pin)) not in functions
        ):
            raise ValueError(
                f"{endpoint.pin} needs a compatible {'/'.join(sorted(functions))} assignment"
            )
    if trigger.pin == echo.pin:
        raise ValueError("ultrasonic trigger and echo require distinct MCU pins")
    return UltrasonicWiring(
        mcu=mcu,
        sensor=sensor,
        trigger_pin=trigger.pin,
        echo_pin=echo.pin,
        trigger_net=trigger_net.id,
        echo_nets=tuple(echo_nets),
        ideal_echo_interface=ideal_echo_interface,
    )


class UltrasonicBoardIO:
    """GPIO edges cause measurements. An ideal interface preserves logic without delay.

    Both mapped pins must already exist, initially low, on the shared clock.
    Distance is sampled by the fixture at the rising trigger edge. This is a
    host peripheral model, without power-net activation or analog electronics.
    """

    def __init__(
        self,
        wiring: UltrasonicWiring,
        clock: SignalClock,
        config: UltrasonicConfig,
        distance_at: Callable[[float], float | None],
        on_measurement: Callable[[EchoMeasurement, float | None], None] | None = None,
    ):
        if wiring.trigger_pin == wiring.echo_pin:
            raise ValueError("ultrasonic trigger and echo require distinct MCU pins")
        try:
            if clock.read(wiring.trigger_pin) or clock.read(wiring.echo_pin):
                raise ValueError("ultrasonic GPIO must initialize low")
        except KeyError as exc:
            raise ValueError("ultrasonic pins must exist on the signal clock") from exc
        if not callable(distance_at) or (
            on_measurement is not None and not callable(on_measurement)
        ):
            raise ValueError("ultrasonic fixture callbacks must be callable")
        self.wiring = wiring
        self._clock = clock
        self._sensor = UltrasonicSensor(config)
        self._distance_at = distance_at
        self._on_measurement = on_measurement
        self._rise = None
        self._distance = None
        clock.on_change(wiring.trigger_pin, self._trigger_changed)

    def digital_read(self, pin: str) -> bool:
        if pin != self.wiring.echo_pin:
            raise ValueError("ultrasonic read requires the mapped MCU echo pin")
        return self._clock.read(pin)

    def digital_write(self, pin: str, level: bool) -> None:
        if pin != self.wiring.trigger_pin or type(level) is not bool:
            raise ValueError("ultrasonic write requires the mapped trigger pin and boolean level")
        self._clock.schedule((SignalEdge(time_s=self._clock.time_s, pin=pin, level=level),))

    def trigger_pulse(self, pin: str, width_s: float) -> None:
        """Queue the MCU high/low edges without advancing mechanics or clock time."""
        if pin != self.wiring.trigger_pin or self._clock.read(pin):
            raise ValueError("trigger pulse requires the mapped MCU pin initially low")
        end = finite(self._clock.time_s + positive(width_s, "trigger pulse width"), "trigger end")
        if end <= self._clock.time_s:
            raise ValueError("trigger pulse is below timestamp precision")
        self._clock.schedule(
            (
                SignalEdge(time_s=self._clock.time_s, pin=pin, level=True),
                SignalEdge(time_s=end, pin=pin, level=False),
            )
        )

    def _trigger_changed(self, event: SignalEdge) -> None:
        if event.level:
            distance = self._distance_at(event.time_s)
            self._distance = None if distance is None else nonnegative(distance, "fixture distance")
            self._rise = event.time_s
        elif self._rise is not None:
            pulse = self._sensor.trigger(self._rise, event.time_s - self._rise, self._distance)
            if pulse.status == "echo":
                self._clock.schedule(
                    (
                        SignalEdge(time_s=pulse.echo_rise_s, pin=self.wiring.echo_pin, level=True),
                        SignalEdge(time_s=pulse.echo_fall_s, pin=self.wiring.echo_pin, level=False),
                    )
                )
            if self._on_measurement is not None:
                self._on_measurement(pulse, self._distance)
            self._rise = None
