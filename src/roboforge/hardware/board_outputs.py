"""Logical MCU output latches wired to the averaged TB6612 model."""

from collections.abc import Mapping

from roboforge.core import nonnegative

from .driver import DriverInputs
from .project import RobotProject
from .validation import check_project


class TB6612BoardOutputs:
    """Direct digital/PWM nets only. Initial low outputs are an explicit model default."""

    def __init__(self, project: RobotProject, mcu: str, driver: str):
        parts = {c.id: c for c in project.components}
        if mcu not in parts or parts[mcu].category != "mcu":
            raise ValueError("output adapter requires an existing MCU")
        if (
            driver not in parts
            or parts[driver].category != "motor_driver"
            or parts[driver].type != "tb6612fng"
        ):
            raise ValueError("output adapter requires a tb6612fng motor driver")
        self.diagnostics = check_project(project)
        errors = [d.code for d in self.diagnostics if d.severity == "ERROR"]
        if errors:
            raise ValueError("project wiring has errors: " + ", ".join(sorted(set(errors))))
        nets = {(e.component, e.pin): n for n in project.nets for e in n.endpoints}
        assignments = {
            (a.endpoint.component, a.endpoint.pin): a.function for a in project.assignments
        }
        driver_pins = {p.id: p for p in parts[driver].pins}
        self._pins = {}
        self._functions = {}
        for signal in ("AIN1", "AIN2", "PWMA", "BIN1", "BIN2", "PWMB", "STBY"):
            sink = driver_pins.get(signal)
            if sink is None or sink.kind != "input" or "DIGITAL_IN" not in sink.capabilities:
                raise ValueError(f"missing driver digital input {signal}")
            net = nets.get((driver, signal))
            if net is None or len(net.endpoints) != 2:
                raise ValueError("driver output adapter requires direct two-endpoint nets")
            sources = [e for e in net.endpoints if e.component == mcu]
            if len(sources) != 1:
                raise ValueError("driver input must connect directly to selected MCU")
            pin = sources[0].pin
            required = "PWM" if signal.startswith("PWM") else "DIGITAL_OUT"
            if assignments.get((mcu, pin)) != required:
                raise ValueError(f"{pin} must be assigned {required}")
            if pin in self._pins:
                raise ValueError("driver signals must use distinct MCU pins")
            self._pins[pin] = signal
            self._functions[pin] = required
        self._levels = {
            pin: 0.0 if function == "PWM" else False for pin, function in self._functions.items()
        }

    @property
    def pin_map(self) -> dict[str, str]:
        return dict(self._pins)

    @property
    def levels(self) -> dict[str, bool | float]:
        return dict(self._levels)

    def write(
        self, digital: Mapping[str, bool] | None = None, pwm: Mapping[str, float] | None = None
    ) -> None:
        """Validate the complete batch before changing output latches."""
        updated = dict(self._levels)
        for pin, value in (digital or {}).items():
            if self._functions.get(pin) != "DIGITAL_OUT" or type(value) is not bool:
                raise ValueError(
                    "digital writes require a mapped DIGITAL_OUT pin and boolean level"
                )
            updated[pin] = value
        for pin, value in (pwm or {}).items():
            if self._functions.get(pin) != "PWM":
                raise ValueError("PWM writes require a mapped PWM pin")
            duty = nonnegative(value, "PWM duty")
            if duty > 1:
                raise ValueError("PWM duty must be between zero and one")
            updated[pin] = duty
        self._levels = updated

    def inputs(self, channel: str) -> DriverInputs:
        if channel not in {"A", "B"}:
            raise ValueError("driver channel must be A or B")
        signals = {signal: self._levels[pin] for pin, signal in self._pins.items()}
        return DriverInputs(
            standby=signals["STBY"],
            in1=signals[f"{channel}IN1"],
            in2=signals[f"{channel}IN2"],
            pwm_duty=signals[f"PWM{channel}"],
        )
