"""Versioned component graph. Loading never executes component or firmware code."""

from pathlib import Path
from typing import Annotated, Literal

import yaml
from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Positive, Real, Schema, _UniqueKeyLoader

Identifier = Annotated[str, Field(pattern=r"^[A-Za-z][A-Za-z0-9_]*$", max_length=80)]
Capability = Literal[
    "POWER",
    "GROUND",
    "DIGITAL_IN",
    "DIGITAL_OUT",
    "PWM",
    "ADC",
    "DAC",
    "I2C_SDA",
    "I2C_SCL",
    "SPI",
    "UART",
    "INTERRUPT",
    "MOTOR_OUTPUT",
]


def _unique(values, label: str) -> None:
    if len(values) != len(set(values)):
        raise ValueError(f"duplicate {label}")


class Parameter(Schema):
    """Named engineering input with explicit units and evidence provenance.

    Units are stored verbatim; model-specific dimension checks belong to the
    consuming component model. '1' is explicitly dimensionless.
    """

    name: Identifier
    value: Real
    unit: Literal[
        "1", "V", "A", "ohm", "Ah", "kg", "m", "s", "Hz", "rpm", "N*m", "kg*m^2", "rad", "count/rev"
    ]
    evidence: Literal["assumed", "datasheet", "measured"] = "assumed"
    reference: str = Field(min_length=1, max_length=2000)


class VoltageRange(Schema):
    """Declared voltage envelope in volts, including expected variation."""

    minimum_v: Nonnegative
    maximum_v: Nonnegative
    reference: str = Field(min_length=1, max_length=2000)

    @model_validator(mode="after")
    def ordered(self):
        if self.minimum_v > self.maximum_v:
            raise ValueError("minimum voltage must not exceed maximum voltage")
        return self


class LogicLevels(Schema):
    """Input VIL/VIH limits or output VOL/VOH guarantees, in volts.

    The reference must identify supply, temperature and load conditions. These
    guarantees are user declarations, not derived from a nominal logic voltage.
    """

    low_max_v: Nonnegative
    high_min_v: Nonnegative
    reference: str = Field(min_length=1, max_length=2000)

    @model_validator(mode="after")
    def ordered(self):
        if self.low_max_v >= self.high_min_v:
            raise ValueError("logic low limit must be below logic high limit")
        return self


class Pin(Schema):
    """Physical endpoint capabilities, not an active GPIO mode assignment."""

    id: Identifier
    kind: Literal["power_in", "power_out", "ground", "input", "output", "bidirectional", "passive"]
    capabilities: tuple[Capability, ...] = Field(min_length=1)
    required: bool = False
    ground_reference: Identifier | None = None
    accepted_voltage: VoltageRange | None = None
    driven_voltage: VoltageRange | None = None
    input_levels: LogicLevels | None = None
    output_levels: LogicLevels | None = None

    @model_validator(mode="after")
    def unique_capabilities(self):
        _unique(self.capabilities, "pin capability")
        expected = {"power_in": "POWER", "power_out": "POWER", "ground": "GROUND"}.get(self.kind)
        if expected and expected not in self.capabilities:
            raise ValueError(f"{self.kind} pin requires {expected} capability")
        for levels, envelope in (
            (self.input_levels, self.accepted_voltage),
            (self.output_levels, self.driven_voltage),
        ):
            if (
                levels is not None
                and envelope is not None
                and not (
                    envelope.minimum_v <= levels.low_max_v < levels.high_min_v <= envelope.maximum_v
                )
            ):
                raise ValueError("logic levels must lie inside their declared voltage envelope")
        return self


class Component(Schema):
    """Serializable instance combining independent hardware domains."""

    id: Identifier
    type: str = Field(min_length=1, max_length=100)
    category: Literal[
        "mcu",
        "motor_driver",
        "motor",
        "encoder",
        "sensor",
        "battery",
        "regulator",
        "interface",
        "wheel",
        "chassis",
        "caster",
    ]
    manufacturer: str | None = None
    part_number: str | None = None
    variant: str | None = None
    datasheet: str | None = None
    pins: tuple[Pin, ...] = Field(default=(), max_length=128)
    parameters: tuple[Parameter, ...] = Field(default=(), max_length=128)

    @model_validator(mode="after")
    def unique_fields(self):
        _unique([p.id for p in self.pins], "pin ID")
        _unique([p.name for p in self.parameters], "parameter name")
        pins = {p.id: p for p in self.pins}
        for pin in self.pins:
            if pin.ground_reference is not None:
                reference = pins.get(pin.ground_reference)
                if reference is None or reference.kind != "ground" or pin.kind == "ground":
                    raise ValueError(
                        "ground_reference must name a local ground pin on a non-ground pin"
                    )
        return self


class PinRef(Schema):
    component: Identifier
    pin: Identifier


class Net(Schema):
    """Canonical connection; each physical pin appears in exactly one net at most."""

    id: Identifier
    endpoints: tuple[PinRef, ...] = Field(min_length=2, max_length=256)


class PinAssignment(Schema):
    """Requested MCU peripheral function; checked against pin capabilities."""

    endpoint: PinRef
    function: Literal["DIGITAL_IN", "DIGITAL_OUT", "PWM", "INTERRUPT", "I2C_SDA", "I2C_SCL"]


class I2CDevice(Schema):
    component: Identifier
    address: Annotated[int, Field(strict=True, ge=0x08, le=0x77)]


class I2CBus(Schema):
    id: Identifier
    controller: Identifier
    sda_net: Identifier
    scl_net: Identifier
    frequency_hz: Positive = 100_000
    latency_s: Nonnegative = 0
    devices: tuple[I2CDevice, ...] = Field(min_length=1, max_length=112)


class MechanicalLink(Schema):
    """Topological coupling only; joint geometry/dynamics are future model data."""

    source: Identifier
    target: Identifier
    kind: Literal["mount", "shaft", "encoder_shaft"]


class RobotAssembly(Schema):
    """First assembly geometry. Mass includes all components, not just chassis."""

    drive: Literal["differential_drive"] = "differential_drive"
    total_mass_kg: Positive
    wheel_radius_m: Positive
    track_width_m: Positive
    links: tuple[MechanicalLink, ...] = Field(default=(), max_length=512)


class RobotProject(Schema):
    """Hardware topology, distinct from a legacy prescribed-motion RunConfig."""

    kind: Literal["hardware_project"] = "hardware_project"
    schema_version: Literal[1] = 1
    name: str = Field(min_length=1, max_length=200)
    assembly: RobotAssembly
    components: tuple[Component, ...] = Field(min_length=1, max_length=128)
    nets: tuple[Net, ...] = Field(default=(), max_length=512)
    i2c_buses: tuple[I2CBus, ...] = Field(default=(), max_length=16)
    assignments: tuple[PinAssignment, ...] = Field(default=(), max_length=512)

    @model_validator(mode="after")
    def graph_integrity(self):
        _unique([c.id for c in self.components], "component ID")
        _unique([n.id for n in self.nets], "net ID")
        _unique([b.id for b in self.i2c_buses], "bus ID")
        components = {c.id: c for c in self.components}
        pins = {(c.id, p.id) for c in self.components for p in c.pins}
        assigned = []
        for assignment in self.assignments:
            endpoint = assignment.endpoint
            key = (endpoint.component, endpoint.pin)
            if key not in pins or components[endpoint.component].category != "mcu":
                raise ValueError("pin assignment must reference an existing MCU pin")
            assigned.append(key)
        _unique(assigned, "MCU pin assignment")
        used = set()
        for net in self.nets:
            for endpoint in net.endpoints:
                key = (endpoint.component, endpoint.pin)
                if key not in pins:
                    raise ValueError(f"unknown pin {key} in net {net.id}")
                if key in used:
                    raise ValueError(f"pin {key} occurs more than once; merge connected nets")
                used.add(key)
        nets = {n.id for n in self.nets}
        bus_lines = set()
        for bus in self.i2c_buses:
            if bus.controller not in components or components[bus.controller].category != "mcu":
                raise ValueError("I2C controller must reference an MCU component")
            if bus.sda_net == bus.scl_net or not {bus.sda_net, bus.scl_net} <= nets:
                raise ValueError("I2C requires two distinct existing nets")
            if bus_lines & {bus.sda_net, bus.scl_net}:
                raise ValueError("I2C nets may belong to only one bus")
            bus_lines.update((bus.sda_net, bus.scl_net))
            _unique([d.component for d in bus.devices], "I2C device membership")
            if any(
                d.component not in components or d.component == bus.controller for d in bus.devices
            ):
                raise ValueError("I2C device must reference a distinct existing component")
        links = [(link.source, link.target, link.kind) for link in self.assembly.links]
        _unique(links, "mechanical link")
        for source, target, _ in links:
            if source == target or source not in components or target not in components:
                raise ValueError("mechanical link requires two distinct existing components")
        return self


def load_project(path: str | Path) -> RobotProject:
    """Load a bounded YAML/JSON project with duplicate-key and graph checks."""
    path = Path(path)
    try:
        with path.open("rb") as stream:
            data = stream.read(1024 * 1024 + 1)
        if len(data) > 1024 * 1024:
            raise ValueError("hardware project exceeds 1 MiB")
        return RobotProject.model_validate(yaml.load(data.decode("utf-8"), Loader=_UniqueKeyLoader))
    except (OSError, ValueError, yaml.YAMLError, RecursionError) as exc:
        raise ValueError(f"Invalid hardware project {path}: {exc}") from exc
