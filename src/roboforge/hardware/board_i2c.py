"""One project-wired MPU6050 bus, with explicit sequential transfer timing."""

from typing import Annotated, Literal

from pydantic import Field

from roboforge.config import Nonnegative, Positive, Schema
from roboforge.core import finite, nonnegative

from .mpu6050 import MPU6050, register_span
from .project import Identifier, RobotProject
from .validation import check_project

Byte = Annotated[int, Field(strict=True, ge=0, le=255)]


class MPU6050Wiring(Schema):
    mcu: Identifier
    bus: Identifier
    component: Identifier
    address: Literal[0x68, 0x69]
    sda_pin: Identifier
    scl_pin: Identifier
    frequency_hz: Positive = Field(le=400_000)
    latency_s: Nonnegative


class I2CTransfer(Schema):
    operation: Literal["read", "write"]
    start_time_s: Nonnegative
    completion_time_s: Nonnegative
    address: Byte
    register_address: Byte
    data: tuple[Byte, ...] = Field(min_length=1, max_length=14)
    sample_time_s: Nonnegative | None


def resolve_mpu6050_wiring(
    project: RobotProject, mcu: str, bus_id: str, component: str
) -> MPU6050Wiring:
    parts = {c.id: c for c in project.components}
    if mcu not in parts or parts[mcu].category != "mcu":
        raise ValueError("I2C adapter requires an existing MCU")
    if (
        component not in parts
        or parts[component].category != "sensor"
        or parts[component].type != "mpu6050-style"
    ):
        raise ValueError("I2C adapter requires an mpu6050-style sensor")
    errors = {d.code for d in check_project(project) if d.severity == "ERROR"}
    if errors:
        raise ValueError("project wiring has errors: " + ", ".join(sorted(errors)))
    bus = next((b for b in project.i2c_buses if b.id == bus_id and b.controller == mcu), None)
    if bus is None:
        raise ValueError("selected I2C bus must belong to the selected MCU")
    device = next((d for d in bus.devices if d.component == component), None)
    if device is None or device.address not in {0x68, 0x69}:
        raise ValueError("MPU must be a member of the selected bus at address 0x68 or 0x69")
    if bus.frequency_hz > 400_000:
        raise ValueError("MPU6050 I2C frequency must not exceed 400 kHz")
    nets = {n.id: n for n in project.nets}
    assignments = {(a.endpoint.component, a.endpoint.pin): a.function for a in project.assignments}
    pin_models = {(c.id, p.id): p for c in project.components for p in c.pins}
    pins = []
    for net_id, signal, function in (
        (bus.sda_net, "SDA", "I2C_SDA"),
        (bus.scl_net, "SCL", "I2C_SCL"),
    ):
        endpoints = nets[net_id].endpoints
        mcu_pins = [e for e in endpoints if e.component == mcu]
        device_pins = [e for e in endpoints if e.component == component]
        if len(mcu_pins) != 1 or len(device_pins) != 1 or device_pins[0].pin != signal:
            raise ValueError(
                "MPU I2C lines need one MCU pin and the corresponding sensor SDA/SCL pin"
            )
        pin = mcu_pins[0].pin
        source = pin_models[(component, signal)]
        mcu_pin = pin_models[(mcu, pin)]
        sensor_kinds = {"bidirectional"} if signal == "SDA" else {"input", "bidirectional"}
        if source.kind not in sensor_kinds or mcu_pin.kind != "bidirectional":
            raise ValueError(
                "MPU I2C requires bidirectional MCU pins and compatible sensor directions"
            )
        if assignments.get((mcu, pin)) != function:
            raise ValueError(f"{pin} must be assigned {function}")
        pins.append(pin)
    return MPU6050Wiring(
        mcu=mcu,
        bus=bus_id,
        component=component,
        address=device.address,
        sda_pin=pins[0],
        scl_pin=pins[1],
        frequency_hz=bus.frequency_hz,
        latency_s=bus.latency_s,
    )


class MPU6050I2C:
    """Transactions return coherent data at start, then reserve the bus until completion.

    This bus owns an independent simulation clock. It does not advance the rover
    or GPIO clock; a future shared scheduler must coordinate those consumers.
    """

    def __init__(self, wiring: MPU6050Wiring, device: MPU6050):
        self.wiring = wiring
        self.device = device
        self._time = 0.0
        self._transfers = 0

    @property
    def time_s(self) -> float:
        return self._time

    def _timing(self, address, register, length, read, at_s):
        if type(address) is not int or address != self.wiring.address:
            raise ValueError("no modeled I2C device at the requested address")
        register_span(register, length)
        start = self._time if at_s is None else nonnegative(at_s, "I2C start time")
        if start < self._time:
            raise ValueError("I2C transfer overlaps a previous transfer or moves backwards")
        if self._transfers >= 10_000:
            raise ValueError("I2C adapter exceeds 10,000-transfer budget")
        # Address/register phases and data, each with one ACK/NACK bit.
        bits = 9 * ((3 if read else 2) + length)
        completion = finite(
            start + bits / self.wiring.frequency_hz + self.wiring.latency_s, "I2C completion"
        )
        if completion <= start:
            raise ValueError("I2C duration is below timestamp precision")
        return start, completion

    def read_registers(
        self, address: int, register: int, length: int, at_s: float | None = None
    ) -> I2CTransfer:
        start, completion = self._timing(address, register, length, True, at_s)
        data = self.device.read_registers(register, length, start)
        transfer = I2CTransfer(
            operation="read",
            start_time_s=start,
            completion_time_s=completion,
            address=address,
            register_address=register,
            data=tuple(data),
            sample_time_s=self.device.sample_time_s if 0x3B <= register <= 0x48 else None,
        )
        self._time, self._transfers = completion, self._transfers + 1
        return transfer

    def write_registers(
        self, address: int, register: int, data: bytes, at_s: float | None = None
    ) -> I2CTransfer:
        if not isinstance(data, bytes):
            raise ValueError("I2C writes require bytes")
        start, completion = self._timing(address, register, len(data), False, at_s)
        self.device.write_registers(register, data, start)
        transfer = I2CTransfer(
            operation="write",
            start_time_s=start,
            completion_time_s=completion,
            address=address,
            register_address=register,
            data=tuple(data),
            sample_time_s=None,
        )
        self._time, self._transfers = completion, self._transfers + 1
        return transfer
