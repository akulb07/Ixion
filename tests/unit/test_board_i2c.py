import pytest

from roboforge.hardware import RobotProject
from roboforge.hardware.board_i2c import MPU6050I2C, resolve_mpu6050_wiring
from roboforge.hardware.mpu6050 import MPU6050, decode_motion_burst
from tests.unit.test_board_inputs import project
from tests.unit.test_mpu6050 import sample


def adapter(model=None):
    return MPU6050I2C(
        resolve_mpu6050_wiring(model or project(), "esp32", "sensors", "imu"), MPU6050()
    )


def test_reference_wiring_and_sequential_read_write_timing():
    bus = adapter()
    assert (bus.wiring.sda_pin, bus.wiring.scl_pin, bus.wiring.address) == (
        "GPIO21",
        "GPIO22",
        0x68,
    )
    identity = bus.read_registers(0x68, 0x75, 1)
    assert identity.data == (0x68,) and identity.sample_time_s is None
    assert identity.start_time_s == 0
    assert identity.completion_time_s == pytest.approx(36 / 100_000)
    wake = bus.write_registers(0x68, 0x6B, b"\x00")
    assert wake.start_time_s == identity.completion_time_s
    assert wake.completion_time_s - wake.start_time_s == pytest.approx(27 / 100_000)
    bus.device.publish(sample(bus.time_s))
    reading = bus.read_registers(0x68, 0x3B, 14)
    assert reading.completion_time_s - reading.start_time_s == pytest.approx(153 / 100_000)
    assert reading.sample_time_s == wake.completion_time_s
    assert decode_motion_burst(bytes(reading.data))["specific_force_m_s2"][0] == pytest.approx(
        9.80665
    )
    assert bus.time_s == reading.completion_time_s


def test_start_times_latency_and_overlap_checks_are_explicit():
    model = project().model_dump()
    model["i2c_buses"][0]["latency_s"] = 0.001
    bus = adapter(RobotProject.model_validate(model))
    transfer = bus.read_registers(0x68, 0x75, 1, at_s=1)
    assert transfer.start_time_s == 1
    assert transfer.completion_time_s == pytest.approx(1.00136)
    with pytest.raises(ValueError, match="overlaps"):
        bus.write_registers(0x68, 0x6B, b"\x00", at_s=1.0001)
    assert bus.device.sleeping and bus.time_s == transfer.completion_time_s


def test_unmodeled_address_or_register_does_not_change_bus_or_device():
    bus = adapter()
    with pytest.raises(ValueError, match="no modeled"):
        bus.read_registers(0x69, 0x75, 1)
    with pytest.raises(ValueError, match="unsupported"):
        bus.write_registers(0x68, 0x6B, b"\x20")
    assert bus.time_s == 0 and bus.device.sleeping
    assert bus.read_registers(0x68, 0x75, 1).data == (0x68,)


def test_declared_alternate_address_keeps_who_am_i_identity():
    data = project().model_dump()
    data["i2c_buses"][0]["devices"][0]["address"] = 0x69
    bus = adapter(RobotProject.model_validate(data))
    assert bus.read_registers(0x69, 0x75, 1).data == (0x68,)
    with pytest.raises(ValueError, match="no modeled"):
        bus.read_registers(0x68, 0x75, 1)


@pytest.mark.parametrize(
    "issue",
    [
        "missing_assignment",
        "wrong_sensor",
        "bad_address",
        "too_fast",
        "wrong_sda",
        "wrong_direction",
    ],
)
def test_invalid_bus_configuration_rejects_execution(issue):
    data = project().model_dump()
    if issue == "missing_assignment":
        data["assignments"] = [a for a in data["assignments"] if a["endpoint"]["pin"] != "GPIO21"]
    elif issue == "wrong_sensor":
        next(c for c in data["components"] if c["id"] == "imu")["type"] = "unmodeled-imu"
    elif issue == "bad_address":
        data["i2c_buses"][0]["devices"][0]["address"] = 0x50
    elif issue == "too_fast":
        data["i2c_buses"][0]["frequency_hz"] = 500_000
    elif issue == "wrong_sda":
        for net in data["nets"]:
            for endpoint in net["endpoints"]:
                if endpoint["component"] == "imu" and endpoint["pin"] in {"SDA", "SCL"}:
                    endpoint["pin"] = {"SDA": "SCL", "SCL": "SDA"}[endpoint["pin"]]
    else:
        imu = next(c for c in data["components"] if c["id"] == "imu")
        next(p for p in imu["pins"] if p["id"] == "SDA")["kind"] = "output"
    with pytest.raises(ValueError):
        adapter(RobotProject.model_validate(data))


def test_transfer_budget_bounds_host_polling():
    bus = adapter()
    for _ in range(10_000):
        bus.read_registers(0x68, 0x75, 1)
    end = bus.time_s
    with pytest.raises(ValueError, match="transfer budget"):
        bus.read_registers(0x68, 0x75, 1)
    assert bus.time_s == end
