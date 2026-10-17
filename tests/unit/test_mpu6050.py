import math

import pytest

from roboforge.hardware.mpu6050 import MPU6050, IMUSample, decode_motion_burst


def sample(
    time=0,
    force=(9.80665, -4.903325, 0),
    angular=(0, math.radians(-10), math.radians(100)),
    temperature=36.53,
):
    return IMUSample(
        time_s=time,
        specific_force_m_s2=force,
        angular_velocity_rad_s=angular,
        temperature_c=temperature,
    )


def awake():
    device = MPU6050()
    device.write_registers(0x6B, b"\x00", 0)
    return device


def test_identity_and_reset_state_are_available_without_fabricating_measurements():
    device = MPU6050()
    assert device.read_registers(0x75, 1, 0) == b"\x68"
    assert device.read_registers(0x6B, 1, 0) == b"\x40"
    assert device.read_registers(0x1B, 2, 0) == b"\x00\x00"
    assert device.sleeping
    with pytest.raises(ValueError, match="measurement unavailable"):
        device.read_registers(0x3B, 14, 0)
    with pytest.raises(ValueError, match="wake"):
        device.publish(sample())


def test_coherent_big_endian_signed_motion_burst_and_si_decode():
    device = awake()
    device.publish(sample())
    data = device.read_registers(0x3B, 14, 0)
    counts = [int.from_bytes(data[i : i + 2], "big", signed=True) for i in range(0, 14, 2)]
    assert counts == [16384, -8192, 0, 0, 0, -1310, 13100]
    assert device.read_registers(0x3D, 2, 0.1) == b"\xe0\x00"
    decoded = decode_motion_burst(data)
    assert decoded["specific_force_m_s2"] == pytest.approx((9.80665, -4.903325, 0))
    assert decoded["angular_velocity_rad_s"] == pytest.approx(
        (0, math.radians(-10), math.radians(100))
    )
    assert decoded["temperature_c"] == 36.53
    assert device.sample_time_s == 0 and device.diagnostics == ()


@pytest.mark.parametrize(
    "selector,accel,gyro", [(0, 16384, 13100), (1, 8192, 6550), (2, 4096, 3280), (3, 2048, 1640)]
)
def test_supported_ranges_change_register_encoding(selector, accel, gyro):
    device = awake()
    device.write_registers(0x1B, bytes((selector << 3, selector << 3)), 0)
    device.publish(sample(force=(0, 0, 9.80665), angular=(0, 0, math.radians(100))))
    decoded = decode_motion_burst(device.read_registers(0x3B, 14, 0), selector, selector)
    assert decoded["raw_counts"][2] == accel
    assert decoded["raw_counts"][6] == gyro
    assert decoded["specific_force_m_s2"] == pytest.approx((0, 0, 9.80665))
    assert decoded["angular_velocity_rad_s"][2] == pytest.approx(math.radians(100))


def test_out_of_range_values_saturate_with_named_diagnostics_instead_of_wrapping():
    device = awake()
    device.publish(sample(force=(1000, -1000, 0), angular=(1000, 0, 0), temperature=500))
    decoded = decode_motion_burst(device.read_registers(0x3B, 14, 0))
    assert decoded["raw_counts"][:2] == [32767, -32768]
    assert decoded["raw_counts"][3:5] == [32767, 32767]
    assert set(device.diagnostics) == {
        "accel_x_clipped",
        "accel_y_clipped",
        "temperature_clipped",
        "gyro_x_clipped",
    }


def test_rejected_batch_preserves_configuration_and_latched_sample():
    device = awake()
    device.publish(sample())
    before = device.read_registers(0x3B, 14, 0)
    with pytest.raises(ValueError, match="other bits"):
        device.write_registers(0x1B, b"\x08\xff", 0.1)
    assert device.read_registers(0x1B, 2, 0) == b"\x00\x00"
    assert device.read_registers(0x3B, 14, 0) == before


def test_configuration_sleep_and_reset_require_fresh_samples():
    device = awake()
    device.publish(sample())
    device.write_registers(0x1C, b"\x08", 0.1)
    assert device.sample_time_s is None
    with pytest.raises(ValueError, match="fresh sample"):
        device.read_registers(0x3B, 14, 0.1)
    device.publish(sample(0.1))
    device.write_registers(0x6B, b"\x40", 0.2)
    assert device.sleeping
    with pytest.raises(ValueError, match="wake"):
        device.publish(sample(0.2))
    device.write_registers(0x6B, b"\x01", 0.3)
    device.publish(sample(0.3))
    device.write_registers(0x6B, b"\x80", 0.4)
    assert device.sleeping and device.sample_time_s is None
    assert device.read_registers(0x6B, 1, 0.4) == b"\x40"
    assert device.read_registers(0x1B, 2, 0.4) == b"\x00\x00"


def test_old_measurements_remain_identifiable_until_next_explicit_publication():
    device = awake()
    device.publish(sample(1))
    before = device.read_registers(0x3B, 14, 2)
    assert device.read_registers(0x3B, 14, 3) == before
    assert device.sample_time_s == 1
    with pytest.raises(ValueError, match="backwards"):
        device.publish(sample(2))
    with pytest.raises(ValueError, match="backwards"):
        device.read_registers(0x75, 1, 2)


@pytest.mark.parametrize(
    "register,data",
    [(0x75, b"\x00"), (0x3B, b"\x00"), (0x19, b"\x00"), (0x6B, b"\x20"), (0x1C, b"\x80")],
)
def test_unsupported_register_writes_are_explicit_errors(register, data):
    with pytest.raises(ValueError):
        awake().write_registers(register, data, 0)


@pytest.mark.parametrize(
    "register,length", [(0x19, 1), (0x1C, 2), (True, 1), (0xFF, 2), (0x3B, 15)]
)
def test_unsupported_reads_and_invalid_spans_are_rejected(register, length):
    with pytest.raises(ValueError):
        awake().read_registers(register, length, 0)
