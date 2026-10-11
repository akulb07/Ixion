import pytest

from roboforge.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor


def sensor():
    return UltrasonicSensor(UltrasonicConfig(response_delay_s=0.0002))


@pytest.mark.parametrize("distance", [0.02, 1, 4])
def test_pulse_width_encodes_round_trip_distance(distance):
    pulse = sensor().trigger(1, 10e-6, distance)
    assert pulse.status == "echo"
    assert pulse.echo_rise_s == pytest.approx(1.00021)
    assert (pulse.echo_fall_s - pulse.echo_rise_s) * 340 / 2 == pytest.approx(distance)


@pytest.mark.parametrize("distance", [None, 0, 0.01, 4.1])
def test_out_of_range_returns_timeout_without_fabricated_echo(distance):
    pulse = sensor().trigger(0, 10e-6, distance)
    assert pulse.status == "no_echo"
    assert pulse.echo_rise_s is None and pulse.echo_fall_s is None
    assert pulse.timeout_time_s == pytest.approx(0.03001)


def test_short_trigger_does_not_start_measurement_cycle():
    device = sensor()
    assert device.trigger(0, 5e-6, 1).status == "short_trigger"
    assert device.trigger(0.00002, 10e-6, 1).status == "echo"


def test_early_trigger_rejected_without_postponing_next_valid_read():
    device = sensor()
    device.trigger(0, 10e-6, 1)
    with pytest.raises(ValueError, match="busy"):
        device.trigger(0.02, 10e-6, 1)
    assert device.trigger(0.061, 10e-6, 2).status == "echo"
    with pytest.raises(ValueError, match="backwards"):
        device.trigger(0.01, 10e-6, 1)


def test_invalid_window_and_nonfinite_distance_rejected():
    with pytest.raises(ValueError, match="timeout"):
        UltrasonicConfig(response_delay_s=1)
    device = sensor()
    with pytest.raises(ValueError):
        device.trigger(0, 10e-6, float("nan"))
    assert device.trigger(0, 10e-6, 1).status == "echo"
