import pytest

from roboforge.hardware.signal_clock import SignalClock, SignalEdge
from roboforge.hardware.ultrasonic import UltrasonicConfig, UltrasonicSensor


def edge(time, level, pin="echo"):
    return SignalEdge(time_s=time, pin=pin, level=level)


def test_ultrasonic_distance_is_recovered_from_pin_pulse():
    pulse = UltrasonicSensor(UltrasonicConfig(response_delay_s=0.0002)).trigger(0, 10e-6, 1)
    clock = SignalClock({"echo": False})
    clock.schedule((edge(pulse.echo_rise_s, True), edge(pulse.echo_fall_s, False)))
    width = clock.pulse_width("echo", 0.03)
    assert width * 340 / 2 == pytest.approx(1)
    assert clock.time_s == pulse.echo_fall_s
    assert clock.read("echo") is False


def test_missing_echo_advances_to_deadline():
    clock = SignalClock({"echo": False})
    assert clock.pulse_width("echo", 0.03) is None
    assert clock.time_s == 0.03


def test_already_high_pulse_is_skipped_and_other_pins_are_delivered():
    clock = SignalClock({"echo": True, "encoder": False})
    seen = []
    clock.on_change("encoder", lambda event: seen.append(event.time_s))
    clock.schedule(
        (edge(0.01, False), edge(0.02, True), edge(0.03, False), edge(0.015, True, "encoder"))
    )
    assert clock.pulse_width("echo", 0.1) == pytest.approx(0.01)
    assert seen == [0.015]


def test_whole_batch_is_rejected_and_equal_time_order_is_stable():
    clock = SignalClock({"echo": False, "encoder": False})
    with pytest.raises(ValueError):
        clock.schedule((edge(0.1, True), edge(0.1, False)))
    seen = []
    for pin in ("echo", "encoder"):
        clock.on_change(pin, lambda event: seen.append(event.pin))
    clock.schedule((edge(0.1, True, "encoder"), edge(0.1, True)))
    clock.run_until(0.1)
    assert seen == ["encoder", "echo"]


def test_falling_edge_at_deadline_is_included_and_no_duplicates():
    clock = SignalClock({"echo": False})
    clock.schedule((edge(0.01, True), edge(0.03, False)))
    assert clock.pulse_width("echo", 0.03) == pytest.approx(0.02)
    assert clock.pulse_width("echo", 0.03) is None
    assert clock.time_s == 0.06


def test_callback_failure_stops_clock():
    clock = SignalClock({"echo": False})
    clock.on_change("echo", lambda event: clock.run_until(1))
    clock.schedule((edge(0.1, True),))
    with pytest.raises(ValueError, match="reenter"):
        clock.run_until(1)
    with pytest.raises(ValueError, match="failed"):
        clock.run_until(1)


def test_peripheral_callbacks_and_gpio_share_stable_equal_time_order():
    clock = SignalClock({"echo": False})
    seen = []
    clock.call_at(0.1, lambda: seen.append(("before", clock.time_s)))
    clock.on_change("echo", lambda event: seen.append(("edge", clock.time_s)))
    clock.schedule((edge(0.1, True),))

    def after():
        seen.append(("after", clock.time_s))
        clock.call_at(0.1, lambda: seen.append(("nested", clock.time_s)))

    clock.call_at(0.1, after)
    clock.run_until(0.1)
    assert seen == [(name, 0.1) for name in ("before", "edge", "after", "nested")]


def test_gpio_pulse_capture_delivers_peripheral_callbacks_during_the_wait():
    clock = SignalClock({"echo": False})
    completed = []
    clock.call_at(0.02, lambda: completed.append(clock.time_s))
    clock.schedule((edge(0.01, True), edge(0.03, False)))
    assert clock.pulse_width("echo", 0.04) == pytest.approx(0.02)
    assert completed == [0.02]
