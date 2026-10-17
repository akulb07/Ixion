import pytest

from roboforge.hardware.serial_console import SerialConsole, SerialConsoleConfig
from roboforge.hardware.signal_clock import SignalClock, SignalEdge


def console(**settings):
    clock = SignalClock({"encoder": False})
    return SerialConsole(clock, SerialConsoleConfig(**settings)), clock


def test_utf8_byte_count_and_fifo_delivery_follow_baud_timing():
    serial, clock = console(baud_rate=1000)
    assert serial.write("µ\n")
    assert serial.write("ok")
    report = serial.report()
    assert [t["byte_count"] for t in report["pending"]] == [3, 2]
    assert report["pending"][0]["completion_time_s"] == 0.03
    assert report["pending"][1]["start_time_s"] == 0.03
    assert report["pending"][1]["completion_time_s"] == 0.05
    clock.run_until(0.029)
    assert serial.received_text == ""
    clock.run_until(0.03)
    assert serial.received_text == "µ\n"
    clock.run_until(0.05)
    assert serial.received_text == "µ\nok"
    assert serial.report()["buffered_bytes"] == 0


def test_full_buffer_drops_whole_write_then_recovers_without_blocking_gpio():
    serial, clock = console(baud_rate=1000, buffer_bytes=4)
    edges = []
    clock.on_change("encoder", lambda event: edges.append(event.time_s))
    clock.schedule((SignalEdge(time_s=0.01, pin="encoder", level=True),))
    assert serial.write("abcd")
    assert serial.write("x") is False
    assert clock.time_s == 0
    clock.run_until(0.02)
    assert edges == [0.01] and serial.received_text == ""
    assert serial.write("y") is False
    clock.run_until(0.04)
    assert serial.write("z")
    clock.run_until(0.05)
    report = serial.report()
    assert serial.received_text == "abcdz"
    assert report["accepted_bytes"] == 5 and report["attempted_bytes"] == 7
    assert len(report["dropped_writes"]) == 2
    assert [t["sequence"] for t in report["completed"]] == [0, 1]


def test_run_end_preserves_pending_messages_without_future_text():
    serial, clock = console(baud_rate=1000)
    serial.write("hello")
    clock.run_until(0.01)
    assert serial.received_text == ""
    assert serial.report()["completed"] == []
    assert serial.report()["pending"][0]["completion_time_s"] == 0.05


def test_idle_console_starts_new_transmission_at_current_time():
    serial, clock = console(baud_rate=1000)
    clock.run_until(1)
    serial.write("x")
    transmission = serial.report()["pending"][0]
    assert transmission["requested_time_s"] == transmission["start_time_s"] == 1
    assert transmission["completion_time_s"] == 1.01


def test_empty_write_is_noop_and_budgets_include_dropped_attempts():
    serial, clock = console(buffer_bytes=1, max_writes=2, max_total_bytes=3)
    assert serial.write("")
    assert serial.report()["write_attempts"] == 0
    assert serial.write("xy") is False
    assert serial.write("z")
    before = serial.report()
    with pytest.raises(ValueError, match="write-attempt budget"):
        serial.write("a")
    assert serial.report() == before


def test_byte_budget_rejection_is_atomic():
    serial, clock = console(max_total_bytes=3)
    serial.write("µ")
    before = serial.report()
    with pytest.raises(ValueError, match="attempted-byte budget"):
        serial.write("µ")
    assert serial.report() == before


def test_invalid_text_does_not_reserve_a_transmission():
    serial, clock = console()
    for value in (b"x", "\ud800", "x" * 65_537):
        with pytest.raises(ValueError):
            serial.write(value)
    assert serial.report()["write_attempts"] == 0
    assert serial.report()["pending"] == []
