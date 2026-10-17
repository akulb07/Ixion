import pytest

from roboforge.hardware.scheduled_i2c import ScheduledMPU6050I2C
from roboforge.hardware.signal_clock import SignalClock, SignalEdge
from tests.unit.test_board_i2c import adapter
from tests.unit.test_mpu6050 import sample


def test_response_arrives_only_at_completion_while_gpio_keeps_running():
    clock = SignalClock({"encoder": False})
    bus = adapter()
    scheduled = ScheduledMPU6050I2C(bus, clock)
    seen = []
    clock.on_change("encoder", lambda event: seen.append(("edge", clock.time_s)))
    clock.schedule((SignalEdge(time_s=0.0001, pin="encoder", level=True),))
    scheduled.read_registers(0x68, 0x75, 1, lambda transfer: seen.append(("read", clock.time_s)))
    assert scheduled.busy and scheduled.completed == ()
    clock.run_until(0.0002)
    assert seen == [("edge", 0.0001)]
    assert scheduled.pending.completion_time_s == pytest.approx(0.00036)
    clock.run_until(0.00036)
    assert seen == [("edge", 0.0001), ("read", 0.00036)]
    assert not scheduled.busy and scheduled.pending is None
    assert scheduled.completed[0].data == (0x68,)


def test_completion_callback_can_chain_another_transfer():
    clock = SignalClock({"unused": False})
    bus = adapter()
    scheduled = ScheduledMPU6050I2C(bus, clock)
    seen = []

    def identity(transfer):
        scheduled.write_registers(0x68, 0x6B, b"\x00", lambda result: seen.append(result))

    scheduled.read_registers(0x68, 0x75, 1, identity)
    clock.run_until(0.00063)
    assert len(seen) == 1 and not bus.device.sleeping
    assert len(scheduled.completed) == 2


def test_read_snapshots_old_frame_but_delivers_after_new_capture():
    clock = SignalClock({"unused": False})
    bus = adapter()
    bus.write_registers(0x68, 0x6B, b"\x00")
    clock.run_until(bus.time_s)
    bus.device.publish(sample(clock.time_s))
    scheduled = ScheduledMPU6050I2C(bus, clock)
    responses = []
    scheduled.read_registers(0x68, 0x3B, 14, responses.append)
    clock.call_at(0.001, lambda: bus.device.publish(sample(0.001, force=(0, 0, 9.80665))))
    clock.run_until(0.002)
    assert responses[0].sample_time_s == pytest.approx(0.00027)
    assert responses[0].data[:2] == (0x40, 0)
    assert bus.device.sample_time_s == 0.001


def test_busy_rejected_request_does_not_replace_pending_transfer():
    clock = SignalClock({"unused": False})
    scheduled = ScheduledMPU6050I2C(adapter(), clock)
    scheduled.read_registers(0x68, 0x75, 1, lambda result: None)
    pending = scheduled.pending
    with pytest.raises(ValueError, match="busy"):
        scheduled.write_registers(0x68, 0x6B, b"\x00", lambda result: None)
    assert scheduled.pending == pending
    clock.run_until(0.00036)
    assert len(scheduled.completed) == 1


def test_bad_request_or_callback_does_not_reserve_clock_or_bus():
    clock = SignalClock({"unused": False})
    bus = adapter()
    scheduled = ScheduledMPU6050I2C(bus, clock)
    with pytest.raises(ValueError, match="callable"):
        scheduled.read_registers(0x68, 0x75, 1, None)
    with pytest.raises(ValueError, match="unsupported"):
        scheduled.read_registers(0x68, 0x19, 1, lambda result: None)
    assert bus.time_s == clock.time_s == 0 and not scheduled.busy


def test_completion_failure_is_terminal_on_both_adapter_and_clock():
    clock = SignalClock({"unused": False})
    scheduled = ScheduledMPU6050I2C(adapter(), clock)

    def fail(transfer):
        raise RuntimeError("controller failed")

    scheduled.read_registers(0x68, 0x75, 1, fail)
    with pytest.raises(RuntimeError, match="controller failed"):
        clock.run_until(0.001)
    with pytest.raises(ValueError, match="failed"):
        scheduled.read_registers(0x68, 0x75, 1, lambda result: None)
    with pytest.raises(ValueError, match="failed"):
        clock.run_until(0.002)
