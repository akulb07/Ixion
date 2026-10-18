import pytest

from roboforge.hardware.encoder_faults import EncoderCounterReads, EncoderCountFault
from roboforge.hardware.signal_clock import SignalClock


def test_freeze_changes_only_counter_reads_at_the_scheduled_time():
    clock = SignalClock({"encoder": False})
    raw = {"left": 10, "right": 20}
    events = []
    reads = EncoderCounterReads(
        clock,
        lambda: raw,
        (EncoderCountFault(channel="left", start_time_s=0.1),),
        events.append,
    )
    clock.run_until(0.099)
    assert reads.read() == raw and reads.report()["activations"] == []
    raw["left"] = 15
    clock.run_until(0.1)
    raw.update(left=30, right=40)
    assert reads.read() == {"left": 15, "right": 40}
    assert raw == {"left": 30, "right": 40}
    assert events == [
        {
            "time_s": 0.1,
            "channel": "left",
            "kind": "freeze",
            "offset_counts": 0,
            "decoded_count_at_activation": 15,
        }
    ]
    events[0]["channel"] = "changed"
    report = reads.report()
    report["activations"][0]["kind"] = "changed"
    reads.read()["left"] = -100
    assert reads.report()["activations"][0]["kind"] == "freeze"
    assert reads.report()["activations"][0]["channel"] == "left"
    assert reads.read()["left"] == 15


@pytest.mark.parametrize("offset", [-100, 100])
def test_initial_offset_is_visible_immediately_and_preserves_future_count_deltas(offset):
    raw = {"left": 0, "right": 1}
    reads = EncoderCounterReads(
        SignalClock({"encoder": False}),
        lambda: raw,
        (EncoderCountFault(channel="left", start_time_s=0, kind="offset", offset_counts=offset),),
    )
    assert reads.read() == {"left": offset, "right": 1}
    raw["left"] = 30
    assert reads.read()["left"] == 30 + offset
    assert reads.report()["activations"][0]["time_s"] == 0


def test_freeze_and_equal_time_count_delivery_use_shared_clock_insertion_order():
    clock = SignalClock({"encoder": False})
    raw = {"left": 0, "right": 0}
    clock.call_at(0.1, lambda: raw.update(left=10, right=10))
    reads = EncoderCounterReads(
        clock,
        lambda: raw,
        tuple(EncoderCountFault(channel=name, start_time_s=0.1) for name in raw),
    )
    clock.call_at(0.1, lambda: raw.update(left=20, right=20))
    clock.run_until(0.1)
    assert reads.read() == {"left": 10, "right": 10}
    assert raw == {"left": 20, "right": 20}


@pytest.mark.parametrize(
    "invalid",
    [
        {"start_time_s": -1},
        {"start_time_s": float("nan")},
        {"start_time_s": True},
        {"channel": "rear"},
        {"kind": "disconnect"},
        {"kind": "offset"},
        {"offset_counts": 1},
        {"kind": "offset", "offset_counts": True},
        {"kind": "offset", "offset_counts": 1.5},
        {"kind": "offset", "offset_counts": 1_000_000_001},
    ],
)
def test_invalid_fault_configuration_is_rejected(invalid):
    with pytest.raises(ValueError):
        EncoderCountFault.model_validate({"channel": "left", "start_time_s": 0, **invalid})


def test_invalid_adapter_setup_does_not_schedule_partial_faults():
    clock = SignalClock({"encoder": False})
    raw = {"left": 0, "right": 0}
    fault = EncoderCountFault(channel="left", start_time_s=0.1)
    for faults in ((fault, fault), (fault, fault, fault)):
        with pytest.raises(ValueError, match="one encoder fault"):
            EncoderCounterReads(clock, lambda: raw, faults)
    for source, sink in ((None, None), (lambda: raw, 1)):
        with pytest.raises(ValueError, match="callable"):
            EncoderCounterReads(clock, source, (fault,), sink)
    clock.run_until(0.2)
    with pytest.raises(ValueError, match="before the shared clock"):
        EncoderCounterReads(clock, lambda: raw, (fault,))
