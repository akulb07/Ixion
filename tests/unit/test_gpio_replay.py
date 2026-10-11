import math

import pytest

from roboforge.hardware.gpio_replay import EncoderGPIOReplay
from roboforge.hardware.quadrature import AngularSample, QuadratureEncoder


def trace(reverse=False):
    return QuadratureEncoder(cycles_per_revolution=2).trace(
        (
            AngularSample(time_s=0, angle_rad=0),
            AngularSample(time_s=1, angle_rad=(-1 if reverse else 1) * math.tau),
        )
    )


def test_callbacks_decode_actual_pin_reads_in_both_directions():
    for reverse in (False, True):
        source = trace(reverse)
        replay = EncoderGPIOReplay({"wheel": source})
        phases = [(False, False), (True, False), (True, True), (False, True)]
        position = 0
        count = 0

        def decode(event):
            nonlocal position, count
            assert replay.time_s == event.time_s
            assert replay.read("wheel", event.pin) == event.level
            current = phases.index((replay.read("wheel", "a"), replay.read("wheel", "b")))
            delta = (current - position) % 4
            assert delta in (1, 3)
            count += 1 if delta == 1 else -1
            position = current

        replay.on_change("wheel", decode)
        assert replay.run_until(1) == 8
        assert count == source.final_count
        assert replay.pending_events == 0


def test_chunked_replay_has_no_duplicate_edges_and_stable_ties():
    replay = EncoderGPIOReplay({"right": trace(), "left": trace()})
    seen = []
    for channel in ("left", "right"):
        replay.on_change(channel, lambda event: seen.append((event.time_s, event.channel)))
    first = trace().edges[0].time_s
    assert replay.run_until(first) == 2
    assert replay.run_until(first) == 0
    replay.run_until(0.5)
    replay.run_until(1)
    assert seen == sorted(seen)
    assert len(seen) == 16
    assert seen[:2] == [(first, "left"), (first, "right")]


def test_failure_is_not_silently_resumed():
    replay = EncoderGPIOReplay({"wheel": trace()})

    def fail(event):
        raise RuntimeError("firmware error")

    replay.on_change("wheel", fail)
    with pytest.raises(RuntimeError, match="firmware error"):
        replay.run_until(1)
    with pytest.raises(ValueError, match="failed"):
        replay.run_until(1)


def test_reentry_time_travel_and_late_registration_rejected():
    replay = EncoderGPIOReplay({"wheel": trace()})
    replay.run_until(0.1)
    with pytest.raises(ValueError):
        replay.run_until(0)
    with pytest.raises(ValueError):
        replay.on_change("wheel", lambda e: None)
    other = EncoderGPIOReplay({"wheel": trace()})
    other.on_change("wheel", lambda e: other.run_until(1))
    with pytest.raises(ValueError, match="reenter"):
        other.run_until(1)
