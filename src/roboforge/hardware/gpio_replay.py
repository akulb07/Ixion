"""Deterministic logical GPIO delivery for precomputed encoder traces."""

from collections.abc import Callable, Mapping
from typing import Literal

from roboforge.config import Nonnegative, Schema
from roboforge.core import nonnegative

from .quadrature import EncoderTrace


class GPIOChange(Schema):
    time_s: Nonnegative
    channel: str
    pin: Literal["a", "b"]
    level: bool


class EncoderGPIOReplay:
    """Ideal input pins and synchronous callbacks, without an MCU emulator.

    Equal-time events are delivered in sorted channel-name order, then source
    order within each channel. Pin state changes before its callback runs.
    """

    def __init__(self, traces: Mapping[str, EncoderTrace]):
        if not traces or len(traces) > 16 or any(not name for name in traces):
            raise ValueError("provide 1 to 16 named encoder traces")
        if sum(len(t.edges) for t in traces.values()) > 200_000:
            raise ValueError("GPIO replay exceeds 200,000-event budget")
        self._levels = {}
        self._events = []
        self._callbacks: dict[str, list[Callable[[GPIOChange], None]]] = {}
        for name, trace in sorted(traces.items()):
            if trace.initial.time_s != 0:
                raise ValueError("GPIO replay requires traces initialized at time zero")
            self._levels[name] = {"a": trace.initial.a, "b": trace.initial.b}
            self._callbacks[name] = []
            previous = trace.initial
            for edge in trace.edges:
                if edge.time_s < previous.time_s:
                    raise ValueError("encoder events must be chronologically ordered")
                changed = [
                    pin for pin in ("a", "b") if getattr(edge, pin) != getattr(previous, pin)
                ]
                if len(changed) != 1:
                    raise ValueError("each encoder event must change exactly one pin")
                pin = changed[0]
                self._events.append(
                    GPIOChange(time_s=edge.time_s, channel=name, pin=pin, level=getattr(edge, pin))
                )
                previous = edge
        self._events.sort(key=lambda e: (e.time_s, e.channel))
        self._index = 0
        self._time_s = 0.0
        self._running = False
        self._started = False
        self._failed = False

    @property
    def time_s(self) -> float:
        return self._time_s

    @property
    def pending_events(self) -> int:
        return len(self._events) - self._index

    def read(self, channel: str, pin: Literal["a", "b"]) -> bool:
        return self._levels[channel][pin]

    def on_change(self, channel: str, callback: Callable[[GPIOChange], None]) -> None:
        if self._started:
            raise ValueError("register callbacks before starting replay")
        if not callable(callback):
            raise ValueError("GPIO callback must be callable")
        self._callbacks[channel].append(callback)

    def run_until(self, time_s: float) -> int:
        end = nonnegative(time_s, "GPIO replay time")
        if self._running or self._failed:
            raise ValueError("cannot reenter or resume a failed GPIO replay")
        if end < self._time_s:
            raise ValueError("GPIO replay cannot move backwards in time")
        delivered = 0
        self._started = self._running = True
        try:
            while self._index < len(self._events) and self._events[self._index].time_s <= end:
                event = self._events[self._index]
                self._time_s = event.time_s
                self._levels[event.channel][event.pin] = event.level
                self._index += 1
                delivered += 1
                for callback in self._callbacks[event.channel]:
                    callback(event)
            self._time_s = end
        except BaseException:
            self._failed = True
            raise
        finally:
            self._running = False
        return delivered
