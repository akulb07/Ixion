"""Bounded simulation-time delivery and pulse capture for ideal digital inputs."""

import heapq
from collections.abc import Callable, Mapping

from pydantic import Field

from roboforge.config import Nonnegative, Schema
from roboforge.core import finite, nonnegative, positive


class SignalEdge(Schema):
    time_s: Nonnegative
    pin: str
    level: bool = Field(strict=True)


class SignalClock:
    def __init__(self, initial: Mapping[str, bool]):
        if (
            not initial
            or len(initial) > 128
            or any(not p or type(v) is not bool for p, v in initial.items())
        ):
            raise ValueError("provide 1 to 128 named pins with boolean initial levels")
        self._levels = dict(initial)
        self._callbacks: dict[str, list[Callable[[SignalEdge], None]]] = {
            pin: [] for pin in initial
        }
        self._events = []
        self._scheduled = set()
        self._sequence = 0
        self._time = 0.0
        self._running = self._failed = False

    @property
    def time_s(self) -> float:
        return self._time

    def read(self, pin: str) -> bool:
        return self._levels[pin]

    def on_change(self, pin: str, callback: Callable[[SignalEdge], None]) -> None:
        if not callable(callback):
            raise ValueError("callback must be callable")
        self._callbacks[pin].append(callback)

    def call_at(self, time_s: float, callback: Callable[[], None]) -> None:
        """Schedule a peripheral callback on the same ordered timeline as GPIO.

        Callbacks may queue more work, but cannot recursively advance the clock.
        GPIO edges and callbacks share the 200,000-event lifetime budget.
        """
        time = nonnegative(time_s, "callback time")
        if self._failed or time < self._time or not callable(callback):
            raise ValueError("callback needs a callable, a non-past time and a working clock")
        if self._sequence >= 200_000:
            raise ValueError("signal clock exceeds 200,000-event lifetime budget")
        heapq.heappush(self._events, (time, self._sequence, callback))
        self._sequence += 1

    def schedule(self, events: tuple[SignalEdge, ...]) -> None:
        if self._failed:
            raise ValueError("cannot schedule on failed signal clock")
        if self._sequence + len(events) > 200_000:
            raise ValueError("signal clock exceeds 200,000-event lifetime budget")
        keys = set()
        for event in events:
            key = (event.pin, event.time_s)
            if event.pin not in self._levels or event.time_s < self._time:
                raise ValueError("signal event needs a mapped pin and non-past time")
            if key in keys or key in self._scheduled:
                raise ValueError("only one event per pin at a timestamp is supported")
            keys.add(key)
        for event in events:
            heapq.heappush(self._events, (event.time_s, self._sequence, event))
            self._sequence += 1
        self._scheduled.update(keys)

    def run_until(self, time_s: float) -> None:
        end = nonnegative(time_s, "signal clock time")
        if self._running or self._failed or end < self._time:
            raise ValueError("cannot reenter, rewind or resume a failed signal clock")
        self._running = True
        try:
            while self._events and self._events[0][0] <= end:
                event_time, _, event = heapq.heappop(self._events)
                self._time = event_time
                if not isinstance(event, SignalEdge):
                    event()
                    continue
                old = self._levels[event.pin]
                self._levels[event.pin] = event.level
                if old != event.level:
                    for callback in tuple(self._callbacks[event.pin]):
                        callback(event)
            self._time = end
        except BaseException:
            self._failed = True
            raise
        finally:
            self._running = False

    def pulse_width(self, pin: str, timeout_s: float) -> float | None:
        """Wait for a fresh high pulse, with one total deadline; return None on timeout.

        An already-high input must fall before a new pulse can be measured.
        Advances only simulation time and delivers other pins' callbacks too.
        """
        if self._running or self._failed:
            raise ValueError("cannot capture from a callback or failed signal clock")
        end = finite(self._time + positive(timeout_s, "pulse timeout"), "pulse deadline")
        if end <= self._time:
            raise ValueError("pulse timeout is too small to advance simulation time")
        waiting_low = self.read(pin)
        rise = width = None

        def capture(event):
            nonlocal waiting_low, rise, width
            if waiting_low:
                if not event.level:
                    waiting_low = False
            elif rise is None and event.level:
                rise = event.time_s
            elif rise is not None and not event.level and width is None:
                width = event.time_s - rise

        self._callbacks[pin].append(capture)
        try:
            while self._events and self._events[0][0] <= end and width is None:
                self.run_until(self._events[0][0])
            if width is None:
                self.run_until(end)
            return width
        finally:
            self._callbacks[pin].remove(capture)
