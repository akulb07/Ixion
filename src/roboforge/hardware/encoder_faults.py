"""Deterministic faults in decoded counter reads, after logical GPIO delivery."""

from collections.abc import Callable, Mapping
from typing import Literal

from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Schema

from .signal_clock import SignalClock


class EncoderCountFault(Schema):
    channel: Literal["left", "right"]
    start_time_s: Nonnegative
    kind: Literal["freeze", "offset"] = "freeze"
    offset_counts: int = Field(strict=True, ge=-1_000_000_000, le=1_000_000_000, default=0)

    @model_validator(mode="after")
    def check_offset(self):
        if self.kind == "freeze" and self.offset_counts != 0:
            raise ValueError("freeze faults cannot add a count offset")
        if self.kind == "offset" and self.offset_counts == 0:
            raise ValueError("offset faults require a nonzero count offset")
        return self


class EncoderCounterReads:
    """At most one permanent counter-read fault per wheel, with no GPIO mutation."""

    def __init__(
        self,
        clock: SignalClock,
        raw_counts: Callable[[], Mapping[str, int]],
        faults: tuple[EncoderCountFault, ...] = (),
        on_fault: Callable[[dict], None] | None = None,
    ):
        if not callable(raw_counts) or (on_fault is not None and not callable(on_fault)):
            raise ValueError("encoder count source and optional event sink must be callable")
        if len(faults) > 2 or len({f.channel for f in faults}) != len(faults):
            raise ValueError("provide at most one encoder fault per wheel")
        if any(f.start_time_s < clock.time_s for f in faults):
            raise ValueError("encoder faults cannot start before the shared clock")
        self._clock, self._raw, self._faults = clock, raw_counts, faults
        self._active = {}
        self._events = []
        for fault in faults:

            def activate(fault=fault):
                raw = self._raw()[fault.channel]
                self._active[fault.channel] = (fault, raw)
                event = {
                    "time_s": clock.time_s,
                    "channel": fault.channel,
                    "kind": fault.kind,
                    "offset_counts": fault.offset_counts,
                    "decoded_count_at_activation": raw,
                }
                self._events.append(event)
                if on_fault is not None:
                    on_fault(dict(event))

            if fault.start_time_s == clock.time_s:
                activate()
            else:
                clock.call_at(fault.start_time_s, activate)

    def read(self) -> dict[str, int]:
        counts = dict(self._raw())
        for channel, (fault, frozen) in self._active.items():
            counts[channel] = (
                frozen if fault.kind == "freeze" else counts[channel] + fault.offset_counts
            )
        return counts

    def report(self) -> dict:
        return {
            "layer": "decoded_counter_read",
            "faults": [fault.model_dump(mode="json") for fault in self._faults],
            "activations": [dict(event) for event in self._events],
            "limitations": [
                "Counter-read faults do not alter physical motion, quadrature GPIO or the underlying decoder",
                "Faults persist after activation; at most one fault per wheel, with no recovery waveform",
                "Equal-time activations and GPIO use the shared clock's insertion order",
            ],
        }
