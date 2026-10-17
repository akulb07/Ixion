"""Bounded nonblocking debug output with explicit UTF-8 / 8N1 transmit timing."""

from pydantic import Field

from roboforge.config import Nonnegative, Positive, Schema
from roboforge.core import finite

from .signal_clock import SignalClock


class SerialConsoleConfig(Schema):
    baud_rate: int = Field(strict=True, ge=1, le=3_000_000, default=115_200)
    buffer_bytes: int = Field(strict=True, ge=1, le=65_536, default=2_048)
    max_total_bytes: int = Field(strict=True, ge=1, le=1_048_576, default=65_536)
    max_writes: int = Field(strict=True, ge=1, le=10_000, default=10_000)


class SerialTransmission(Schema):
    sequence: int = Field(strict=True, ge=0)
    requested_time_s: Nonnegative
    start_time_s: Nonnegative
    completion_time_s: Positive
    byte_count: int = Field(strict=True, gt=0)
    text: str


class SerialConsole:
    """An ideal logical console, independent of UART pins, USB and host wall time.

    Writes queue whole messages or return False on buffer overflow. Pending
    messages occupy their full byte count until whole-message completion; this
    conservative queue policy is explicit rather than a byte-level UART model.
    """

    def __init__(self, clock: SignalClock, config: SerialConsoleConfig):
        self._clock, self._config = clock, config
        self._pending = {}
        self._completed = []
        self._drops = []
        self._buffered = self._attempted_bytes = self._writes = self._accepted_bytes = 0
        self._available_time = clock.time_s

    def write(self, text: str) -> bool:
        if not isinstance(text, str) or len(text) > 65_536:
            raise ValueError("serial write requires text of at most 65,536 characters")
        try:
            size = len(text.encode("utf-8"))
        except UnicodeEncodeError as exc:
            raise ValueError("serial text must encode as UTF-8") from exc
        if size == 0:
            return True
        if size > 65_536:
            raise ValueError("serial write exceeds 65,536 UTF-8 bytes")
        if self._writes >= self._config.max_writes:
            raise ValueError("serial console exceeds its write-attempt budget")
        if self._attempted_bytes + size > self._config.max_total_bytes:
            raise ValueError("serial console exceeds its attempted-byte budget")
        if self._buffered + size > self._config.buffer_bytes:
            self._drops.append(
                {"time_s": self._clock.time_s, "byte_count": size, "reason": "buffer_full"}
            )
            self._writes += 1
            self._attempted_bytes += size
            return False
        start = max(self._clock.time_s, self._available_time)
        completion = finite(start + 10 * size / self._config.baud_rate, "serial completion")
        if completion <= start:
            raise ValueError("serial duration is below timestamp precision")
        transmission = SerialTransmission(
            sequence=len(self._completed) + len(self._pending),
            requested_time_s=self._clock.time_s,
            start_time_s=start,
            completion_time_s=completion,
            byte_count=size,
            text=text,
        )

        def complete():
            self._pending.pop(transmission.sequence)
            self._buffered -= size
            self._completed.append(transmission)

        # Reserve the completion before mutating console state, so a rejected
        # clock reservation cannot leave a phantom pending message.
        self._clock.call_at(completion, complete)
        self._pending[transmission.sequence] = transmission
        self._buffered += size
        self._writes += 1
        self._attempted_bytes += size
        self._accepted_bytes += size
        self._available_time = completion
        return True

    @property
    def received_text(self) -> str:
        return "".join(t.text for t in self._completed)

    def report(self) -> dict:
        return {
            "config": self._config.model_dump(),
            "encoding": "UTF-8",
            "framing": "8N1",
            "bits_per_byte": 10,
            "write_attempts": self._writes,
            "attempted_bytes": self._attempted_bytes,
            "accepted_bytes": self._accepted_bytes,
            "buffered_bytes": self._buffered,
            "completed": [t.model_dump() for t in self._completed],
            "pending": [t.model_dump() for t in self._pending.values()],
            "dropped_writes": [dict(drop) for drop in self._drops],
            "limitations": [
                "Logical debug console; no project UART pins, physical serial port or USB behavior",
                "Whole-message completion and buffer release; no partial-byte delivery",
                "Nonblocking whole-write rejection on buffer overflow; no CPU cost or control delay",
            ],
        }
