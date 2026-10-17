"""Deliver a sequential I2C adapter's responses on the shared simulation clock."""

from collections.abc import Callable

from .board_i2c import MPU6050I2C, I2CTransfer
from .signal_clock import SignalClock


class ScheduledMPU6050I2C:
    """One outstanding request; the completion callback may start another.

    The underlying adapter still snapshots data/applies writes at request start.
    Host code receives the transfer only at its scheduled completion. A clock
    scheduling failure is terminal because an accepted bus operation may exist.
    """

    def __init__(self, bus: MPU6050I2C, clock: SignalClock):
        if bus.time_s > clock.time_s:
            raise ValueError("attach an idle I2C adapter at or before the shared clock time")
        self._bus, self._clock = bus, clock
        self._pending = None
        self._completed = []
        self._failed = False

    @property
    def busy(self) -> bool:
        return self._pending is not None or self._clock.time_s < self._bus.time_s

    @property
    def pending(self) -> I2CTransfer | None:
        return self._pending

    @property
    def completed(self) -> tuple[I2CTransfer, ...]:
        return tuple(self._completed)

    def _request(self, operation, address, register, payload, on_complete):
        if self._failed or self.busy:
            raise ValueError("scheduled I2C adapter is busy or failed")
        if not callable(on_complete):
            raise ValueError("I2C completion callback must be callable")
        if operation == "read":
            transfer = self._bus.read_registers(address, register, payload, at_s=self._clock.time_s)
        else:
            transfer = self._bus.write_registers(
                address, register, payload, at_s=self._clock.time_s
            )
        self._pending = transfer

        def complete():
            self._pending = None
            self._completed.append(transfer)
            try:
                on_complete(transfer)
            except BaseException:
                self._failed = True
                raise

        try:
            self._clock.call_at(transfer.completion_time_s, complete)
        except BaseException:
            self._failed = True
            raise

    def read_registers(
        self, address: int, register: int, length: int, on_complete: Callable[[I2CTransfer], None]
    ) -> None:
        self._request("read", address, register, length, on_complete)

    def write_registers(
        self, address: int, register: int, data: bytes, on_complete: Callable[[I2CTransfer], None]
    ) -> None:
        self._request("write", address, register, data, on_complete)
