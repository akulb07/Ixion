"""Host controller boundary for the straight-line, latched-brake rover fixture."""

from collections.abc import Callable
from typing import Literal, Protocol

from pydantic import Field, field_validator, model_validator

from roboforge.config import Nonnegative, Positive, Probability, Schema

from .project import Identifier
from .rover_imu import IMUObservation
from .voltage_monitor import VoltageObservation


class FirmwareInputs(Schema):
    """Delivered MCU observations, without access to the physics or wall fixture."""

    time_s: Nonnegative
    left_encoder_count: int = Field(strict=True)
    right_encoder_count: int = Field(strict=True)


class RangeObservation(Schema):
    time_s: Nonnegative
    status: Literal["echo", "timeout"]
    distance_m: Positive | None = None

    @model_validator(mode="after")
    def check_distance(self):
        if (self.status == "echo") != (self.distance_m is not None):
            raise ValueError("an echo needs a distance; a timeout must not invent one")
        return self


class BrakeRequest(Schema):
    reason: Identifier


class DriveRequest(Schema):
    """Equal forward PWM on both wheels; stopping uses an explicit brake request."""

    pwm_duty: Probability = Field(gt=0)


class FirmwareLog(Schema):
    text: str = Field(strict=True, min_length=1, max_length=4096)
    level: Literal["debug", "info", "warning", "error"] = "info"

    @field_validator("text")
    @classmethod
    def check_utf8(cls, text):
        try:
            size = len(text.encode("utf-8"))
        except UnicodeEncodeError as exc:
            raise ValueError("firmware log must encode as UTF-8") from exc
        if size > 4096:
            raise ValueError("firmware log exceeds 4,096 UTF-8 bytes")
        return text


class FirmwareResponse(Schema):
    brake_request: BrakeRequest | None = None
    drive_request: DriveRequest | None = None
    logs: tuple[FirmwareLog, ...] = Field(default=(), max_length=16)

    @model_validator(mode="after")
    def check_command(self):
        if self.brake_request is not None and self.drive_request is not None:
            raise ValueError("choose a drive request or a brake request, not both")
        return self


FirmwareResult = BrakeRequest | DriveRequest | FirmwareResponse | None


class RoverFirmware(Protocol):
    """Return a request, a response with debug logs, or None.

    Range callbacks run at delivery time. Tick callbacks run only at configured
    control boundaries. Both receive immutable observations. Applying the brake
    is deferred until a control tick; the fixture starts with both motors driving.
    """

    def on_range(self, observation: RangeObservation, inputs: FirmwareInputs) -> FirmwareResult: ...

    def on_tick(self, inputs: FirmwareInputs) -> FirmwareResult: ...


class IMURoverFirmware(RoverFirmware, Protocol):
    """Optional extension; existing range/tick programs remain valid."""

    def on_imu(self, observation: IMUObservation, inputs: FirmwareInputs) -> FirmwareResult: ...


class ObstacleBrakeFirmware(Schema):
    brake_threshold_m: Positive
    timeout_action: Literal["brake", "continue"] = "brake"

    def on_range(
        self, observation: RangeObservation, inputs: FirmwareInputs
    ) -> BrakeRequest | None:
        if observation.status == "echo" and observation.distance_m <= self.brake_threshold_m:
            return BrakeRequest(reason="obstacle")
        if observation.status == "timeout" and self.timeout_action == "brake":
            return BrakeRequest(reason="echo_timeout")
        return None

    def on_tick(self, inputs: FirmwareInputs) -> BrakeRequest | None:
        return None


class HostRoverFirmware:
    """Bounded callback contract; trusted Python, with no CPU or process isolation."""

    def __init__(
        self,
        program: RoverFirmware,
        *,
        on_log: Callable[[FirmwareLog, str], bool] | None = None,
    ):
        if any(not callable(getattr(program, method, None)) for method in ("on_range", "on_tick")):
            raise ValueError("rover firmware requires callable on_range and on_tick methods")
        imu_callback = getattr(program, "on_imu", None)
        if imu_callback is not None and not callable(imu_callback):
            raise ValueError("optional on_imu firmware method must be callable")
        voltage_callback = getattr(program, "on_voltage", None)
        if voltage_callback is not None and not callable(voltage_callback):
            raise ValueError("optional on_voltage firmware method must be callable")
        self._has_voltage_callback = voltage_callback is not None
        self._voltage_callback_count = 0
        if on_log is not None and not callable(on_log):
            raise ValueError("firmware log sink must be callable")
        self._program = program
        self._on_log = on_log
        self._log_attempts = self._logs_accepted = self._logs_rejected = 0
        self._has_imu_callback = imu_callback is not None
        self._imu_callback_count = 0
        self._request = None
        self._pending_drive = None
        self._pwm_duty = 1.0
        self._drive_request_time = None
        self._drive_requests = self._drive_commands = self._drive_ignored = 0
        self._detection_time = None
        self._command = "drive"
        self._time = 0.0
        self._running = self._failed = False

    @property
    def request(self) -> BrakeRequest | None:
        return self._request

    @property
    def detection_time_s(self) -> float | None:
        return self._detection_time

    @property
    def command(self) -> Literal["drive", "brake"]:
        return self._command

    @property
    def pwm_duty(self) -> float:
        return self._pwm_duty

    @property
    def drive_request_time_s(self) -> float | None:
        return self._drive_request_time

    def _invoke(self, method: str, inputs: FirmwareInputs, *args) -> None:
        if self._running or self._failed or inputs.time_s < self._time:
            raise ValueError("cannot reenter, rewind or resume failed host firmware")
        self._running = True
        try:
            result = getattr(self._program, method)(*args, inputs)
            logs = ()
            drive = None
            if isinstance(result, FirmwareResponse):
                response = FirmwareResponse.model_validate(result.model_dump())
                request, logs = response.brake_request, response.logs
                drive = response.drive_request
            elif isinstance(result, BrakeRequest):
                request = BrakeRequest.model_validate(result.model_dump())
            elif isinstance(result, DriveRequest):
                drive = DriveRequest.model_validate(result.model_dump())
                request = None
            elif result is None:
                request = None
            else:
                raise ValueError(
                    "callback must return BrakeRequest, DriveRequest, FirmwareResponse or None"
                )
            if request is not None and self._request is None:
                self._request = request
                self._detection_time = inputs.time_s
                self._pending_drive = None
            if drive is not None:
                self._drive_requests += 1
                if self._request is not None:
                    self._drive_ignored += 1
                else:
                    self._pending_drive = (drive, inputs.time_s)
            for entry in logs:
                accepted = False if self._on_log is None else self._on_log(entry, method)
                if type(accepted) is not bool:
                    raise ValueError("firmware log sink must return a boolean")
                self._log_attempts += 1
                self._logs_accepted += int(accepted)
                self._logs_rejected += int(not accepted)
            self._time = inputs.time_s
        except Exception as exc:
            self._failed = True
            raise ValueError(f"host firmware {method} failed: {exc}") from exc
        finally:
            self._running = False

    def on_range(self, observation: RangeObservation, inputs: FirmwareInputs) -> None:
        if observation.time_s != inputs.time_s:
            raise ValueError("range observation and input frame must share delivery time")
        self._invoke("on_range", inputs, observation)

    def on_imu(self, observation: IMUObservation, inputs: FirmwareInputs) -> None:
        if observation.time_s != inputs.time_s:
            raise ValueError("IMU observation and input frame must share delivery time")
        if self._has_imu_callback:
            self._invoke("on_imu", inputs, observation)
            self._imu_callback_count += 1

    def tick(self, inputs: FirmwareInputs) -> Literal["drive", "brake"]:
        self._invoke("on_tick", inputs)
        if self._request is not None:
            self._command = "brake"
            self._pwm_duty = 1.0
        elif self._pending_drive is not None:
            drive, self._drive_request_time = self._pending_drive
            self._pwm_duty = drive.pwm_duty
            self._pending_drive = None
            self._drive_commands += 1
        return self._command

    def on_voltage(self, observation: VoltageObservation, inputs: FirmwareInputs) -> None:
        if observation.time_s != inputs.time_s:
            raise ValueError("voltage observation and input frame must share delivery time")
        if self._has_voltage_callback:
            self._invoke("on_voltage", inputs, observation)
            self._voltage_callback_count += 1

    def report(self) -> dict:
        kind = type(self._program)
        return {
            "interface_version": 5 if self._has_voltage_callback else 4,
            **(
                {}
                if not self._has_voltage_callback
                else {
                    "voltage_callback": True,
                    "voltage_callback_count": self._voltage_callback_count,
                }
            ),
            "program": f"{kind.__module__}.{kind.__qualname__}",
            "parameters": self._program.model_dump(mode="json")
            if isinstance(self._program, Schema)
            else None,
            "execution": "host_python",
            "imu_callback": self._has_imu_callback,
            "imu_callback_count": self._imu_callback_count,
            "log_write_attempts": self._log_attempts,
            "log_writes_accepted": self._logs_accepted,
            "log_writes_rejected": self._logs_rejected,
            "policy": "forward_pwm_then_latched_brake",
            "pwm_duty": self._pwm_duty,
            "drive_request_time_s": self._drive_request_time,
            "drive_request_count": self._drive_requests,
            "drive_command_count": self._drive_commands,
            "drive_requests_ignored_after_brake": self._drive_ignored,
            "pending_drive_request": None
            if self._pending_drive is None
            else {
                "pwm_duty": self._pending_drive[0].pwm_duty,
                "time_s": self._pending_drive[1],
            },
            "request": None if self._request is None else self._request.model_dump(),
            "detection_time_s": self._detection_time,
            "command": self._command,
        }
