import pytest

from roboforge.hardware.rover_firmware import (
    BrakeRequest,
    DriveRequest,
    FirmwareInputs,
    FirmwareLog,
    FirmwareResponse,
    HostRoverFirmware,
    RangeObservation,
)


class Program:
    def on_tick(self, inputs):
        return None

    def on_range(self, observation, inputs):
        return DriveRequest(pwm_duty=observation.distance_m)


def frame(time):
    return FirmwareInputs(time_s=time, left_encoder_count=0, right_encoder_count=0)


def observe(host, time, duty):
    host.on_range(RangeObservation(time_s=time, status="echo", distance_m=duty), frame(time))


def test_latest_sensor_request_waits_for_tick_and_then_holds_its_duty():
    host = HostRoverFirmware(Program())
    assert host.tick(frame(0)) == "drive" and host.pwm_duty == 1
    observe(host, 0.01, 0.4)
    observe(host, 0.012, 0.6)
    assert host.pwm_duty == 1
    assert host.report()["pending_drive_request"] == {"pwm_duty": 0.6, "time_s": 0.012}
    assert host.tick(frame(0.02)) == "drive" and host.pwm_duty == 0.6
    assert host.drive_request_time_s == 0.012
    assert host.report()["pending_drive_request"] is None
    host.tick(frame(0.03))
    assert host.pwm_duty == 0.6 and host.report()["drive_command_count"] == 1


def test_tick_request_is_latest_and_can_apply_at_startup():
    class TickDrive(Program):
        def on_tick(self, inputs):
            return DriveRequest(pwm_duty=0.25)

    host = HostRoverFirmware(TickDrive())
    host.tick(frame(0))
    assert host.pwm_duty == 0.25 and host.drive_request_time_s == 0
    observe(host, 0.01, 0.75)
    host.tick(frame(0.02))
    assert host.pwm_duty == 0.25
    assert host.report()["drive_request_count"] == 3
    assert host.report()["drive_command_count"] == 2


def test_latched_brake_cancels_pending_duty_and_ignores_later_drive_requests():
    class BrakeAtTick(Program):
        def on_tick(self, inputs):
            return BrakeRequest(reason="stop")

    host = HostRoverFirmware(BrakeAtTick())
    observe(host, 0.01, 0.5)
    assert host.tick(frame(0.02)) == "brake" and host.pwm_duty == 1
    assert host.report()["pending_drive_request"] is None
    observe(host, 0.03, 0.7)
    assert host.tick(frame(0.04)) == "brake"
    assert host.request.reason == "stop" and host.detection_time_s == 0.02
    assert host.report()["drive_requests_ignored_after_brake"] == 1
    assert host.report()["drive_command_count"] == 0


@pytest.mark.parametrize("duty", [0, -0.1, 1.1, float("nan"), float("inf"), True, "0.5"])
def test_forward_duty_must_be_finite_positive_and_at_most_one(duty):
    with pytest.raises(ValueError):
        DriveRequest(pwm_duty=duty)


def test_ambiguous_or_invalid_response_fails_before_any_command_or_log_is_applied():
    with pytest.raises(ValueError, match="not both"):
        FirmwareResponse(
            brake_request=BrakeRequest(reason="stop"), drive_request=DriveRequest(pwm_duty=0.5)
        )

    class Invalid(Program):
        def on_tick(self, inputs):
            return FirmwareResponse.model_construct(
                drive_request=DriveRequest.model_construct(pwm_duty=2),
                logs=(FirmwareLog(text="must not be emitted"),),
            )

    calls = []
    host = HostRoverFirmware(Invalid(), on_log=lambda *args: calls.append(args))
    with pytest.raises(ValueError, match="host firmware on_tick failed"):
        host.tick(frame(0))
    assert host.pwm_duty == 1 and host.report()["pending_drive_request"] is None
    assert host.report()["drive_request_count"] == 0 and calls == []


def test_drive_response_can_include_logs_without_turning_rejection_into_a_stop():
    class Logged(Program):
        def on_tick(self, inputs):
            return FirmwareResponse(
                drive_request=DriveRequest(pwm_duty=0.3), logs=(FirmwareLog(text="soft start"),)
            )

    host = HostRoverFirmware(Logged(), on_log=lambda *args: False)
    assert host.tick(frame(0)) == "drive" and host.pwm_duty == 0.3
    assert host.request is None and host.report()["log_writes_rejected"] == 1
