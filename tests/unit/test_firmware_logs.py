import pytest

from roboforge.hardware.rover_firmware import (
    BrakeRequest,
    FirmwareInputs,
    FirmwareLog,
    FirmwareResponse,
    HostRoverFirmware,
    ObstacleBrakeFirmware,
)


class LoggingProgram(ObstacleBrakeFirmware):
    def on_tick(self, inputs):
        return FirmwareResponse(
            brake_request=BrakeRequest(reason="controller_stop"),
            logs=(FirmwareLog(text="stopping", level="warning"),),
        )


def frame():
    return FirmwareInputs(time_s=0.01, left_encoder_count=10, right_encoder_count=10)


@pytest.mark.parametrize("accepted", [True, False, None])
def test_log_rejection_or_absent_console_does_not_suppress_brake_request(accepted):
    calls = []

    def sink(entry, callback):
        calls.append((entry, callback))
        return accepted

    host = HostRoverFirmware(
        LoggingProgram(brake_threshold_m=0.6), on_log=None if accepted is None else sink
    )
    assert host.tick(frame()) == "brake"
    assert host.request.reason == "controller_stop" and host.detection_time_s == 0.01
    assert len(calls) == (0 if accepted is None else 1)
    if calls:
        assert calls[0][0].text == "stopping" and calls[0][1] == "on_tick"
    report = host.report()
    assert report["log_write_attempts"] == 1
    assert report["log_writes_accepted"] == int(accepted is True)
    assert report["log_writes_rejected"] == int(accepted is not True)


def test_whole_response_is_validated_before_latching_or_sending_any_logs():
    calls = []

    class Invalid(LoggingProgram):
        def on_tick(self, inputs):
            return FirmwareResponse.model_construct(
                brake_request=BrakeRequest(reason="invalid_batch"),
                logs=(FirmwareLog(text="valid"), FirmwareLog.model_construct(text="x" * 4097)),
            )

    host = HostRoverFirmware(
        Invalid(brake_threshold_m=0.6), on_log=lambda *args: calls.append(args)
    )
    with pytest.raises(ValueError, match="host firmware on_tick failed"):
        host.tick(frame())
    assert calls == [] and host.request is None
    assert host.report()["log_write_attempts"] == 0
    with pytest.raises(ValueError, match="failed host firmware"):
        host.tick(frame())


@pytest.mark.parametrize("text", ["", "x" * 4097, "🛞" * 1025, "\ud800", 1, None])
def test_invalid_text_is_rejected_before_it_can_reach_a_console(text):
    with pytest.raises(ValueError):
        FirmwareLog(text=text)


def test_response_size_levels_and_mutability_are_bounded():
    entry = FirmwareLog(text="🛞" * 1024)
    with pytest.raises(ValueError):
        FirmwareResponse(logs=(entry,) * 17)
    with pytest.raises(ValueError):
        FirmwareLog(text="example", level="unknown")
    response = FirmwareResponse(logs=(entry,))
    with pytest.raises(ValueError, match="frozen"):
        response.logs = ()
    with pytest.raises(ValueError, match="frozen"):
        entry.text = "changed"


def test_invalid_log_sink_fails_explicitly_and_empty_response_is_a_noop():
    with pytest.raises(ValueError, match="log sink must be callable"):
        HostRoverFirmware(LoggingProgram(brake_threshold_m=0.6), on_log=42)
    host = HostRoverFirmware(LoggingProgram(brake_threshold_m=0.6), on_log=lambda *args: "yes")
    with pytest.raises(ValueError, match="log sink must return a boolean"):
        host.tick(frame())

    class Empty(LoggingProgram):
        def on_tick(self, inputs):
            return FirmwareResponse()

    host = HostRoverFirmware(Empty(brake_threshold_m=0.6))
    assert host.tick(frame()) == "drive"
    assert host.report()["log_write_attempts"] == 0
