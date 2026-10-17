import pytest

from roboforge.hardware.encoder_speed import EncoderSpeedConfig
from roboforge.hardware.rover_firmware import FirmwareInputs, RangeObservation
from roboforge.hardware.speed_firmware import EncoderSpeedFirmware


def program(**kwargs):
    return EncoderSpeedFirmware(
        encoder=EncoderSpeedConfig(counts_per_wheel_revolution=(1000, 1000), wheel_radius_m=0.1),
        brake_threshold_m=0.6,
        log_every_updates=None,
        **kwargs,
    )


def frame(time, count=0):
    return FirmwareInputs(time_s=time, left_encoder_count=count, right_encoder_count=count)


def test_encoder_feedback_changes_power_request_under_the_same_target():
    stopped = program(initial_pwm_duty=0.25, target_ramp_s=0.01)
    moving = program(initial_pwm_duty=0.25, target_ramp_s=0.01)
    stopped.on_tick(frame(0))
    moving.on_tick(frame(0))
    low = stopped.on_tick(frame(0.02)).drive_request.pwm_duty
    high = moving.on_tick(frame(0.02, 20)).drive_request.pwm_duty
    assert low > 0.25 > high
    assert abs(low - 0.25) <= 0.5 * 0.02 + 1e-12
    assert abs(high - 0.25) <= 0.5 * 0.02 + 1e-12


def test_saturation_blocks_integral_windup_and_output_stays_bounded():
    controller = program(
        target_speed_m_s=5, target_ramp_s=0.01, initial_pwm_duty=0.1, maximum_pwm_duty=0.1
    )
    controller.on_tick(frame(0))
    for index in range(1, 51):
        result = controller.on_tick(frame(index * 0.02))
        assert result.drive_request.pwm_duty == 0.1
    trace = controller.report()["samples"]
    assert all(sample["integral_error_m"] == 0 for sample in trace)
    assert all(sample["limited"] for sample in trace[1:])


def test_sonar_brake_stops_pi_updates_but_keeps_observing_encoder_speed():
    controller = program()
    controller.on_tick(frame(0))
    controller.on_tick(frame(0.02, 1))
    integral = controller.report()["samples"][-1]["integral_error_m"]
    request = controller.on_range(RangeObservation(time_s=0.025, status="timeout"), frame(0.025, 1))
    assert request.reason == "echo_timeout"
    assert controller.on_tick(frame(0.04, 2)) is None
    last = controller.report()["samples"][-1]
    assert last["braking"] and last["observation"] is not None
    assert last["requested_pwm_duty"] is None and last["integral_error_m"] == integral


def test_partial_window_holds_output_and_fresh_instances_keep_separate_state():
    first, second = program(), program()
    parameters = first.model_dump()
    assert first.on_tick(frame(0)).pwm_duty == first.initial_pwm_duty
    assert first.on_tick(frame(0.01, 2)) is None
    assert len(first.report()["samples"]) == 1
    assert second.report()["samples"] == []
    assert first.model_dump() == parameters
    assert first.report()["metrics"]["post_ramp_tracking_rmse_m_s"] is None
    copied = first.report()
    copied["samples"].clear()
    assert len(first.report()["samples"]) == 1


@pytest.mark.parametrize(
    "invalid",
    [
        {"initial_pwm_duty": 0},
        {"initial_pwm_duty": 0.9},
        {"target_speed_m_s": float("nan")},
        {"target_ramp_s": 0},
        {"maximum_duty_rate_per_s": 0},
        {"integral_gain_per_m": -1},
    ],
)
def test_invalid_controller_parameters_are_rejected(invalid):
    with pytest.raises(ValueError):
        program(**invalid)
