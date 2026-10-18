import pytest

from roboforge.hardware.encoder_health import EncoderFeedbackGuard, EncoderHealthConfig
from roboforge.hardware.encoder_speed import EncoderSpeedObservation


def observation(time, deltas=(10, 10), speeds=(0.1, 0.1)):
    return EncoderSpeedObservation(
        time_s=time,
        interval_start_s=time - 0.02,
        interval_s=0.02,
        count_delta=deltas,
        wheel_speed_m_s=speeds,
        mean_speed_m_s=sum(speeds) / 2,
    )


def still(time):
    return observation(time, (0, 0), (0, 0))


@pytest.mark.parametrize(
    "deltas,speeds,code,wheels",
    [
        ((-1, 10), (-0.01, 0.1), "encoder_reverse", ("left",)),
        ((-1, -1), (-0.01, -0.01), "encoder_reverse", ("left", "right")),
        ((10, 1000), (0.1, 3), "encoder_speed_implausible", ("right",)),
    ],
)
def test_immediate_faults_latch_first_observed_evidence_even_below_drive_thresholds(
    deltas, speeds, code, wheels
):
    guard = EncoderFeedbackGuard(EncoderHealthConfig())
    sample = observation(0.02, deltas, speeds)
    fault = guard.observe(sample, 0, 0)
    assert (fault.code, fault.wheels, fault.time_s, fault.since_s) == (code, wheels, 0.02, 0.02)
    assert fault.observation == sample
    assert guard.observe(observation(0.04), 0.2, 0.2) is fault
    assert guard.fault is fault
    with pytest.raises(ValueError):
        fault.code = "encoder_no_progress"
    with pytest.raises(ValueError, match="increasing time"):
        guard.observe(observation(0.04), 0.2, 0.2)


def test_no_progress_timeout_starts_with_first_complete_zero_window_and_checks_both_wheels():
    guard = EncoderFeedbackGuard(EncoderHealthConfig())
    assert guard.observe(still(0.02), 0.05, 0.05) is None
    assert guard.observe(still(0.219), 0.05, 0.05) is None
    fault = guard.observe(still(0.22), 0.05, 0.05)
    assert fault.code == "encoder_no_progress"
    assert fault.wheels == ("left", "right")
    assert fault.since_s == 0.02 and fault.time_s == 0.22


def test_progress_resets_each_wheels_timeout_independently():
    guard = EncoderFeedbackGuard(EncoderHealthConfig(maximum_speed_difference_m_s=1))
    assert guard.observe(observation(0.02, (0, 0), (0, 0)), 0.2, 0.2) is None
    assert guard.observe(observation(0.2, (1, 0), (0.01, 0)), 0.2, 0.2) is None
    fault = guard.observe(observation(0.22, (0, 0), (0, 0)), 0.2, 0.2)
    assert fault.code == "encoder_no_progress" and fault.wheels == ("right",)


@pytest.mark.parametrize("target,duty", [(0.049, 0.2), (0.2, 0.049)])
def test_ineligible_startup_or_low_power_clears_persistent_evidence(target, duty):
    guard = EncoderFeedbackGuard(EncoderHealthConfig())
    assert guard.observe(still(0.02), 0.2, 0.2) is None
    assert guard.observe(still(0.2), target, duty) is None
    assert guard.observe(still(0.22), 0.2, 0.2) is None
    assert guard.observe(still(0.4), 0.2, 0.2) is None
    assert guard.observe(still(0.42), 0.2, 0.2).since_s == 0.22


def test_disagreement_requires_persistence_and_does_not_diagnose_which_encoder_is_wrong():
    guard = EncoderFeedbackGuard(EncoderHealthConfig(maximum_speed_difference_m_s=0.125))

    def mismatch(time):
        return observation(time, (1, 10), (0.01, 0.2))

    assert guard.observe(mismatch(0.02), 0.2, 0.2) is None
    # Equality to the allowed difference clears the previous suspect interval.
    assert guard.observe(observation(0.1, speeds=(0.125, 0.25)), 0.2, 0.2) is None
    assert guard.observe(mismatch(0.12), 0.2, 0.2) is None
    assert guard.observe(mismatch(0.219), 0.2, 0.2) is None
    fault = guard.observe(mismatch(0.22), 0.2, 0.2)
    assert fault.code == "encoder_disagreement"
    assert fault.wheels == ("left", "right") and fault.since_s == 0.12


def test_speed_at_maximum_and_small_quantization_difference_are_allowed():
    guard = EncoderFeedbackGuard(EncoderHealthConfig())
    assert guard.observe(observation(0.02, speeds=(2, 2)), 0.2, 0.2) is None
    for index in range(2, 30):
        assert guard.observe(observation(index * 0.02, speeds=(0.2, 0.202)), 0.2, 0.2) is None


@pytest.mark.parametrize("target,duty", [(-1, 0.2), (0.2, 1.1), (float("inf"), 0.2)])
def test_invalid_arguments_do_not_advance_guard_state(target, duty):
    guard = EncoderFeedbackGuard(EncoderHealthConfig())
    with pytest.raises(ValueError):
        guard.observe(observation(0.02), target, duty)
    assert guard.observe(observation(0.02), 0.2, 0.2) is None


@pytest.mark.parametrize(
    "invalid",
    [
        {"no_progress_timeout_s": 0},
        {"disagreement_timeout_s": -1},
        {"maximum_wheel_speed_m_s": float("inf")},
        {"maximum_speed_difference_m_s": -1},
        {"minimum_check_target_m_s": True},
        {"minimum_check_pwm_duty": 0},
        {"minimum_check_pwm_duty": 1.1},
    ],
)
def test_invalid_thresholds_are_rejected(invalid):
    with pytest.raises(ValueError):
        EncoderHealthConfig(**invalid)
