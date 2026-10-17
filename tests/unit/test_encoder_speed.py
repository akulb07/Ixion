import math

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.encoder_speed import EncoderSpeedConfig, EncoderSpeedEstimator
from roboforge.hardware.rover_firmware import FirmwareInputs
from tests.unit.test_hardware_project import REFERENCE


def frame(time, left, right):
    return FirmwareInputs(time_s=time, left_encoder_count=left, right_encoder_count=right)


def config():
    return EncoderSpeedConfig(counts_per_wheel_revolution=(100, 200), wheel_radius_m=0.1)


def test_window_accumulates_counts_and_uses_each_wheels_resolution():
    estimator = EncoderSpeedEstimator(config())
    assert estimator.update(frame(0, 1000, 2000)) is None
    assert estimator.update(frame(0.01, 1005, 2010)) is None
    sample = estimator.update(frame(0.02, 1010, 2020))
    expected = math.tau * 0.1 * 0.1 / 0.02
    assert sample.wheel_speed_m_s == pytest.approx((expected, expected))
    assert sample.count_delta == (10, 20) and sample.interval_start_s == 0
    assert sample.interval_s == 0.02 and sample.mean_speed_m_s == pytest.approx(expected)
    still = estimator.update(frame(0.04, 1010, 2020))
    assert still.wheel_speed_m_s == (0, 0)


def test_signed_wheel_counts_and_mean_are_not_forced_to_match():
    estimator = EncoderSpeedEstimator(config())
    estimator.update(frame(0, 0, 0))
    sample = estimator.update(frame(0.02, -10, 40))
    assert sample.wheel_speed_m_s[0] < 0 < sample.wheel_speed_m_s[1]
    assert sample.mean_speed_m_s == pytest.approx(sum(sample.wheel_speed_m_s) / 2)


def test_nonincreasing_input_times_are_rejected_even_inside_an_accumulating_window():
    estimator = EncoderSpeedEstimator(config())
    estimator.update(frame(0, 0, 0))
    estimator.update(frame(0.01, 1, 1))
    for time in (0, 0.005, 0.01):
        with pytest.raises(ValueError, match="strictly increasing"):
            estimator.update(frame(time, 99, 99))
    assert estimator.update(frame(0.02, 10, 20)).count_delta == (10, 20)


def test_failed_numeric_conversion_leaves_anchor_and_time_unchanged():
    estimator = EncoderSpeedEstimator(config())
    estimator.update(frame(0, 0, 0))
    with pytest.raises(ValueError, match="numeric range"):
        estimator.update(frame(0.02, 10**1000, 0))
    assert estimator.update(frame(0.02, 10, 20)).count_delta == (10, 20)


def test_exact_integer_tick_windows_are_not_skipped_due_to_rounding():
    estimator = EncoderSpeedEstimator(config())
    estimator.update(frame(0, 0, 0))
    for index in range(1, 101):
        assert estimator.update(frame(index * 0.02, index, index)) is not None


def test_resolution_comes_from_rotor_encoder_gearing_and_edge_multiplier():
    project = load_project(REFERENCE)
    resolved = EncoderSpeedConfig.from_project(project, "left_motor", "right_motor")
    assert resolved.counts_per_wheel_revolution == (4800, 4800)
    assert resolved.wheel_radius_m == project.assembly.wheel_radius_m
    with pytest.raises(ValueError, match="distinct"):
        EncoderSpeedConfig.from_project(project, "left_motor", "left_motor")


@pytest.mark.parametrize(
    "invalid",
    [
        {"minimum_interval_s": 0},
        {"wheel_radius_m": True},
        {"counts_per_wheel_revolution": (100, 0)},
    ],
)
def test_invalid_units_and_resolution_are_rejected(invalid):
    with pytest.raises(ValueError):
        EncoderSpeedConfig.model_validate({**config().model_dump(), **invalid})
