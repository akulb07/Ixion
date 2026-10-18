"""Configurable feedback plausibility checks for equal forward wheel commands."""

import math
from typing import Literal

from pydantic import Field

from roboforge.config import Nonnegative, Positive, Probability, Schema
from roboforge.core import nonnegative

from .encoder_speed import EncoderSpeedObservation

FeedbackFaultCode = Literal[
    "encoder_reverse", "encoder_speed_implausible", "encoder_no_progress", "encoder_disagreement"
]


class EncoderHealthConfig(Schema):
    maximum_wheel_speed_m_s: Positive = 2
    maximum_speed_difference_m_s: Nonnegative = 0.05
    no_progress_timeout_s: Positive = 0.2
    disagreement_timeout_s: Positive = 0.1
    minimum_check_target_m_s: Positive = 0.05
    minimum_check_pwm_duty: Probability = Field(gt=0, default=0.05)


class EncoderFeedbackFault(Schema):
    time_s: Nonnegative
    since_s: Nonnegative
    code: FeedbackFaultCode
    wheels: tuple[Literal["left", "right"], ...] = Field(min_length=1, max_length=2)
    observation: EncoderSpeedObservation
    target_speed_m_s: Nonnegative
    requested_pwm_duty: Probability


class EncoderFeedbackGuard:
    """Trips once from observed evidence; it cannot diagnose the physical cause."""

    def __init__(self, config: EncoderHealthConfig):
        self.config = config
        self._last_time = None
        self._zero_since = {"left": None, "right": None}
        self._difference_since = None
        self._fault = None

    @property
    def fault(self) -> EncoderFeedbackFault | None:
        return self._fault

    @staticmethod
    def _expired(time, since, duration):
        rounding = 4 * max(math.ulp(time), math.ulp(since))
        return time - since >= duration - rounding

    def observe(
        self, sample: EncoderSpeedObservation, target_speed_m_s: float, requested_pwm_duty: float
    ) -> EncoderFeedbackFault | None:
        target = nonnegative(target_speed_m_s, "feedback target")
        duty = nonnegative(requested_pwm_duty, "feedback duty")
        if duty > 1:
            raise ValueError("feedback duty cannot exceed one")
        if self._last_time is not None and sample.time_s <= self._last_time:
            raise ValueError("encoder health observations require increasing time")
        self._last_time = sample.time_s
        if self._fault is not None:
            return self._fault
        names = ("left", "right")

        def trip(code, wheels, since):
            self._fault = EncoderFeedbackFault(
                time_s=sample.time_s,
                since_s=since,
                code=code,
                wheels=wheels,
                observation=sample,
                target_speed_m_s=target,
                requested_pwm_duty=duty,
            )
            return self._fault

        backwards = tuple(name for name, delta in zip(names, sample.count_delta) if delta < 0)
        if backwards:
            return trip("encoder_reverse", backwards, sample.time_s)
        excessive = tuple(
            name
            for name, speed in zip(names, sample.wheel_speed_m_s)
            if abs(speed) > self.config.maximum_wheel_speed_m_s
        )
        if excessive:
            return trip("encoder_speed_implausible", excessive, sample.time_s)
        eligible = (
            target >= self.config.minimum_check_target_m_s
            and duty >= self.config.minimum_check_pwm_duty
        )
        for name, delta in zip(names, sample.count_delta):
            if eligible and delta == 0:
                if self._zero_since[name] is None:
                    self._zero_since[name] = sample.time_s
            else:
                self._zero_since[name] = None
        stopped = tuple(
            name
            for name in names
            if self._zero_since[name] is not None
            and self._expired(
                sample.time_s, self._zero_since[name], self.config.no_progress_timeout_s
            )
        )
        if stopped:
            return trip(
                "encoder_no_progress", stopped, min(self._zero_since[name] for name in stopped)
            )
        mismatch = abs(sample.wheel_speed_m_s[0] - sample.wheel_speed_m_s[1])
        if eligible and mismatch > self.config.maximum_speed_difference_m_s:
            if self._difference_since is None:
                self._difference_since = sample.time_s
        else:
            self._difference_since = None
        if self._difference_since is not None and self._expired(
            sample.time_s, self._difference_since, self.config.disagreement_timeout_s
        ):
            return trip("encoder_disagreement", names, self._difference_since)
        return None
