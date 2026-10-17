"""Reference PI speed controller, with count feedback and latched sonar braking."""

import math

from pydantic import Field, PrivateAttr, model_validator

from roboforge.config import Nonnegative, Positive, Probability, Real, Schema
from roboforge.core import finite

from .encoder_speed import EncoderSpeedConfig, EncoderSpeedEstimator, EncoderSpeedObservation
from .rover_firmware import (
    DriveRequest,
    FirmwareInputs,
    FirmwareLog,
    FirmwareResponse,
    ObstacleBrakeFirmware,
)


class SpeedControlSample(Schema):
    time_s: Nonnegative
    target_speed_m_s: Nonnegative
    observation: EncoderSpeedObservation | None = None
    requested_pwm_duty: Probability | None = None
    integral_error_m: Real = 0
    limited: bool = False
    braking: bool = False


class EncoderSpeedFirmware(ObstacleBrakeFirmware):
    encoder: EncoderSpeedConfig
    target_speed_m_s: Positive = 0.2
    target_ramp_s: Positive = 1
    proportional_gain_s_m: Nonnegative = 1
    integral_gain_per_m: Nonnegative = 1
    feedforward_s_m: Nonnegative = 0.7
    minimum_pwm_duty: Probability = Field(gt=0, default=0.02)
    maximum_pwm_duty: Probability = Field(gt=0, default=0.8)
    initial_pwm_duty: Probability = Field(gt=0, default=0.02)
    maximum_duty_rate_per_s: Positive = 0.5
    log_every_updates: int | None = Field(strict=True, gt=0, le=10_000, default=10)

    _estimator: EncoderSpeedEstimator | None = PrivateAttr(default=None)
    _integral: float = PrivateAttr(default=0)
    _duty: float = PrivateAttr(default=0)
    _braking: bool = PrivateAttr(default=False)
    _trace: list[SpeedControlSample] = PrivateAttr(default_factory=list)

    @model_validator(mode="after")
    def check_bounds(self):
        if not self.minimum_pwm_duty <= self.initial_pwm_duty <= self.maximum_pwm_duty:
            raise ValueError("initial PWM must be between minimum and maximum duty")
        return self

    def on_range(self, observation, inputs):
        request = super().on_range(observation, inputs)
        if request is not None:
            self._braking = True
        return request

    def on_tick(self, inputs: FirmwareInputs):
        if len(self._trace) >= 10_000:
            raise ValueError("speed controller exceeds 10,000-observation budget")
        target = finite(
            self.target_speed_m_s * min(inputs.time_s / self.target_ramp_s, 1), "speed target"
        )
        if self._estimator is None:
            self._estimator = EncoderSpeedEstimator(self.encoder)
            self._estimator.update(inputs)
            self._duty = self.initial_pwm_duty
            self._trace.append(
                SpeedControlSample(
                    time_s=inputs.time_s,
                    target_speed_m_s=target,
                    requested_pwm_duty=None if self._braking else self._duty,
                    braking=self._braking,
                )
            )
            return None if self._braking else DriveRequest(pwm_duty=self._duty)
        observed = self._estimator.update(inputs)
        if observed is None:
            return None
        limited = False
        if not self._braking:
            error = finite(target - observed.mean_speed_m_s, "speed error")
            integral = finite(self._integral + error * observed.interval_s, "integral error")
            base = finite(
                self.feedforward_s_m * target + self.proportional_gain_s_m * error,
                "speed control base",
            )
            raw = finite(base + self.integral_gain_per_m * integral, "speed control duty")
            change = finite(self.maximum_duty_rate_per_s * observed.interval_s, "duty rate limit")
            lower = max(self.minimum_pwm_duty, self._duty - change)
            upper = min(self.maximum_pwm_duty, self._duty + change)
            limited = not lower <= raw <= upper
            if (raw > upper and error > 0) or (raw < lower and error < 0):
                # Don't integrate further into an amplitude or slew-rate limit.
                raw = finite(base + self.integral_gain_per_m * self._integral, "limited duty")
            else:
                self._integral = integral
            self._duty = min(upper, max(lower, raw))
        self._trace.append(
            SpeedControlSample(
                time_s=inputs.time_s,
                target_speed_m_s=target,
                observation=observed,
                requested_pwm_duty=None if self._braking else self._duty,
                integral_error_m=self._integral,
                limited=limited,
                braking=self._braking,
            )
        )
        if self._braking:
            return None
        logs = ()
        if (
            self.log_every_updates is not None
            and (len(self._trace) - 1) % self.log_every_updates == 0
        ):
            logs = (
                FirmwareLog(
                    text=f"Speed target {target:.4f} m/s, measured {observed.mean_speed_m_s:.4f} m/s, PWM {self._duty:.4f}"
                ),
            )
        return FirmwareResponse(drive_request=DriveRequest(pwm_duty=self._duty), logs=logs)

    def report(self) -> dict:
        tracking = [
            sample
            for sample in self._trace
            if sample.observation is not None
            and not sample.braking
            and sample.time_s >= self.target_ramp_s
        ]
        return {
            "schema_version": 1,
            "config": self.model_dump(mode="json"),
            "samples": [sample.model_dump(mode="json") for sample in self._trace],
            "metrics": {
                "post_ramp_observation_count": len(tracking),
                "post_ramp_tracking_rmse_m_s": None
                if not tracking
                else math.hypot(
                    *(
                        (sample.observation.mean_speed_m_s - sample.target_speed_m_s)
                        / math.sqrt(len(tracking))
                        for sample in tracking
                    )
                ),
            },
            "limitations": [
                "Wheel speed is a count-window average; interpreting it as ground speed assumes no slip",
                "Fixed target ramp and illustrative PI/feedforward gains; no automatic tuning",
                "Positive PWM bounds can cause creep; no zero-speed hold, reverse, turning or restart",
                "Duty limits and anti-windup don't guarantee stability or prevent regeneration on other models",
                "Fresh controller instance required for each experiment",
            ],
        }
