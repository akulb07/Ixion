import pytest
from pydantic import ValidationError

from roboforge.config import PIDConfig, RunConfig, WheelControllerConfig
from roboforge.control import PID, EncoderWheelController
from roboforge.robotics import WheelSpeeds
from roboforge.sensors.readings import EncoderReading
from roboforge.simulation import Simulator


def encoder(time, ticks, delivery=None):
    return EncoderReading(
        sensor="encoders",
        frame="base",
        sequence=round(time * 100),
        capture_time=time,
        delivery_time=time if delivery is None else delivery,
        ticks_per_revolution=1000,
        left_ticks=ticks,
        right_ticks=ticks,
    )


def test_pid_terms_and_reset():
    pid = PID(PIDConfig(kp=2, ki=3, kd=0.5, derivative_time_constant=0))
    first = pid.step(2, 1, 0.1)
    assert first.proportional == 2
    assert first.integral == pytest.approx(0.3)
    assert first.derivative == 0
    second = pid.step(2, 1.2, 0.1)
    assert second.derivative == pytest.approx(-1)
    assert second.integral == pytest.approx(0.54)
    pid.reset()
    assert pid.step(2, 1, 0.1) == first


def test_no_derivative_setpoint_kick_and_filter():
    pid = PID(PIDConfig(kp=0, kd=1, derivative_time_constant=0.1))
    pid.step(0, 0, 0.1)
    assert pid.step(10, 0, 0.1).derivative == 0
    assert pid.step(10, 1, 0.1).derivative == pytest.approx(-5)
    assert pid.step(10, 1, 0.1).derivative == pytest.approx(-2.5)


def test_antiwindup_and_recovery():
    pid = PID(PIDConfig(kp=2, ki=10, output_min=-1, output_max=1))
    for _ in range(1000):
        assert pid.step(10, 0, 0.01).output == 1
    assert pid.integral == 0
    assert pid.step(0, 0, 0.01).output == 0
    assert pid.step(-10, 0, 0.01).output == -1


def test_integral_bound():
    pid = PID(PIDConfig(kp=0, ki=1, integral_limit=0.5))
    for _ in range(20):
        result = pid.step(1, 0, 0.1)
    assert result.integral == 0.5


@pytest.mark.parametrize("dt", [0, -1, float("nan"), True])
def test_invalid_dt_leaves_state(dt):
    pid = PID()
    with pytest.raises(ValueError):
        pid.step(1, 0, dt)
    assert pid.integral == 0


def test_encoder_latency_dropout_and_rate_interval():
    controller = EncoderWheelController(WheelControllerConfig())
    assert controller.update(0, WheelSpeeds(2, 2), (encoder(0, 0),)) == WheelSpeeds(0, 0)
    controller.update(0.1, WheelSpeeds(2, 2), (encoder(0.1, None),))
    controller.update(0.3, WheelSpeeds(2, 2), (encoder(0.2, 100, 0.3),))
    sample = controller.samples[-1]
    assert sample.measurement_dt == 0.2
    assert sample.left.measurement == pytest.approx(3.141592653589793)
    assert sample.capture_time == 0.2
    with pytest.raises(ValueError, match="undelivered"):
        controller.update(0.3, WheelSpeeds(0, 0), (encoder(0.3, 200, 0.4),))


def feedback_config():
    return RunConfig.model_validate(
        {
            "commands": [{"left": 6, "right": 6, "steps": 800}],
            "sensors": [{"type": "encoder", "rate_hz": 50, "ticks_per_revolution": 65536}],
            "actuators": {
                "left": {"gain": 0.7, "time_constant": 0.15},
                "right": {"gain": 0.8, "time_constant": 0.2},
            },
            "wheel_controller": {"left": {"kp": 0.6, "ki": 3}, "right": {"kp": 0.6, "ki": 3}},
        }
    )


def test_closed_loop_corrects_gain_error_from_encoders():
    config = feedback_config()
    result = Simulator(config).run()
    open_result = Simulator(config.model_copy(update={"wheel_controller": None})).run()
    assert result == Simulator(config).run()
    for side in ("left", "right"):
        error = abs(getattr(result.states[-1].wheels, side) - 6)
        assert error < 0.01
        assert error < 0.01 * abs(getattr(open_result.states[-1].wheels, side) - 6)
    assert all(s.capture_time <= s.time for s in result.control_samples)
    assert result.actuator_samples[0].applied == WheelSpeeds(0, 0)


def test_controller_requires_encoder():
    with pytest.raises(ValidationError, match="requires its named encoder"):
        RunConfig.model_validate(
            {"commands": [{"left": 0, "right": 0, "steps": 1}], "wheel_controller": {}}
        )
