import math

import pytest
from pydantic import ValidationError

from roboforge.actuators import WheelActuators
from roboforge.config import (
    ActuatorConfig,
    EncoderConfig,
    RunConfig,
    WheelActuatorConfig,
    WheelCommand,
)
from roboforge.robotics import WheelSpeeds
from roboforge.simulation import Simulator


def test_ideal_passthrough():
    actuator = WheelActuators()
    for requested in (WheelSpeeds(8, -3), WheelSpeeds(-9, 5), WheelSpeeds(0, 0)):
        assert actuator.step(requested, 0.01, 0).applied == requested


def test_exact_first_order_endpoint():
    config = WheelActuatorConfig(time_constant=0.3)
    for dt in (0.1, 0.01, 0.001):
        actuator = WheelActuators(ActuatorConfig(left=config, right=config))
        for i in range(round(1 / dt)):
            sample = actuator.step(WheelSpeeds(10, -10), dt, i * dt)
        assert sample.applied.left == pytest.approx(10 * (1 - math.exp(-1 / 0.3)))
        assert sample.applied.right == -sample.applied.left


def test_limits_reversal_and_flags():
    config = WheelActuatorConfig(max_speed=3, max_acceleration=2)
    actuator = WheelActuators(ActuatorConfig(left=config, right=config))
    previous = 0
    for i in range(80):
        sample = actuator.step(WheelSpeeds(10 if i < 40 else -10, 0), 0.1, i * 0.1)
        assert abs(sample.applied.left) <= 3
        assert abs(sample.applied.left - previous) <= 0.2 + 1e-14
        assert sample.speed_limited == (True, False)
        previous = sample.applied.left
    assert previous == -3


def test_delay_and_asymmetric_gain_deadzone():
    actuator = WheelActuators(
        ActuatorConfig(
            delay_steps=2,
            left=WheelActuatorConfig(gain=0.5),
            right=WheelActuatorConfig(deadzone=2),
        )
    )
    samples = [actuator.step(WheelSpeeds(4, 2), 0.1, i * 0.1) for i in range(4)]
    assert [s.applied for s in samples] == [WheelSpeeds(0, 0)] * 2 + [WheelSpeeds(2, 0)] * 2


@pytest.mark.parametrize(
    "data",
    [
        {"delay_steps": -1},
        {"delay_steps": 1.5},
        {"delay_steps": True},
        {"left": {"time_constant": -1}},
        {"left": {"max_speed": 0}},
        {"right": {"gain": float("nan")}},
        {"right": {"deadzone": -1}},
    ],
)
def test_invalid_configuration(data):
    with pytest.raises(ValidationError):
        ActuatorConfig.model_validate(data)


def test_simulation_encoders_follow_applied_and_repeat():
    config = RunConfig(
        commands=(WheelCommand(left=10, right=10, steps=100),),
        actuators=ActuatorConfig(
            delay_steps=10,
            left=WheelActuatorConfig(max_speed=2),
            right=WheelActuatorConfig(max_speed=2),
        ),
        sensors=(EncoderConfig(rate_hz=100),),
    )
    result = Simulator(config).run()
    assert result == Simulator(config).run()
    assert result.states[-1].pose.x == pytest.approx(0.05 * 2 * 0.9)
    assert result.readings[-1].left_ticks == round(1.8 / (2 * math.pi) * 2048)
    assert len(result.actuator_samples) == 100


def test_motion_converges_to_continuous_first_order_integral():
    errors = []
    for dt in (0.1, 0.05, 0.025):
        config = RunConfig.model_validate(
            {
                "simulation": {"dt": dt},
                "commands": [{"left": 10, "right": 10, "steps": round(1 / dt)}],
                "actuators": {"left": {"time_constant": 0.3}, "right": {"time_constant": 0.3}},
            }
        )
        exact_distance = 0.5 * (1 - 0.3 * (1 - math.exp(-1 / 0.3)))
        errors.append(abs(Simulator(config).run().states[-1].pose.x - exact_distance))
    assert errors[1] < 0.55 * errors[0]
    assert errors[2] < 0.55 * errors[1]
