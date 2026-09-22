import math

import pytest

from roboforge.config import FaultConfig, RunConfig
from roboforge.io import save_result
from roboforge.replay import ReplayLog
from roboforge.sensors.readings import EncoderReading, ImuReading
from roboforge.simulation import Simulator


def config(faults):
    return RunConfig.model_validate(
        {
            "robot": {"initial_pose": {"x": 2, "y": 2}},
            "simulation": {"dt": 0.1},
            "commands": [{"left": 4, "right": 4, "steps": 20}],
            "sensors": [{"type": "encoder", "rate_hz": 10}, {"type": "imu", "rate_hz": 10}],
            "faults": faults,
        }
    )


def test_slip_separates_shaft_encoders_and_ground_motion_and_replays(tmp_path):
    cfg = config(
        [
            {
                "name": "loss",
                "kind": "wheel_slip",
                "target": "both",
                "start": 0.5,
                "end": 1.5,
                "magnitude": 0.5,
            }
        ]
    )
    result = Simulator(cfg).run()
    assert result.states[-1].pose.x == pytest.approx(2.3)
    encoders = [r for r in result.readings if isinstance(r, EncoderReading)]
    assert encoders[-1].left_ticks == round(8 / math.tau * 2048)
    assert result.states[10].wheels.left == 4
    assert result.states[10].twist.linear == pytest.approx(0.1)
    assert [e.time for e in result.fault_events] == [0.5, 1.5]
    save_result(result, tmp_path)
    replay = ReplayLog(tmp_path)
    assert replay.at(1).state == result.states[10]
    assert replay.at(1.05).state.wheels.left == 4
    assert replay.at(1.05).state.twist.linear == pytest.approx(0.1)
    assert result == Simulator(cfg).run()


def test_sensor_fault_boundaries_combination_and_immutable_contracts():
    cfg = config(
        [
            {
                "name": "drop",
                "kind": "sensor_dropout",
                "target": "encoders",
                "start": 0.5,
                "end": 1,
                "magnitude": 1,
            },
            {
                "name": "bias",
                "kind": "gyro_bias",
                "target": "imu",
                "start": 0.5,
                "end": 1,
                "magnitude": 0.2,
            },
            {
                "name": "drift",
                "kind": "gyro_drift",
                "target": "imu",
                "start": 0,
                "end": 2,
                "magnitude": 0.1,
            },
        ]
    )
    result = Simulator(cfg).run()
    for reading in result.readings:
        if isinstance(reading, EncoderReading):
            assert (reading.left_ticks is None) == (0.5 <= reading.capture_time < 1)
        if isinstance(reading, ImuReading):
            expected = (reading.capture_time * 0.1 if reading.capture_time < 2 else 0) + (
                0.2 if 0.5 <= reading.capture_time < 1 else 0
            )
            assert reading.gyro_z == pytest.approx(expected)


def test_fault_random_streams_independent_of_configuration_order():
    faults = [
        {"name": "a", "kind": "sensor_dropout", "target": "encoders", "magnitude": 0.4},
        {"name": "b", "kind": "sensor_dropout", "target": "imu", "magnitude": 0.3},
    ]
    assert (
        Simulator(config(faults)).run().readings == Simulator(config(faults[::-1])).run().readings
    )


def test_delay_and_saturation():
    cfg = config(
        [
            {"name": "delay", "kind": "actuator_delay", "target": "both", "delay_steps": 2},
            {"name": "limit", "kind": "actuator_saturation", "target": "both", "magnitude": 2},
        ]
    )
    result = Simulator(cfg).run()
    assert result.states[-1].pose.x == pytest.approx(2.18)
    assert all(s.applied.left == 0 for s in result.actuator_samples[:2])
    assert result.actuator_samples[-1].speed_limited == (True, True)


@pytest.mark.parametrize(
    "fault",
    [
        {"name": "bad", "kind": "sensor_dropout", "target": "missing", "magnitude": 0.5},
        {"name": "bad", "kind": "encoder_scale", "target": "imu", "magnitude": 0.5},
        {"name": "bad", "kind": "wheel_slip", "target": "left", "magnitude": 2},
        {"name": "bad", "kind": "actuator_delay", "target": "left", "delay_steps": 1},
    ],
)
def test_invalid_fault_targets_and_magnitudes(fault):
    with pytest.raises(ValueError):
        config([fault])


def test_encoder_scale_is_explicit_readout_error():
    cfg = config(
        [{"name": "scale", "kind": "encoder_scale", "target": "encoders", "magnitude": 0.1}]
    )
    result = Simulator(cfg).run()
    baseline = Simulator(config([])).run()
    assert result.states == baseline.states
    a = [r for r in result.readings if isinstance(r, EncoderReading)][-1]
    b = [r for r in baseline.readings if isinstance(r, EncoderReading)][-1]
    assert a.left_ticks == round(b.left_ticks * 1.1)


def test_fault_timing_validation():
    with pytest.raises(ValueError):
        FaultConfig(name="bad", kind="gyro_bias", target="imu", start=2, end=1)
