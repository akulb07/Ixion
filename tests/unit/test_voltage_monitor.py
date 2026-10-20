from copy import deepcopy

import pytest

from roboforge.hardware.signal_clock import SignalClock
from roboforge.hardware.voltage_monitor import (
    VoltageMonitor,
    VoltageMonitorConfig,
    project_voltage_monitor,
    quantize_voltage,
)


def test_quantization_uses_nearest_count_and_keeps_overrange_distinct():
    config = VoltageMonitorConfig(full_scale_v=10, resolution_bits=4)
    assert quantize_voltage(0, config) == (0, 0, False)
    assert quantize_voltage(5, config) == (8, 80 / 15, False)
    assert quantize_voltage(10, config) == (15, 10, False)
    assert quantize_voltage(12, config) == (15, 10, True)
    for voltage in (0.1, 2.35, 7.71, 9.99):
        assert abs(quantize_voltage(voltage, config)[1] - voltage) <= 10 / 15 / 2


def test_sample_is_held_until_completion_and_busy_poll_does_not_replace_it():
    clock = SignalClock({"test": False})
    readings = []
    monitor = VoltageMonitor(clock, VoltageMonitorConfig(conversion_s=0.01), readings.append)
    assert monitor.poll(7)
    clock.run_until(0.005)
    assert not monitor.poll(9)
    assert not readings
    assert monitor.report()["pending"]["source_voltage_v"] == 7
    clock.run_until(0.01)
    assert len(readings) == 1
    assert readings[0].sample_time_s == 0
    assert readings[0].time_s == 0.01
    assert readings[0].voltage_v == quantize_voltage(7, VoltageMonitorConfig())[1]
    assert "source_voltage_v" not in readings[0].model_dump()
    assert monitor.report()["pending"] is None
    assert monitor.report()["busy_skips"] == 1


@pytest.mark.parametrize(
    "field,value",
    [
        ("resolution_bits", True),
        ("resolution_bits", 1),
        ("resolution_bits", 25),
        ("full_scale_v", 0),
        ("conversion_s", 0),
        ("conversion_s", float("nan")),
        ("poll_every_steps", 0),
    ],
)
def test_invalid_configuration_is_rejected(field, value):
    with pytest.raises(ValueError):
        VoltageMonitorConfig(**{field: value})


def test_report_validation_retains_pending_evidence_and_rejects_invented_delivery():
    clock = SignalClock({"test": False})
    config = VoltageMonitorConfig(conversion_s=0.01)
    monitor = VoltageMonitor(clock, config, lambda reading: None)
    monitor.poll(7)
    clock.run_until(0.005)
    data = monitor.report()
    samples = [{"time_s": 0, "pack_voltage_v": 7}, {"time_s": 0.005, "pack_voltage_v": 8}]
    assert project_voltage_monitor(data, config.model_dump(), samples) == data
    edited = deepcopy(data)
    edited["readings"] = [edited["pending"]]
    edited["pending"] = None
    with pytest.raises(ValueError, match="beyond run end"):
        project_voltage_monitor(edited, config.model_dump(), samples)


def test_finite_values_and_clock_precision_are_required():
    config = VoltageMonitorConfig()
    for value in (-1, True, float("inf")):
        with pytest.raises(ValueError):
            quantize_voltage(value, config)
    clock = SignalClock({"test": False})
    clock.run_until(1e20)
    monitor = VoltageMonitor(clock, config, lambda reading: None)
    with pytest.raises(ValueError, match="finish after"):
        monitor.poll(7)
    assert monitor.report()["pending"] is None
