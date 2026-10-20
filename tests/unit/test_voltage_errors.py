from copy import deepcopy

import pytest

from roboforge.hardware.signal_clock import SignalClock
from roboforge.hardware.voltage_monitor import (
    VoltageErrorConfig,
    VoltageMonitor,
    VoltageMonitorConfig,
    project_voltage_monitor,
)


def capture_sequence(errors, *, busy_poll=False):
    clock = SignalClock({"test": False})
    config = VoltageMonitorConfig(errors=errors)
    monitor = VoltageMonitor(clock, config, lambda observation: None)
    samples = []
    for index in range(4):
        clock.run_until(index * 0.01)
        monitor.poll(7.4)
        samples.append({"time_s": clock.time_s, "pack_voltage_v": 7.4})
        if busy_poll:
            clock.run_until(clock.time_s + 0.001)
            assert not monitor.poll(100)
        clock.run_until(index * 0.01 + 0.003)
    samples.append({"time_s": clock.time_s, "pack_voltage_v": 7.4})
    return monitor.report(), config.model_dump(mode="json"), samples


@pytest.mark.parametrize(
    "source, gain, offset, unclipped, count, over_range",
    [(5, 1.1, -0.2, 5.3, 8, False), (1, 0.5, -2, -1.5, 0, False), (20, 1.2, 0, 24, 15, True)],
)
def test_affine_error_precedes_clipping_and_quantization(
    source, gain, offset, unclipped, count, over_range
):
    clock = SignalClock({"test": False})
    config = VoltageMonitorConfig(
        resolution_bits=4, errors=VoltageErrorConfig(gain=gain, offset_v=offset)
    )
    delivered = []
    monitor = VoltageMonitor(clock, config, delivered.append)
    monitor.poll(source)
    clock.run_until(0.003)
    capture = monitor.report()["readings"][0]
    assert capture["source_voltage_v"] == source
    assert capture["unclipped_voltage_v"] == pytest.approx(unclipped)
    assert capture["noise_v"] == 0
    assert delivered[0].raw_count == count
    assert delivered[0].over_range is over_range
    assert delivered[0].voltage_v == count * 10 / 15
    assert (
        not {"noise_v", "source_voltage_v", "unclipped_voltage_v"}
        & delivered[0].model_dump().keys()
    )
    project_voltage_monitor(
        monitor.report(),
        config.model_dump(),
        [{"time_s": 0, "pack_voltage_v": source}, {"time_s": 0.003, "pack_voltage_v": source}],
    )


def test_noise_is_bounded_reproducible_and_not_advanced_by_busy_polls():
    errors = VoltageErrorConfig(noise_bound_v=0.05, seed=7)
    plain, config, samples = capture_sequence(errors)
    busy, _, _ = capture_sequence(errors, busy_poll=True)
    assert plain["readings"] == busy["readings"]
    assert busy["busy_skips"] == 4
    noises = [c["noise_v"] for c in plain["readings"]]
    assert noises == [
        0.046631727035998674,
        -0.002952930628169148,
        -0.011194771340362432,
        0.032694208430757356,
    ]
    assert all(-0.05 <= noise <= 0.05 for noise in noises)
    assert len(set(noises)) == 4
    assert capture_sequence(errors)[0] == plain
    changed, _, _ = capture_sequence(errors.model_copy(update={"seed": 8}))
    assert changed["readings"] != plain["readings"]
    assert project_voltage_monitor(plain, config, samples) == plain


@pytest.mark.parametrize("field", ["noise_v", "unclipped_voltage_v", "seed", "model", "missing"])
def test_error_evidence_is_recomputed_and_edits_are_rejected(field):
    raw, config, samples = capture_sequence(VoltageErrorConfig(noise_bound_v=0.1, seed=11))
    data = deepcopy(raw)
    if field == "seed":
        data["config"]["errors"]["seed"] += 1
    elif field == "model":
        data["model"] = "ideal_pack_voltage_monitor"
    elif field == "missing":
        del data["readings"][0]["noise_v"]
    else:
        data["readings"][0][field] += 0.001
    with pytest.raises(ValueError):
        project_voltage_monitor(data, config, samples)


@pytest.mark.parametrize(
    "field,value",
    [
        ("gain", 0),
        ("gain", 11),
        ("offset_v", float("inf")),
        ("noise_bound_v", -0.1),
        ("seed", True),
        ("seed", -1),
        ("seed", 2**32),
    ],
)
def test_invalid_error_settings_are_rejected(field, value):
    with pytest.raises(ValueError):
        VoltageErrorConfig(**{field: value})


def test_pending_conversion_keeps_its_original_error_draw():
    clock = SignalClock({"test": False})
    config = VoltageMonitorConfig(errors=VoltageErrorConfig(noise_bound_v=0.1, seed=11))
    monitor = VoltageMonitor(clock, config, lambda observation: None)
    monitor.poll(7.4)
    pending = monitor.report()["pending"]
    project_voltage_monitor(
        monitor.report(), config.model_dump(), [{"time_s": 0, "pack_voltage_v": 7.4}]
    )
    clock.run_until(0.003)
    assert monitor.report()["readings"] == [pending]


def test_ideal_capture_encoding_stays_unchanged_and_cannot_claim_error_evidence():
    assert "errors" not in VoltageMonitorConfig().model_dump()
    raw, config, samples = capture_sequence(None)
    assert raw["model"] == "ideal_pack_voltage_monitor"
    assert "noise_v" not in raw["readings"][0]
    edited = deepcopy(raw)
    edited["readings"][0]["noise_v"] = 0
    with pytest.raises(ValueError, match="sensor errors"):
        project_voltage_monitor(edited, config, samples)
