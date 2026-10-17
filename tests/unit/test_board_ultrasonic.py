import pytest

from roboforge.hardware import RobotProject
from roboforge.hardware.board_ultrasonic import (
    UltrasonicBoardIO,
    resolve_ultrasonic_wiring,
)
from roboforge.hardware.signal_clock import SignalClock, SignalEdge
from roboforge.hardware.ultrasonic import UltrasonicConfig
from tests.unit.test_board_inputs import project


def setup(distance=lambda time: 1, model=None, interface="echo_level_shifter"):
    wiring = resolve_ultrasonic_wiring(model or project(), "esp32", "ultrasonic", interface)
    clock = SignalClock({wiring.trigger_pin: False, wiring.echo_pin: False, "encoder": False})
    observed = []
    device = UltrasonicBoardIO(
        wiring,
        clock,
        UltrasonicConfig(response_delay_s=0.0002),
        distance,
        lambda pulse, target: observed.append((pulse, target)),
    )
    return device, clock, observed


def test_reference_resolves_both_sides_of_explicit_ideal_interface():
    wiring = resolve_ultrasonic_wiring(project(), "esp32", "ultrasonic", "echo_level_shifter")
    assert wiring.trigger_pin == "GPIO16"
    assert wiring.echo_pin == "GPIO17"
    assert wiring.trigger_net == "trigger"
    assert wiring.echo_nets == ("echo_high", "echo_low")
    assert wiring.ideal_echo_interface == "echo_level_shifter"


def test_trigger_gpio_pulse_causes_echo_on_resolved_input_with_shared_clock():
    device, clock, observed = setup()
    events = []
    for pin in (device.wiring.trigger_pin, device.wiring.echo_pin, "encoder"):
        clock.on_change(
            pin, lambda event: events.append((event.pin, event.time_s, clock.read(event.pin)))
        )
    clock.schedule((SignalEdge(time_s=0.001, pin="encoder", level=True),))
    device.trigger_pulse("GPIO16", 10e-6)
    assert clock.time_s == 0 and observed == []
    clock.run_until(0.03)
    assert len(observed) == 1
    pulse, distance = observed[0]
    assert distance == 1
    assert pulse.status == "echo"
    assert events[0][:2] == ("GPIO16", 0)
    assert events[1][:2] == ("GPIO16", 10e-6)
    echo = [e for e in events if e[0] == "GPIO17"]
    assert echo[0][1] == pytest.approx(0.00021)
    assert (echo[1][1] - echo[0][1]) * 340 / 2 == pytest.approx(1)
    assert [e[2] for e in echo] == [True, False]
    assert any(e[0] == "encoder" for e in events)
    assert device.digital_read("GPIO17") is False


def test_manual_gpio_writes_sample_distance_on_rising_edge():
    distances = [1]
    device, clock, observed = setup(lambda time: distances[0])
    device.digital_write("GPIO16", True)
    clock.run_until(0.00002)
    distances[0] = 2
    device.digital_write("GPIO16", False)
    clock.run_until(0.03)
    assert observed[0][1] == 1
    pulse = observed[0][0]
    assert pulse.trigger_width_s == pytest.approx(0.00002)
    assert (pulse.echo_fall_s - pulse.echo_rise_s) * 340 / 2 == pytest.approx(1)


def test_missing_target_and_short_trigger_emit_no_echo():
    device, clock, observed = setup(lambda time: None)
    events = []
    clock.on_change("GPIO17", lambda event: events.append(event))
    device.trigger_pulse("GPIO16", 5e-6)
    clock.run_until(0.001)
    assert observed[0][0].status == "short_trigger"
    device.trigger_pulse("GPIO16", 10e-6)
    clock.run_until(0.04)
    assert observed[1][0].status == "no_echo"
    assert observed[1][0].timeout_time_s == pytest.approx(0.03101)
    assert events == []


def test_minimum_width_from_timestamp_subtraction_is_accepted_over_repeated_polls():
    device, clock, observed = setup()
    for i in range(30):
        clock.run_until(i * 0.061)
        device.trigger_pulse("GPIO16", 10e-6)
        clock.run_until(i * 0.061 + 0.03)
    assert len(observed) == 30
    assert all(p.status == "echo" for p, _ in observed)


def test_too_early_retrigger_fails_the_clock_instead_of_fabricating_a_reading():
    device, clock, observed = setup()
    device.trigger_pulse("GPIO16", 10e-6)
    clock.run_until(0.02)
    device.trigger_pulse("GPIO16", 10e-6)
    with pytest.raises(ValueError, match="busy"):
        clock.run_until(0.03)
    assert len(observed) == 1
    with pytest.raises(ValueError, match="failed"):
        clock.run_until(0.04)


def test_wrong_mcu_pin_and_non_boolean_writes_do_not_queue_edges():
    device, clock, observed = setup()
    with pytest.raises(ValueError, match="mapped trigger"):
        device.digital_write("GPIO25", True)
    with pytest.raises(ValueError, match="boolean"):
        device.digital_write("GPIO16", 1)
    with pytest.raises(ValueError, match="mapped MCU"):
        device.trigger_pulse("GPIO25", 10e-6)
    with pytest.raises(ValueError, match="mapped MCU echo"):
        device.digital_read("GPIO16")
    clock.run_until(0.03)
    assert observed == []


def test_direct_echo_path_is_supported_only_when_selected_explicitly():
    data = project().model_dump()
    data["nets"] = [n for n in data["nets"] if n["id"] not in {"echo_high", "echo_low"}]
    data["nets"].append(
        {
            "id": "direct_echo",
            "endpoints": [
                {"component": "ultrasonic", "pin": "ECHO"},
                {"component": "esp32", "pin": "GPIO17"},
            ],
        }
    )
    interface = next(c for c in data["components"] if c["id"] == "echo_level_shifter")
    for pin in interface["pins"]:
        if pin["id"] != "GND":
            pin["required"] = False
    model = RobotProject.model_validate(data)
    device, clock, observed = setup(model=model, interface=None)
    assert device.wiring.echo_nets == ("direct_echo",)
    device.trigger_pulse("GPIO16", 10e-6)
    clock.run_until(0.03)
    assert observed[0][0].status == "echo"
    with pytest.raises(ValueError, match="interface IN"):
        resolve_ultrasonic_wiring(model, "esp32", "ultrasonic", "echo_level_shifter")


@pytest.mark.parametrize(
    "issue",
    [
        "output_assignment",
        "input_assignment",
        "missing_trigger",
        "missing_echo",
        "unknown_interface",
        "wrong_interface_type",
        "wrong_sensor",
    ],
)
def test_bad_wiring_and_unsupported_parts_cannot_use_a_hidden_echo_pin(issue):
    data = project().model_dump()
    if issue in {"output_assignment", "input_assignment"}:
        pin = "GPIO16" if issue == "output_assignment" else "GPIO17"
        next(a for a in data["assignments"] if a["endpoint"]["pin"] == pin)["function"] = (
            "DIGITAL_IN" if pin == "GPIO16" else "DIGITAL_OUT"
        )
    elif issue in {"missing_trigger", "missing_echo"}:
        net = "trigger" if issue == "missing_trigger" else "echo_low"
        data["nets"] = [n for n in data["nets"] if n["id"] != net]
    elif issue == "wrong_interface_type":
        next(c for c in data["components"] if c["id"] == "echo_level_shifter")["type"] = (
            "unknown-analog-circuit"
        )
    interface = "missing" if issue == "unknown_interface" else "echo_level_shifter"
    sensor = "imu" if issue == "wrong_sensor" else "ultrasonic"
    with pytest.raises(ValueError):
        resolve_ultrasonic_wiring(RobotProject.model_validate(data), "esp32", sensor, interface)


def test_ideal_buffer_does_not_override_declared_voltage_incompatibility():
    data = project().model_dump()
    sensor = next(c for c in data["components"] if c["id"] == "ultrasonic")
    interface = next(c for c in data["components"] if c["id"] == "echo_level_shifter")
    next(p for p in sensor["pins"] if p["id"] == "ECHO")["driven_voltage"] = {
        "minimum_v": 0,
        "maximum_v": 5,
        "reference": "Test fixture output envelope",
    }
    next(p for p in interface["pins"] if p["id"] == "IN")["accepted_voltage"] = {
        "minimum_v": 0,
        "maximum_v": 3.3,
        "reference": "Test fixture input envelope",
    }
    with pytest.raises(ValueError, match="voltage"):
        resolve_ultrasonic_wiring(
            RobotProject.model_validate(data), "esp32", "ultrasonic", "echo_level_shifter"
        )
