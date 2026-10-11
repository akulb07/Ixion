from pathlib import Path

import pytest

from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.board_inputs import BoardInputReplay, EncoderBinding
from tests.unit.test_gpio_replay import trace


def project():
    return load_project(Path(__file__).parents[2] / "examples/esp32_diff_drive/robot.yaml")


def board(model=None, binding=None):
    return BoardInputReplay(
        model or project(),
        "esp32",
        {"left": trace()},
        {"left": binding or EncoderBinding(component="left_encoder")},
    )


def test_reference_mapping_and_interrupt_pin_reads():
    replay = board()
    assert replay.pin_map == {"GPIO32": ("left", "a"), "GPIO33": ("left", "b")}
    seen = []

    def callback(event):
        assert event.mcu == "esp32"
        assert replay.digital_read(event.pin) == event.level
        seen.append(event.pin)

    for pin in replay.pin_map:
        replay.attach_interrupt(pin, callback)
    assert replay.run_until(1) == 8
    assert seen == ["GPIO32", "GPIO33"] * 4


def test_swapping_wires_reverses_fixed_firmware_decoder():
    data = project().model_dump()
    for net in data["nets"]:
        if net["id"] in {"left_encoder_A", "left_encoder_B"}:
            for endpoint in net["endpoints"]:
                if endpoint["component"] == "esp32":
                    endpoint["pin"] = {"GPIO32": "GPIO33", "GPIO33": "GPIO32"}[endpoint["pin"]]
    replay = board(RobotProject.model_validate(data))
    phases = [(False, False), (True, False), (True, True), (False, True)]
    previous, count = 0, 0

    def decode(event):
        nonlocal previous, count
        current = phases.index((replay.digital_read("GPIO32"), replay.digital_read("GPIO33")))
        count += 1 if (current - previous) % 4 == 1 else -1
        previous = current

    for pin in ("GPIO32", "GPIO33"):
        replay.attach_interrupt(pin, decode)
    replay.run_until(1)
    assert count == -8


def test_digital_input_allows_reads_but_not_interrupt_attachment():
    data = project().model_dump()
    for assignment in data["assignments"]:
        if assignment["endpoint"]["pin"] == "GPIO32":
            assignment["function"] = "DIGITAL_IN"
    replay = board(RobotProject.model_validate(data))
    assert replay.digital_read("GPIO32") is False
    with pytest.raises(ValueError, match="INTERRUPT"):
        replay.attach_interrupt("GPIO32", lambda e: None)


@pytest.mark.parametrize(
    "binding",
    [
        EncoderBinding(component="battery"),
        EncoderBinding(component="left_encoder", a_pin="missing"),
        EncoderBinding(component="left_encoder", b_pin="A"),
    ],
)
def test_invalid_bindings_rejected(binding):
    with pytest.raises(ValueError):
        board(binding=binding)


def test_wiring_errors_and_unbound_reads_rejected():
    data = project().model_dump()
    data["nets"] = [n for n in data["nets"] if n["id"] != "left_encoder_A"]
    with pytest.raises(ValueError, match="wiring has errors"):
        board(RobotProject.model_validate(data))
    with pytest.raises(KeyError):
        board().digital_read("GPIO25")
