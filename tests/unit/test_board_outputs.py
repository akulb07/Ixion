import pytest

from roboforge.hardware import RobotProject
from roboforge.hardware.board_outputs import TB6612BoardOutputs
from tests.unit.test_board_inputs import project


def board(model=None):
    return TB6612BoardOutputs(model or project(), "esp32", "driver")


def test_wired_outputs_drive_both_channels_and_shared_standby():
    outputs = board()
    assert outputs.inputs("A").resolve()[0] == "standby"
    outputs.write(
        digital={"GPIO13": True, "GPIO25": True, "GPIO19": True}, pwm={"GPIO27": 0.4, "GPIO23": 0.7}
    )
    assert outputs.inputs("A").resolve() == ("forward", 0.4)
    assert outputs.inputs("B").resolve() == ("reverse", -0.7)
    outputs.write(digital={"GPIO13": False})
    assert outputs.inputs("A").resolve()[0] == "standby"
    assert outputs.inputs("B").resolve()[0] == "standby"


def test_brake_command_uses_actual_input_latches():
    outputs = board()
    outputs.write(digital={"GPIO13": True, "GPIO25": True, "GPIO26": True})
    assert outputs.inputs("A").resolve()[0] == "brake"


@pytest.mark.parametrize(
    "pwm", [{"GPIO27": 1.1}, {"GPIO27": True}, {"GPIO27": float("nan")}, {"GPIO25": 0.5}]
)
def test_bad_batch_does_not_partially_update_outputs(pwm):
    outputs = board()
    original = outputs.levels
    with pytest.raises(ValueError):
        outputs.write(digital={"GPIO13": True}, pwm=pwm)
    assert outputs.levels == original


def test_wrong_digital_pin_or_value_rejected():
    outputs = board()
    for values in ({"GPIO32": True}, {"GPIO27": True}, {"GPIO13": 1}):
        with pytest.raises(ValueError):
            outputs.write(digital=values)


def test_swapped_direction_wiring_reverses_fixed_pin_command():
    data = project().model_dump()
    for net in data["nets"]:
        if net["id"] in {"signal_AIN1", "signal_AIN2"}:
            for endpoint in net["endpoints"]:
                if endpoint["component"] == "esp32":
                    endpoint["pin"] = {"GPIO25": "GPIO26", "GPIO26": "GPIO25"}[endpoint["pin"]]
    outputs = board(RobotProject.model_validate(data))
    outputs.write(digital={"GPIO13": True, "GPIO25": True}, pwm={"GPIO27": 1})
    assert outputs.inputs("A").resolve()[0] == "reverse"


def test_wrong_pwm_assignment_rejected():
    data = project().model_dump()
    for assignment in data["assignments"]:
        if assignment["endpoint"]["pin"] == "GPIO27":
            assignment["function"] = "DIGITAL_OUT"
    with pytest.raises(ValueError, match="assigned PWM"):
        board(RobotProject.model_validate(data))
