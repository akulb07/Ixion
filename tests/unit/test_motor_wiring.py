import pytest

from roboforge.hardware import RobotProject
from roboforge.hardware.battery import BatteryState
from roboforge.hardware.driver import DriverInputs
from roboforge.hardware.motor_wiring import resolve_motor_connection
from roboforge.hardware.shaft_dynamics import ShaftLoad
from roboforge.hardware.shaft_runtime import ShaftRuntime
from tests.unit.test_board_inputs import project
from tests.unit.test_shaft_dynamics import constant_supply


def reversed_project():
    data = project().model_dump()
    for net in data["nets"]:
        for endpoint in net["endpoints"]:
            if endpoint["component"] == "left_motor":
                endpoint["pin"] = {"P": "N", "N": "P"}[endpoint["pin"]]
    return RobotProject.model_validate(data)


def test_reference_channels_resolve_from_motor_leads():
    model = project()
    assert resolve_motor_connection(model, "left_motor").channel == "A"
    assert resolve_motor_connection(model, "right_motor").channel == "B"
    assert resolve_motor_connection(model, "left_motor").polarity == 1


def test_reversed_leads_reverse_acceleration_and_preserve_supply_draw():
    forward = resolve_motor_connection(project(), "left_motor")
    reverse = resolve_motor_connection(reversed_project(), "left_motor")
    command = DriverInputs(standby=True, in1=True)
    states = []
    for connection in (forward, reverse):
        runtime = ShaftRuntime(constant_supply(), BatteryState(), (ShaftLoad(inertia_kg_m2=0.01),))
        states.append(runtime.step(0.01, (connection.motor_inputs(command),)).end)
    assert states[1].speed_rad_s[0] == pytest.approx(-states[0].speed_rad_s[0])
    assert states[1].point.battery.current_a == pytest.approx(states[0].point.battery.current_a)


@pytest.mark.parametrize(
    "command",
    [DriverInputs(), DriverInputs(standby=True), DriverInputs(standby=True, in1=True, in2=True)],
)
def test_reversing_leads_preserves_non_driving_modes(command):
    connection = resolve_motor_connection(reversed_project(), "left_motor")
    assert connection.motor_inputs(command).resolve() == command.resolve()


def test_cross_channel_connection_is_rejected():
    data = project().model_dump()
    # Swap the negative leads between motors, creating AO1/BO2 and BO1/AO2 pairs.
    for net in data["nets"]:
        if net["id"] in {"left_motor_n", "right_motor_n"}:
            for endpoint in net["endpoints"]:
                if endpoint["component"] == "driver":
                    endpoint["pin"] = {"AO2": "BO2", "BO2": "AO2"}[endpoint["pin"]]
    with pytest.raises(ValueError, match="one complete driver channel"):
        resolve_motor_connection(RobotProject.model_validate(data), "left_motor")
