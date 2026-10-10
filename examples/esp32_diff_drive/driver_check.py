"""Compare driver commands at a prescribed reference-motor speed."""

import json
import math
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.driver import DriverInputs, TB6612Driver
from roboforge.hardware.motor import MotorDatasheet


def main():
    project = load_project(Path(__file__).with_name("robot.yaml"))
    component = next(c for c in project.components if c.id == "left_motor")
    motor = MotorDatasheet.from_component(component).derive()
    driver = TB6612Driver()
    commands = {
        "half_forward": DriverInputs(standby=True, in1=True, pwm_duty=0.5),
        "reverse": DriverInputs(standby=True, in2=True),
        "brake": DriverInputs(standby=True, in1=True, in2=True),
        "coast": DriverInputs(standby=True),
        "standby": DriverInputs(),
    }
    print(
        json.dumps(
            {
                "result_type": "estimate",
                "motor_inputs": [p.model_dump() for p in component.parameters],
                "supply": {"vm_v": 6, "vcc_v": 3.3, "source": "prescribed, not battery-coupled"},
                "output_rpm": 150,
                "cases": {
                    name: driver.evaluate(motor, inputs, 6, 3.3, 150 * math.tau / 60).model_dump()
                    for name, inputs in commands.items()
                },
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
