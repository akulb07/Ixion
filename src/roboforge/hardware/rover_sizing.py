"""Project-backed sizing for the existing two-motor, straight-line fixture."""

import math
from pathlib import Path

import yaml
from pydantic import Field, model_validator

from roboforge.config import Nonnegative, Probability, _UniqueKeyLoader

from .battery import BatteryModel, BatteryState
from .design_identity import canonical_sha256, project_identity
from .driver import TB6612Driver
from .motor import MotorDatasheet
from .motor_wiring import resolve_motor_connection, resolve_rover_wheels
from .project import Identifier, RobotProject
from .rolling import RollingBody
from .rolling_sizing import RollingDemand, size_rolling_drive
from .validation import check_project


class RoverSizingConfig(RollingDemand):
    left_motor: Identifier = "left_motor"
    right_motor: Identifier = "right_motor"
    battery: Identifier = "battery"
    rotating_inertia_per_wheel_kg_m2: Nonnegative
    friction_coefficient: Nonnegative
    driven_weight_fraction: Probability = Field(gt=0)
    grade_rad: Nonnegative = Field(default=0, lt=math.pi / 2)
    soc: Probability = 1

    @model_validator(mode="after")
    def distinct_motors(self):
        if self.left_motor == self.right_motor:
            raise ValueError("select two distinct rover motors")
        return self


def load_rover_sizing(path: str | Path) -> RoverSizingConfig:
    path = Path(path)
    try:
        with path.open("rb") as stream:
            content = stream.read(65537)
        if len(content) > 65536:
            raise ValueError("sizing input exceeds 64 KiB")
        return RoverSizingConfig.model_validate(
            yaml.load(content.decode("utf-8"), Loader=_UniqueKeyLoader)
        )
    except (OSError, ValueError, yaml.YAMLError, RecursionError) as exc:
        raise ValueError(f"Invalid rover sizing input {path}: {exc}") from exc


def size_rover(project: RobotProject, config: RoverSizingConfig) -> dict:
    motors = (config.left_motor, config.right_motor)
    wheels = resolve_rover_wheels(project, motors)
    connections = tuple(resolve_motor_connection(project, name) for name in motors)
    if connections[0].driver != connections[1].driver or {c.channel for c in connections} != {
        "A",
        "B",
    }:
        raise ValueError("rover sizing requires separate channels on one TB6612 driver")
    if any(c.polarity != 1 for c in connections):
        raise ValueError("rover sizing requires positive motor lead polarity on both wheels")
    parts = {c.id: c for c in project.components}
    if config.battery not in parts:
        raise ValueError("selected battery does not exist")
    # This fixture consumes a direct pack supply, not an inferred regulator path.
    net_for = {(e.component, e.pin): n.id for n in project.nets for e in n.endpoints}
    if net_for.get((config.battery, "P")) is None or net_for.get(
        (config.battery, "P")
    ) != net_for.get((connections[0].driver, "VM")):
        raise ValueError("selected battery P must directly supply the selected driver VM net")
    if net_for.get((config.battery, "N")) is None or net_for.get(
        (config.battery, "N")
    ) != net_for.get((connections[0].driver, "GND")):
        raise ValueError("selected battery N and driver GND must share a net")
    models = tuple(MotorDatasheet.from_component(parts[name]).derive() for name in motors)
    if models[0] != models[1]:
        raise ValueError("rover sizing requires two identical motor models")
    battery = BatteryModel.from_component(parts[config.battery])
    driver = TB6612Driver()
    body = RollingBody(
        total_mass_kg=project.assembly.total_mass_kg,
        wheel_radius_m=project.assembly.wheel_radius_m,
        rotating_inertia_per_wheel_kg_m2=config.rotating_inertia_per_wheel_kg_m2,
        friction_coefficient=config.friction_coefficient,
        driven_weight_fraction=config.driven_weight_fraction,
        grade_rad=config.grade_rad,
    )
    demand = RollingDemand.model_validate(
        config.model_dump(include=set(RollingDemand.model_fields))
    )
    state = BatteryState(soc=config.soc)
    result = size_rolling_drive(models[0], battery, driver, state, body, demand)
    return {
        "schema_version": 1,
        "artifact_type": "ixion_rover_sizing",
        "result_type": "estimate",
        "engineering_readiness": "not_assessed",
        "status": result.status,
        "project": {
            "name": project.name,
            "sha256": canonical_sha256(project_identity(project)),
            "snapshot": project.model_dump(mode="json"),
        },
        "config": config.model_dump(mode="json"),
        "models": {
            "motor": models[0].model_dump(mode="json"),
            "battery": battery.model_dump(mode="json"),
            "driver": driver.model_dump(mode="json"),
            "body": body.model_dump(mode="json"),
        },
        "connections": [c.model_dump(mode="json") for c in connections],
        "wheels": wheels,
        "result": result.model_dump(mode="json"),
        "diagnostics": [d.model_dump(mode="json") for d in check_project(project)],
        "limitations": [
            "One prescribed forward speed and nonnegative acceleration on a constant uphill grade; not a mission or a startup-to-cruise trajectory.",
            "Selects the lower-duty, higher-voltage battery solution; no battery stability or transient model.",
            "Positive motoring or ideal zero-current coasting only; no reverse drive, regenerative braking, rolling resistance, load transfer or wheel slip.",
            "Supplied VCC and pack-side auxiliary current are explicit inputs, not inferred regulator or sensor consumption.",
            "Driver bridge resistance is the existing assumed 0.5-ohm average model; PWM RMS losses, temperature and protection are not modeled.",
            "Known current-reference exceedances fail screening checks, not a simulated protection trip. Unknown ratings remain unassessed even when other checks pass.",
            "Motor continuous current, gearbox strength, static hill holding and physical accuracy are unassessed. Project parameters retain their original evidence provenance.",
            "The battery point's charge-limited runtime assumes its instantaneous current stays constant; it is not predicted mission duration or a usable cutoff-aware runtime.",
        ],
    }
