"""Resolve the reference rotor-mounted quadrature convention explicitly."""

from roboforge.config import Positive, Schema

from .motor import MotorDatasheet
from .project import Identifier, RobotProject
from .quadrature import QuadratureEncoder


class MountedEncoder(Schema):
    motor: Identifier
    encoder: Identifier
    angle_ratio: Positive
    quadrature: QuadratureEncoder


def resolve_mounted_encoder(project: RobotProject, motor: str) -> MountedEncoder:
    parts = {c.id: c for c in project.components}
    if motor not in parts or parts[motor].category != "motor":
        raise ValueError("mounted encoder requires a motor component")
    links = [
        link
        for link in project.assembly.links
        if link.kind == "encoder_shaft" and link.source == motor
    ]
    if len(links) != 1:
        raise ValueError("motor requires exactly one outgoing encoder_shaft link")
    encoder = parts[links[0].target]
    if (
        sum(
            link.kind == "encoder_shaft" and link.target == encoder.id
            for link in project.assembly.links
        )
        != 1
    ):
        raise ValueError("mounted encoder must be linked to exactly one motor")
    if encoder.category != "encoder":
        raise ValueError("encoder_shaft target must be an encoder")
    params = {p.name: p for p in encoder.parameters}
    cycles = params.get("cycles_per_rotor_revolution")
    multiplier = params.get("edge_multiplier")
    if cycles is None or cycles.unit != "count/rev" or not cycles.value.is_integer():
        raise ValueError("encoder requires integer cycles_per_rotor_revolution in count/rev")
    if multiplier is None or multiplier.unit != "1" or multiplier.value != 4:
        raise ValueError("mounted encoder currently requires edge_multiplier=4")
    ratio = MotorDatasheet.from_component(parts[motor]).gear_ratio
    return MountedEncoder(
        motor=motor,
        encoder=encoder.id,
        angle_ratio=ratio,
        quadrature=QuadratureEncoder(cycles_per_revolution=int(cycles.value)),
    )
