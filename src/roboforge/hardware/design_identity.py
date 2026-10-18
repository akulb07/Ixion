"""Stable design fingerprints and explicit changes, independent of list ordering."""

import hashlib
import json

from .project import RobotProject


def canonical_sha256(value) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def project_identity(project: RobotProject) -> dict:
    data = project.model_dump(mode="json")
    for component in data["components"]:
        for pin in component["pins"]:
            pin["capabilities"] = sorted(pin["capabilities"])
        component["pins"] = {pin["id"]: pin for pin in component["pins"]}
        component["parameters"] = {p["name"]: p for p in component["parameters"]}
    data["components"] = {component["id"]: component for component in data["components"]}
    for net in data["nets"]:
        net["endpoints"] = sorted(f"{e['component']}.{e['pin']}" for e in net["endpoints"])
    data["nets"] = {net["id"]: net for net in data["nets"]}
    for bus in data["i2c_buses"]:
        bus["devices"] = {device["component"]: device for device in bus["devices"]}
    data["i2c_buses"] = {bus["id"]: bus for bus in data["i2c_buses"]}
    data["assignments"] = {
        f"{a['endpoint']['component']}.{a['endpoint']['pin']}": a for a in data["assignments"]
    }
    data["assembly"]["links"] = sorted(
        f"{link['kind']}:{link['source']}:{link['target']}" for link in data["assembly"]["links"]
    )
    return data


def design_changes(baseline: RobotProject, candidate: RobotProject, limit: int = 1000) -> dict:
    changes = []
    total = 0

    def visit(before, after, path, before_present=True, after_present=True):
        nonlocal total
        if (
            before_present
            and after_present
            and isinstance(before, dict)
            and isinstance(after, dict)
        ):
            for key in sorted(before.keys() | after.keys()):
                visit(before.get(key), after.get(key), f"{path}/{key}", key in before, key in after)
        elif before_present != after_present or before != after:
            total += 1
            if len(changes) < limit:
                changes.append(
                    {
                        "path": path,
                        "baseline": before,
                        "candidate": after,
                        "baseline_present": before_present,
                        "candidate_present": after_present,
                    }
                )

    visit(project_identity(baseline), project_identity(candidate), "")
    return {
        "total": total,
        "shown": len(changes),
        "truncated": total > len(changes),
        "changes": changes,
    }
