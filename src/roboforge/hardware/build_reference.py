"""Portable parts, wiring and measurement references from one saved design."""

import csv
import hashlib
import io
import json
import stat
import zipfile
import zlib
from pathlib import Path

from .artifact_io import decode_json_object
from .design_identity import canonical_sha256, project_identity
from .encoder_speed import EncoderSpeedConfig
from .project import RobotProject
from .rover_firmware import ObstacleBrakeFirmware
from .rover_suite import RoverSuiteConfig
from .speed_firmware import EncoderSpeedFirmware
from .validation import check_project
from .voltage_firmware import VoltageBrakeFirmware

MAX_ENTRY_BYTES = 2 * 1024 * 1024
MAX_TOTAL_BYTES = 8 * 1024 * 1024
MAX_ARCHIVE_BYTES = MAX_TOTAL_BYTES + 65_536
BASE_FILES = {
    "project.json",
    "parts.csv",
    "pins.csv",
    "parameters.csv",
    "wiring.md",
    "diagnostics.json",
    "bench-checklist.csv",
    "README.md",
}
CASE_FILES = {"suite.json", "case.json", "firmware-config.json"}


def _json(value):
    return (
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n"
    ).encode("utf-8")


def _csv(columns, rows):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, columns, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        values = {}
        for key in columns:
            value = row.get(key, "")
            # Keep free-form labels/references as text when opened in a spreadsheet.
            if isinstance(value, str) and value.lstrip().startswith(("=", "+", "-", "@")):
                value = "'" + value
            values[key] = "" if value is None else value
        writer.writerow(values)
    return stream.getvalue().encode("utf-8")


def _case_config(project, suite, case_name):
    if suite is None:
        if case_name is not None:
            raise ValueError("selecting a build case requires a suite")
        return None, None
    if case_name is None and len(suite.cases) != 1:
        raise ValueError("select one named case from a multi-case suite")
    case = next((case for case in suite.cases if case_name is None or case.name == case_name), None)
    if case is None:
        raise ValueError("selected build case is not present in the suite")
    program = ObstacleBrakeFirmware(
        brake_threshold_m=case.scenario.brake_threshold_m,
        timeout_action=case.scenario.timeout_action,
    )
    if case.controller == "encoder_speed":
        program = EncoderSpeedFirmware(
            encoder=EncoderSpeedConfig.from_project(
                project, case.scenario.left_motor, case.scenario.right_motor
            ),
            encoder_health=case.encoder_health,
            target_speed_m_s=case.target_speed_m_s,
            target_ramp_s=case.target_ramp_s,
            brake_threshold_m=case.scenario.brake_threshold_m,
            timeout_action=case.scenario.timeout_action,
        )
    elif case.controller == "voltage_guard":
        program = VoltageBrakeFirmware(
            brake_threshold_m=case.scenario.brake_threshold_m,
            timeout_action=case.scenario.timeout_action,
            **case.voltage_guard.model_dump(),
        )
    return case, {
        "execution": "host_python",
        "program": f"{type(program).__module__}.{type(program).__qualname__}",
        "parameters": program.model_dump(mode="json"),
        "limitations": [
            "Configuration reference only; no ESP32 binary, flashing instructions or timing compatibility is generated"
        ],
    }


def build_reference_files(
    project: RobotProject, suite: RoverSuiteConfig | None = None, case_name: str | None = None
) -> dict[str, bytes]:
    case, firmware = _case_config(project, suite, case_name)
    diagnostics = [d.model_dump(mode="json") for d in check_project(project)]
    nets = {
        (endpoint.component, endpoint.pin): net
        for net in project.nets
        for endpoint in net.endpoints
    }
    assignments = {(a.endpoint.component, a.endpoint.pin): a.function for a in project.assignments}
    parts, parameters, pins, incomplete = [], [], [], []
    for component in sorted(project.components, key=lambda c: c.id):
        missing = [
            name
            for name in ("manufacturer", "part_number", "variant", "datasheet")
            if not getattr(component, name)
        ]
        if missing:
            incomplete.append({"component": component.id, "missing_fields": missing})
        parts.append(
            {
                "component": component.id,
                "quantity": 1,
                "category": component.category,
                "type": component.type,
                "manufacturer": component.manufacturer,
                "part_number": component.part_number,
                "variant": component.variant,
                "datasheet": component.datasheet,
                "missing_metadata": ";".join(missing),
            }
        )
        for parameter in sorted(component.parameters, key=lambda p: p.name):
            parameters.append({"component": component.id, **parameter.model_dump()})
        if component.category == "mcu":
            for pin in sorted(component.pins, key=lambda p: p.id):
                net = nets.get((component.id, pin.id))
                others = (
                    []
                    if net is None
                    else [
                        f"{p.component}.{p.pin}"
                        for p in net.endpoints
                        if (p.component, p.pin) != (component.id, pin.id)
                    ]
                )
                pins.append(
                    {
                        "mcu": component.id,
                        "pin": pin.id,
                        "kind": pin.kind,
                        "assignment": assignments.get((component.id, pin.id)),
                        "capabilities": ";".join(sorted(pin.capabilities)),
                        "net": None if net is None else net.id,
                        "connected_to": ";".join(sorted(others)),
                        "required": pin.required,
                    }
                )
    wiring = [
        "# Declared wiring reference",
        "",
        "This lists the saved logical nets, not a physical wiring or soldering instruction.",
        "",
        "```text",
    ]
    for net in sorted(project.nets, key=lambda n: n.id):
        wiring.append(
            f"{net.id}: " + " <-> ".join(sorted(f"{e.component}.{e.pin}" for e in net.endpoints))
        )
    wiring += ["```", "", "## Declared I2C buses", "", "```text"]
    for bus in sorted(project.i2c_buses, key=lambda b: b.id):
        wiring.append(
            f"{bus.id}: controller={bus.controller}, SDA={bus.sda_net}, SCL={bus.scl_net}, frequency={bus.frequency_hz:g} Hz"
        )
        for device in sorted(bus.devices, key=lambda d: d.component):
            wiring.append(f"  {device.component}: address=0x{device.address:02x}")
    wiring += [
        "```",
        "",
        "Ground, logic levels, current limits and exact module variants need independent verification.",
        "",
    ]
    measurements = [
        ("total_mass", "kg", "Record the complete assembled robot, including battery and payload"),
        ("wheel_radius", "m", "Record effective loaded rolling radius and measurement method"),
        ("wheel_speed", "m/s", "Record time-stamped speed and encoder resolution/gearing"),
        (
            "pack_voltage",
            "V",
            "Record loaded pack voltage, state of charge and operating condition",
        ),
        (
            "pack_current",
            "A",
            "Record time-stamped pack current with instrument bandwidth and uncertainty",
        ),
        (
            "brake_distance",
            "m",
            "Record initial speed, command time, travel and final speed threshold",
        ),
        (
            "sonar_response",
            "s",
            "Record target distance, trigger/echo timing and missing-echo cases",
        ),
    ]
    files = {
        "project.json": _json(project.model_dump(mode="json")),
        "parts.csv": _csv(
            [
                "component",
                "quantity",
                "category",
                "type",
                "manufacturer",
                "part_number",
                "variant",
                "datasheet",
                "missing_metadata",
            ],
            parts,
        ),
        "parameters.csv": _csv(
            ["component", "name", "value", "unit", "evidence", "reference"], parameters
        ),
        "pins.csv": _csv(
            ["mcu", "pin", "kind", "assignment", "capabilities", "net", "connected_to", "required"],
            pins,
        ),
        "wiring.md": "\n".join(wiring).encode("utf-8"),
        "diagnostics.json": _json({"diagnostics": diagnostics}),
        "bench-checklist.csv": _csv(
            [
                "measurement",
                "unit",
                "measured_value",
                "uncertainty",
                "instrument",
                "conditions",
                "notes",
            ],
            [
                {"measurement": name, "unit": unit, "notes": notes}
                for name, unit, notes in measurements
            ],
        ),
        "README.md": (
            "# Ixion build reference\n\n"
            "This packet comes from one saved project. Start with project.json and diagnostics.json.\n\n"
            "parts.csv lists component instances, one row each; missing metadata remains blank. It isn't a verified shopping list.\n"
            "parameters.csv keeps declared values, units and assumed/datasheet/measured references. These labels aren't independently verified.\n"
            "pins.csv lists every declared MCU pin and its selected function, net and connected endpoints. Unassigned fields remain blank.\n"
            "wiring.md lists logical connections. It isn't a physical wiring guide.\n"
            "bench-checklist.csv is blank measurement paperwork, not measured results. Follow an appropriate bench procedure for the actual hardware.\n\n"
            "When included, suite.json, case.json and firmware-config.json identify one host Python experiment. No firmware binary is generated.\n\n"
            "The manifest hashes every other file. Verification checks consistency, not authorship or engineering readiness.\n"
            "Keep the original packet. Save filled-in measurements separately so they aren't mistaken for the exported blank checklist.\n"
            "CSV text that could be interpreted as a spreadsheet formula is prefixed with an apostrophe; project.json retains original strings.\n"
        ).encode("utf-8"),
    }
    if case is not None:
        files.update(
            {
                "suite.json": _json(suite.model_dump(mode="json")),
                "case.json": _json(case.model_dump(mode="json")),
                "firmware-config.json": _json(firmware),
            }
        )
    manifest = {
        "schema_version": 7
        if suite is not None
        and any(
            c.scenario.voltage_monitor is not None and c.scenario.voltage_monitor.errors is not None
            for c in suite.cases
        )
        else 6
        if suite is not None and any(c.scenario.voltage_monitor is not None for c in suite.cases)
        else 5
        if suite is not None and any(c.scenario.voltage_limit_action != "end" for c in suite.cases)
        else 4
        if suite is not None
        and any(
            c.scenario.minimum_pack_voltage_v is not None or c.acceptance.uses_extended_bounds
            for c in suite.cases
        )
        else 3
        if suite is not None and any(not c.scenario.uses_default_power for c in suite.cases)
        else 2
        if suite is not None and any(c.scenario.grade_rad != 0 for c in suite.cases)
        else 1,
        "artifact": "hardware_build_reference",
        "project_name": project.name,
        "project_sha256": canonical_sha256(project_identity(project)),
        "selected_case": None if case is None else case.name,
        "component_instances": len(project.components),
        "incomplete_component_metadata": incomplete,
        "design_errors": sum(d["severity"] == "ERROR" for d in diagnostics),
        "engineering_readiness_assessed": False,
        "files": {
            name: {"sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}
            for name, content in sorted(files.items())
        },
    }
    files["manifest.json"] = _json(manifest)
    if (
        any(len(content) > MAX_ENTRY_BYTES for content in files.values())
        or sum(map(len, files.values())) > MAX_TOTAL_BYTES
    ):
        raise ValueError("build reference exceeds entry or total uncompressed size limits")
    return files


def write_build_reference(
    path: str | Path,
    project: RobotProject,
    suite: RoverSuiteConfig | None = None,
    case_name: str | None = None,
) -> dict:
    files = build_reference_files(project, suite, case_name)
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name, content in sorted(files.items()):
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, content)
    with Path(path).open("xb") as stream:
        stream.write(buffer.getvalue())
    return decode_json_object(
        files["manifest.json"], maximum_bytes=MAX_ENTRY_BYTES, label="build manifest"
    )


def verify_build_reference(path: str | Path) -> dict:
    try:
        if Path(path).stat().st_size > MAX_ARCHIVE_BYTES:
            raise ValueError("build reference archive exceeds its compressed input limit")
        with zipfile.ZipFile(path) as archive:
            entries = archive.infolist()
            names = [entry.filename for entry in entries]
            if len(names) != len(set(names)) or not 9 <= len(entries) <= 12:
                raise ValueError("build reference has duplicate or unexpected entry count")
            if set(names) - (BASE_FILES | CASE_FILES | {"manifest.json"}) or any(
                entry.flag_bits & 1
                or entry.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED)
                for entry in entries
            ):
                raise ValueError("build reference contains unsupported or encrypted entries")
            if any(
                stat.S_IFMT(entry.external_attr >> 16) not in (0, stat.S_IFREG) for entry in entries
            ):
                raise ValueError("build reference entries must be regular files")
            if (
                any(entry.file_size > MAX_ENTRY_BYTES for entry in entries)
                or sum(entry.file_size for entry in entries) > MAX_TOTAL_BYTES
            ):
                raise ValueError("build reference exceeds entry or total uncompressed size limits")
            payloads = {}
            for entry in entries:
                with archive.open(entry) as member:
                    content = member.read(entry.file_size + 1)
                if len(content) != entry.file_size:
                    raise ValueError("build reference member length disagrees with its header")
                payloads[entry.filename] = content
        manifest = decode_json_object(
            payloads["manifest.json"], maximum_bytes=MAX_ENTRY_BYTES, label="build manifest"
        )
        if (
            type(manifest.get("schema_version")) is not int
            or manifest["schema_version"] not in (1, 2, 3, 4, 5, 6, 7)
            or manifest.get("artifact") != "hardware_build_reference"
        ):
            raise ValueError("unsupported build reference manifest")
        required = (
            BASE_FILES
            | ({"manifest.json"})
            | (CASE_FILES if manifest["selected_case"] is not None else set())
        )
        if set(names) != required or set(manifest["files"]) != required - {"manifest.json"}:
            raise ValueError("build reference file membership disagrees with the manifest")
        for name, entry in manifest["files"].items():
            if entry != {
                "sha256": hashlib.sha256(payloads[name]).hexdigest(),
                "bytes": len(payloads[name]),
            }:
                raise ValueError(f"build reference checksum or size mismatch: {name}")
        project = RobotProject.model_validate(
            decode_json_object(
                payloads["project.json"], maximum_bytes=MAX_ENTRY_BYTES, label="build project"
            )
        )
        suite = (
            None
            if manifest["selected_case"] is None
            else RoverSuiteConfig.model_validate(
                decode_json_object(
                    payloads["suite.json"], maximum_bytes=MAX_ENTRY_BYTES, label="build suite"
                )
            )
        )
        expected = build_reference_files(project, suite, manifest["selected_case"])
        if payloads != expected:
            raise ValueError(
                "build reference contents disagree with the saved design or generated references"
            )
        return manifest
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        zipfile.BadZipFile,
        RuntimeError,
        RecursionError,
        zlib.error,
        EOFError,
    ) as exc:
        raise ValueError(f"Invalid build reference {path}: {exc}") from exc
