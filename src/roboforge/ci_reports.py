"""JUnit evidence for CI systems, without interpreting missing data as success."""

import json
from pathlib import Path
from xml.etree import ElementTree as ET


def _xml_text(value):
    # XML 1.0 forbids control characters even when JSON can represent them.
    return "".join(
        c
        if c in "\t\n\r"
        or 0x20 <= ord(c) <= 0xD7FF
        or 0xE000 <= ord(c) <= 0xFFFD
        or 0x10000 <= ord(c) <= 0x10FFFF
        else "\ufffd"
        for c in str(value)
    )


def junit_report(report):
    checks = report["checks"]
    suite = ET.Element(
        "testsuite",
        {
            "name": _xml_text(report["policy"]["name"]),
            "tests": str(len(checks)),
            "failures": str(sum(c["status"] == "fail" for c in checks)),
            "errors": str(sum(c["status"] == "inconclusive" for c in checks)),
            "skipped": "0",
        },
    )
    properties = ET.SubElement(suite, "properties")
    for name, value in {
        "roboforge.version": report["software_version"],
        "acceptance.status": report["status"],
        "policy.sha256": report["policy_sha256"],
        "baseline.id": report["comparison"]["baseline_id"],
        "candidate.id": report["comparison"]["runs"][1]["id"],
    }.items():
        ET.SubElement(properties, "property", name=name, value=_xml_text(value))
    for index, check in enumerate(checks):
        case = ET.SubElement(
            suite,
            "testcase",
            {
                "classname": "roboforge.acceptance",
                "name": f"{index + 1:03d}: {_xml_text(check['name'])}",
            },
        )
        if check["status"] == "fail":
            ET.SubElement(
                case, "failure", type="AcceptanceFailure", message=_xml_text(check["reason"])
            ).text = _xml_text(check["reason"])
        elif check["status"] == "inconclusive":
            ET.SubElement(
                case, "error", type="InconclusiveEvidence", message=_xml_text(check["reason"])
            ).text = _xml_text(check["reason"])
        elif check["status"] != "pass":
            raise ValueError("unsupported acceptance status")
        ET.SubElement(case, "system-out").text = _xml_text(
            json.dumps(check, ensure_ascii=False, allow_nan=False)
        )
    ET.indent(suite)
    return ET.tostring(suite, encoding="utf-8", xml_declaration=True).decode("utf-8")


def validate_output_paths(paths, *, protected=(), store=None):
    resolved = [Path(path).resolve() for path in paths if path is not None]
    if len(set(resolved)) != len(resolved):
        raise ValueError("report output paths must be distinct")
    for path in resolved:
        if path.exists() or path in {Path(p).resolve() for p in protected}:
            raise ValueError("report output must be new and must not replace an input")
        if store is not None and path.is_relative_to(Path(store).resolve()):
            raise ValueError("report outputs must be outside the run store")


def write_junit(path, report):
    text = junit_report(report)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        stream.write(text)
