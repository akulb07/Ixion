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


def rover_suite_junit(report):
    """One CI test per simulated rover case, retaining every declared check."""
    if report.get("experiment") != "rover_acceptance_suite" or report.get("schema_version") != 1:
        raise ValueError("JUnit rover export requires a suite schema 1 report")
    cases = report["cases"]
    if not 1 <= len(cases) <= 16:
        raise ValueError("JUnit rover export requires 1 to 16 cases")
    for case in cases:
        checks = case["checks"]
        if case["status"] == "passed":
            if not checks or not all(c.get("passed") is True for c in checks):
                raise ValueError("passed rover case lacks passing check evidence")
        elif case["status"] == "failed":
            if (
                not checks
                or not all(type(c.get("passed")) is bool for c in checks)
                or not any(not c["passed"] for c in checks)
            ):
                raise ValueError("failed rover case lacks failed check evidence")
        elif case["status"] == "error":
            if not isinstance(case.get("error"), str) or not case["error"] or checks:
                raise ValueError(
                    "rover model error requires an explanation and no evaluated checks"
                )
        else:
            raise ValueError("unsupported rover suite case status")
    suite = ET.Element(
        "testsuite",
        {
            "name": _xml_text(report["name"]),
            "tests": str(len(cases)),
            "failures": str(sum(c["status"] == "failed" for c in cases)),
            "errors": str(sum(c["status"] == "error" for c in cases)),
            "skipped": "0",
        },
    )
    properties = ET.SubElement(suite, "properties")
    ET.SubElement(properties, "property", name="ixion.experiment", value="rover_acceptance_suite")
    ET.SubElement(properties, "property", name="ixion.result_type", value="simulation_estimate")
    for index, result in enumerate(cases):
        case = ET.SubElement(
            suite,
            "testcase",
            {
                "classname": "ixion.rover",
                "name": f"{index + 1:03d}: {_xml_text(result['name'])}",
            },
        )
        if result["status"] == "failed":
            failed = ", ".join(check["name"] for check in result["checks"] if not check["passed"])
            ET.SubElement(
                case, "failure", type="RoverAcceptanceFailure", message=_xml_text(failed)
            ).text = _xml_text(json.dumps(result["checks"], ensure_ascii=False, allow_nan=False))
        elif result["status"] == "error":
            ET.SubElement(
                case, "error", type="RoverModelError", message=_xml_text(result["error"])
            ).text = _xml_text(result["error"])
        ET.SubElement(case, "system-out").text = _xml_text(
            json.dumps(
                {
                    "status": result["status"],
                    "checks": result["checks"],
                    "error": result.get("error"),
                },
                ensure_ascii=False,
                allow_nan=False,
            )
        )
    ET.indent(suite)
    return ET.tostring(suite, encoding="utf-8", xml_declaration=True).decode("utf-8")


def bench_acceptance_junit(report):
    """Recompute bench checks before exporting individual CI test results."""
    from roboforge.hardware.bench_acceptance import recompute_bench_acceptance

    report = recompute_bench_acceptance(report)
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
        "ixion.experiment": "rover_bench_acceptance",
        "acceptance.status": report["status"],
        "declared.source_kind": report["source_kind"],
        "policy.sha256": report["policy_sha256"],
    }.items():
        ET.SubElement(properties, "property", name=name, value=_xml_text(value))
    for check in checks:
        case = ET.SubElement(
            suite, "testcase", classname="ixion.bench", name=_xml_text(check["name"])
        )
        if check["status"] != "pass":
            kind = "failure" if check["status"] == "fail" else "error"
            ET.SubElement(
                case,
                kind,
                type="BenchAcceptanceFailure" if kind == "failure" else "InsufficientBenchEvidence",
                message=_xml_text(check["reason"]),
            )
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
