"""Create a portable starter from versioned resources included in the wheel."""

import hashlib
import json
from importlib.resources import files
from pathlib import Path

import yaml

from roboforge import __version__
from roboforge.ci_reports import rover_suite_junit

from .project import RobotProject
from .rover_brake import RoverBrakeConfig
from .rover_report import render_rover_evidence
from .rover_suite import RoverSuiteConfig, run_rover_suite
from .validation import check_project

TEMPLATE_FILES = (
    "robot.yaml",
    "rover_wall.yaml",
    "rover_feedback_suite.yaml",
    "README.md",
    "LICENSE",
    ".gitignore",
)


def create_starter(
    destination: str | Path, *, template: str = "esp32-rover", run_demo: bool = False
) -> dict:
    if template != "esp32-rover":
        raise ValueError("unsupported starter template; choose esp32-rover")
    destination = Path(destination)
    if destination.exists():
        raise ValueError("starter destination already exists; choose a new directory")
    resources = files("roboforge").joinpath("hardware/templates/esp32_rover")
    payloads = {
        name: resources.joinpath(name)
        .read_text(encoding="utf-8")
        .replace("\r\n", "\n")
        .encode("utf-8")
        for name in TEMPLATE_FILES
    }
    try:
        project = RobotProject.model_validate(yaml.safe_load(payloads["robot.yaml"]))
        RoverBrakeConfig.model_validate(yaml.safe_load(payloads["rover_wall.yaml"]))
        suite = RoverSuiteConfig.model_validate(
            yaml.safe_load(payloads["rover_feedback_suite.yaml"])
        )
    except (ValueError, yaml.YAMLError, RecursionError) as exc:
        raise ValueError(f"Invalid bundled rover template: {exc}") from exc
    diagnostics = check_project(project)
    summary = None
    if run_demo:
        report = run_rover_suite(project, suite)
        encoded = (json.dumps(report, indent=2, allow_nan=False) + "\n").encode("utf-8")
        payloads["results/rover-suite.json"] = encoded
        payloads["results/rover-suite.xml"] = rover_suite_junit(report).encode("utf-8")
        payloads["results/rover-suite.html"] = render_rover_evidence(
            report, hashlib.sha256(encoded).hexdigest()
        ).encode("utf-8")
        summary = {"status": report["status"], **report["summary"]}
    manifest = {
        "schema_version": 1,
        "artifact_type": "ixion_starter_snapshot",
        "template": template,
        "ixion_version": __version__,
        "project_name": project.name,
        "component_instances": len(project.components),
        "engineering_readiness": "not_assessed",
        "diagnostics": {
            level.lower(): sum(d.severity == level for d in diagnostics)
            for level in ("ERROR", "WARNING", "INFO", "PASS")
        },
        "demo": summary,
        "files": {
            name: {"bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()}
            for name, content in sorted(payloads.items())
        },
        "scope": "Initial starter snapshot only; edits intentionally make these original hashes stale. Template values include assumptions and unverified part ratings. Demo results are simulation estimates.",
    }
    payloads["starter-manifest.json"] = (
        json.dumps(manifest, indent=2, allow_nan=False) + "\n"
    ).encode("utf-8")
    # Prepare the complete snapshot before creating the directory; writes remain exclusive.
    destination.mkdir(parents=True, exist_ok=False)
    for name, content in payloads.items():
        path = destination / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as stream:
            stream.write(content)
    return manifest
