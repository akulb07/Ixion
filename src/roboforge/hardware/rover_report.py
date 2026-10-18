"""Portable inspection of saved rover evidence, without running a simulation."""

import base64
import hashlib
import json
from importlib.resources import files
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, StrictBool, StrictInt, model_validator

from roboforge.config import Nonnegative, Probability, Real
from roboforge.core import finite

from .artifact_io import decode_json_object
from .encoder_health import EncoderFeedbackFault
from .speed_firmware import SpeedControlSample


class _View(BaseModel):
    # A projection of the supported report; other original evidence stays in JSON.
    model_config = ConfigDict(extra="ignore", frozen=True)


class _Counts(_View):
    left: StrictInt = Field(ge=-(2**53 - 1), le=2**53 - 1)
    right: StrictInt = Field(ge=-(2**53 - 1), le=2**53 - 1)


class _Metrics(_View):
    final_front_clearance_m: Real
    peak_sampled_pack_current_a: Nonnegative
    brake_command_time_s: Nonnegative | None = None
    detection_time_s: Nonnegative | None = None
    stop_time_s: Nonnegative | None = None
    wall_contact_time_s: Nonnegative | None = None


class _Health(_View):
    fault: EncoderFeedbackFault | None


class _Speed(_View):
    schema_version: Literal[2]
    samples: tuple[SpeedControlSample, ...] = Field(max_length=10_000)
    encoder_health: _Health | None = None


class _Check(_View):
    name: str = Field(min_length=1, max_length=200)
    actual: Any
    expected: Any
    passed: StrictBool


class _Sample(_View):
    time_s: Nonnegative
    distance_m: Real
    speed_m_s: Real
    front_clearance_m: Real
    encoder_counts: _Counts
    decoded_encoder_counts: _Counts
    driver_pwm_duty: tuple[Probability, Probability]
    pack_current_a: Real
    pack_voltage_v: Real
    soc: Probability


class _RoverView(_View):
    schema_version: Literal[7]
    experiment: Literal["straight_line_wall_brake"]
    status: str = Field(min_length=1, max_length=100)
    brake_reason: str | None = Field(default=None, max_length=200)
    samples: tuple[_Sample, ...] = Field(min_length=1, max_length=10_001)
    metrics: _Metrics

    @model_validator(mode="after")
    def check_times(self):
        if any(a.time_s >= b.time_s for a, b in zip(self.samples, self.samples[1:])):
            raise ValueError("rover samples require strictly increasing times")
        return self


def load_rover_evidence(path: str | Path) -> tuple[dict, str]:
    try:
        with Path(path).open("rb") as stream:
            raw = stream.read(64 * 1024 * 1024 + 1)
        report = decode_json_object(raw, maximum_bytes=64 * 1024 * 1024, label="rover evidence")
        return report, hashlib.sha256(raw).hexdigest()
    except (OSError, ValueError, RecursionError) as exc:
        raise ValueError(f"Invalid rover evidence {path}: {exc}") from exc


def _case(name, status, checks, report, error=None):
    if not isinstance(name, str) or not 1 <= len(name) <= 200:
        raise ValueError("case names require 1 to 200 characters")
    if status not in {"passed", "failed", "error", "recorded"}:
        raise ValueError("unknown suite case status")
    if not isinstance(checks, list) or len(checks) > 100:
        raise ValueError("case checks require a list of at most 100 entries")
    checks = [_Check.model_validate(check).model_dump(mode="json") for check in checks]
    if status == "passed" and (not checks or not all(c["passed"] for c in checks)):
        raise ValueError("passed suite case requires passing acceptance evidence")
    if report is None:
        if status != "error" or not isinstance(error, str):
            raise ValueError("missing rover report requires an explicit error case")
        return {"name": str(name), "status": status, "checks": checks, "error": error, "run": None}
    view = _RoverView.model_validate(report)
    metrics = view.metrics.model_dump(mode="json")
    events = []

    def event(time, kind, title, detail):
        if time is not None:
            time = finite(time, "event time")
            if time < view.samples[0].time_s or time > view.samples[-1].time_s:
                raise ValueError("delivered event lies outside the recorded sample horizon")
            events.append(
                {
                    "time_s": time,
                    "kind": kind,
                    "title": str(title),
                    "detail": detail,
                }
            )

    for entry in report.get("encoder_feedback", {}).get("activations", []):
        event(entry["time_s"], "fault", f"{entry['channel']} counter {entry['kind']}", entry)
    for field, title in (
        ("detection_time_s", "Brake request detected"),
        ("brake_command_time_s", "Brake applied"),
        ("stop_time_s", "Stopped-speed threshold reached"),
        ("wall_contact_time_s", "Wall contact"),
    ):
        event(metrics.get(field), "control", title, {"reason": view.brake_reason})
    for entry in report.get("readings", []):
        event(entry["time_s"], "sonar", f"Sonar {entry['status']}", entry)
    for entry in (report.get("imu") or {}).get("readings", []):
        event(entry["time_s"], "imu", "IMU read completed", entry.get("decoded"))
    console = report.get("serial") or {}
    for state in ("completed", "pending"):
        for entry in console.get(state, []):
            time = entry["completion_time_s"] if state == "completed" else entry["requested_time_s"]
            event(time, "serial", f"Serial {state}", entry)
    control = report.get("speed_control")
    if control is not None:
        control = _Speed.model_validate(control).model_dump(mode="json")
        times = [sample["time_s"] for sample in control["samples"]]
        if any(a >= b for a, b in zip(times, times[1:])) or any(
            time < view.samples[0].time_s or time > view.samples[-1].time_s for time in times
        ):
            raise ValueError("speed-control trace must increase within the recorded sample horizon")
        fault = (control["encoder_health"] or {}).get("fault")
        if fault is not None:
            event(fault["time_s"], "fault", "Feedback check tripped", fault)
        observations = [
            s["observation"] for s in control["samples"] if s["observation"] is not None
        ]
        if fault is not None:
            observations.append(fault["observation"])
        if any(
            abs(delta) > 2**53 - 1 for sample in observations for delta in sample["count_delta"]
        ):
            raise ValueError("count deltas exceed JavaScript's exact integer range")
    if len(events) > 20_000:
        raise ValueError("rover view exceeds 20,000 events per case")
    # Validate the complete embedded projection, including optional source fields.
    result = {
        "name": str(name),
        "status": status,
        "checks": checks,
        "error": error,
        "run": {
            **view.model_dump(mode="json"),
            "events": sorted(events, key=lambda event: event["time_s"]),
            "speed_control": control,
            "configuration": {"scenario": report.get("config"), "firmware": report.get("firmware")},
        },
    }
    json.dumps(result, allow_nan=False)
    return result


def project_rover_evidence(report: dict, source_sha256: str = "unavailable") -> dict:
    """Validate the displayed subset and retain all recorded motion samples."""
    try:
        if report.get("experiment") == "rover_bench_acceptance":
            from .bench_acceptance import recompute_bench_acceptance

            checked = recompute_bench_acceptance(report)
            data = project_rover_evidence(checked["comparison"], source_sha256)
            data["name"] = (
                f"Bench acceptance / {checked['status']} / declared {checked['source_kind']}"
            )
            data["bench_acceptance"] = {
                key: value for key, value in checked.items() if key != "comparison"
            }
            return data
        if report.get("experiment") == "rover_bench_comparison":
            from .bench_trace import recompute_bench_comparison

            bench = recompute_bench_comparison(report)
            return {
                "name": f"Bench comparison / declared {bench['source_kind']}",
                "source_sha256": source_sha256,
                "cases": [_case("simulation", "recorded", [], report["simulation"])],
                "bench": {key: value for key, value in bench.items() if key != "simulation"},
            }
        if report.get("experiment") == "rover_design_comparison":
            from .rover_comparison import compare_rover_suites

            if report.get("schema_version") != 1:
                raise ValueError("rover view supports comparison schema 1")
            comparison = compare_rover_suites(
                report["runs"]["baseline"], report["runs"]["candidate"]
            )
            # Recompute analysis from the embedded evidence rather than trust edited deltas.
            views = {
                label: project_rover_evidence(report["runs"][label])["cases"]
                for label in ("baseline", "candidate")
            }
            by_name = {
                label: {case["name"]: case for case in cases} for label, cases in views.items()
            }
            cases = []
            for result in comparison["cases"]:
                first = len(cases)
                for label in ("baseline", "candidate"):
                    case = by_name[label][result["name"]]
                    cases.append(
                        {
                            **case,
                            "name": f"{label} / {result['name']}",
                            "design": label,
                            "comparison_key": result["name"],
                            "peer_index": first + (1 if label == "baseline" else 0),
                        }
                    )
            return {
                "name": "Hardware design comparison",
                "source_sha256": source_sha256,
                "cases": cases,
                "comparison": {key: value for key, value in comparison.items() if key != "runs"},
            }
        if report.get("experiment") == "rover_acceptance_suite":
            if report.get("schema_version") != 1:
                raise ValueError("rover view supports suite schema 1")
            source_cases = report["cases"]
            if not 1 <= len(source_cases) <= 16:
                raise ValueError("rover view requires 1 to 16 cases")
            cases = [
                _case(c["name"], c["status"], c["checks"], c["report"], c.get("error"))
                for c in source_cases
            ]
            name = report["name"]
        else:
            cases = [_case("rover", "recorded", [], report)]
            name = "Rover experiment"
        return {"name": str(name), "source_sha256": source_sha256, "cases": cases}
    except (KeyError, TypeError, ValueError, AttributeError, RecursionError) as exc:
        raise ValueError(f"Unsupported or invalid rover evidence: {exc}") from exc


def render_rover_evidence(report: dict, source_sha256: str = "unavailable") -> str:
    try:
        data = project_rover_evidence(report, source_sha256)
        encoded = (
            json.dumps(data, ensure_ascii=True, allow_nan=False)
            .replace("<", "\\u003c")
            .replace("&", "\\u0026")
        )
        assets = files("roboforge").joinpath("hardware/assets")
        script = (
            "const ROVER_EVIDENCE = "
            + encoded
            + ";\n"
            + assets.joinpath("rover-report.js").read_text(encoding="utf-8")
        )
        digest = base64.b64encode(hashlib.sha256(script.encode("utf-8")).digest()).decode("ascii")
        css = assets.joinpath("rover-report.css").read_text(encoding="utf-8")
        return (
            '<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<meta http-equiv="Content-Security-Policy" content="default-src \'none\'; '
            f"script-src 'sha256-{digest}'; style-src 'unsafe-inline'; base-uri 'none'; form-action 'none'\">"
            "<title>Ixion · Rover evidence</title><style>" + css + "</style></head><body>"
            '<header><strong>IXION</strong><span>ROVER / RECORDED EVIDENCE</span><span id="suite-name"></span></header>'
            '<aside><h2>EXPERIMENTS</h2><nav id="cases" aria-label="Recorded cases"></nav>'
            "<p>Saved simulation evidence. No simulation runs in this viewer.</p></aside>"
            '<main><div class="heading"><h1 id="case-name"></h1><span id="outcome"></span></div>'
            '<p id="error" role="status"></p><section id="comparison" hidden><h2>Design comparison · candidate minus baseline</h2>'
            '<p id="comparison-outcomes"></p><div id="comparison-metrics" class="table-scroll"></div>'
            '<details><summary>Declared design changes</summary><pre id="design-changes"></pre></details></section>'
            '<section id="bench" hidden><h2>Bench trace · prediction minus measurement</h2><p id="bench-source"></p>'
            '<div id="bench-acceptance" hidden><h3>Declared acceptance policy</h3><p id="bench-acceptance-status"></p>'
            '<div id="bench-acceptance-checks" class="table-scroll"></div>'
            '<details><summary>Policy, coverage and scope</summary><pre id="bench-acceptance-policy"></pre></details></div>'
            '<div id="bench-metrics" class="table-scroll"></div>'
            '<label for="bench-row">Inspect measurement row <output id="bench-row-label"></output></label>'
            '<input id="bench-row" type="range" min="0" step="1" value="0" aria-label="Measurement row">'
            '<pre id="bench-detail"></pre><p class="hint">Only paired values appear as trace overlays. '
            "Selecting an in-range measurement row moves the simulation cursor. Out-of-range rows leave it unchanged; rejected rows remain unpaired.</p></section>"
            '<section id="run"><div id="metrics" class="metrics"></div>'
            '<div class="transport"><label for="playhead">Inspect time <output id="time"></output></label>'
            '<input id="playhead" type="range" min="0" step="1" value="0" aria-label="Recorded sample">'
            '<span id="sample-index"></span></div><div id="readout" class="readout"></div>'
            '<div id="charts"></div><p class="hint">Click a plot or event to inspect the latest physics sample at or before its time. '
            "Large plots keep bucket minima and maxima in time order, up to 1,500 points per series. Smaller oscillations may be omitted; readouts use every saved sample. Lines join recorded points.</p>"
            '<section><h2>Acceptance checks</h2><div id="checks"></div></section>'
            '<section><h2>Event timeline</h2><label for="filter">Show </label>'
            '<select id="filter"><option value="all">All events</option><option value="fault">Counter faults</option>'
            '<option value="control">Braking</option><option value="sonar">Sonar</option>'
            '<option value="imu">IMU reads</option><option value="serial">Serial</option></select>'
            '<div class="event-layout"><div id="events"></div><pre id="event-detail">Select an event for saved evidence.</pre></div></section>'
            '<details><summary>Recorded configuration</summary><pre id="configuration"></pre></details></section>'
            "<footer>Simulation estimates · Counter faults affect readings after GPIO decoding · "
            "Positive PWM during a latched brake is the driver brake command, not forward drive. "
            '<span id="fingerprint"></span></footer></main><script>'
            + script
            + "</script></body></html>"
        )
    except (KeyError, TypeError, ValueError, AttributeError, RecursionError) as exc:
        raise ValueError(f"Unsupported or invalid rover evidence: {exc}") from exc
