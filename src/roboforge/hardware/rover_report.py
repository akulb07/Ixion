"""Portable inspection of saved rover evidence, without running a simulation."""

import base64
import hashlib
import json
import math
from importlib.resources import files
from pathlib import Path
from typing import Any, Literal

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictBool,
    StrictInt,
    model_serializer,
    model_validator,
)

from roboforge.config import Nonnegative, Probability, Real
from roboforge.core import finite

from .artifact_io import decode_json_object
from .encoder_health import EncoderFeedbackFault
from .rover_power import DriverMode, RoverPowerConditions, RoverPowerInterval, RoverPowerResponse
from .speed_firmware import SpeedControlSample
from .voltage_limit import VoltageLimitEvent
from .voltage_monitor import project_voltage_monitor


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
    driver_modes: tuple[DriverMode, DriverMode] | None = None
    pack_current_a: Real
    pack_voltage_v: Real
    soc: Probability

    @model_serializer(mode="wrap")
    def preserve_absent_modes(self, handler):
        data = handler(self)
        if self.driver_modes is None:
            data.pop("driver_modes", None)
        return data


class _RoverView(_View):
    schema_version: Literal[7, 8, 9, 10, 11, 12]
    experiment: Literal["straight_line_wall_brake"]
    status: str = Field(min_length=1, max_length=100)
    brake_reason: str | None = Field(default=None, max_length=200)
    samples: tuple[_Sample, ...] = Field(min_length=1, max_length=10_002)
    metrics: _Metrics

    @model_validator(mode="after")
    def check_times(self):
        if any(a.time_s >= b.time_s for a, b in zip(self.samples, self.samples[1:])):
            raise ValueError("rover samples require strictly increasing times")
        return self


class _PowerSample(_View):
    auxiliary_current_a: Nonnegative
    auxiliary_power_w: Nonnegative
    interval_auxiliary_energy_j: Nonnegative
    interval_terminal_energy_j: Nonnegative
    pack_power_w: Nonnegative


class _ResponseSample(_View):
    driver_modes: tuple[DriverMode, DriverMode]
    interval_start: RoverPowerInterval | None


def _response_interval_projection(report, view, power, sample_power):
    if view.schema_version not in (11, 12):
        return
    energy = auxiliary = 0.0
    peak = max(s.pack_current_a for s in view.samples)
    for index, (sample, raw, extra) in enumerate(
        zip(view.samples, report["samples"], sample_power)
    ):
        response = _ResponseSample.model_validate(raw)
        extra.update(response.model_dump(mode="json"))
        start = response.interval_start
        if index == 0:
            if start is not None:
                raise ValueError("initial response sample cannot contain an elapsed interval")
            continue
        previous = view.samples[index - 1]
        if (
            start is None
            or start.time_s != previous.time_s
            or start.speed_m_s != previous.speed_m_s
            or start.soc != previous.soc
        ):
            raise ValueError("power interval start disagrees with preceding motion state")
        if response.driver_modes != start.driver_modes:
            raise ValueError("driver mode changed inside a held-command interval")
        elapsed = sample.time_s - start.time_s
        if (
            extra["interval_terminal_energy_j"] != start.pack_power_w * elapsed
            or extra["interval_auxiliary_energy_j"] != start.auxiliary_power_w * elapsed
        ):
            raise ValueError("recorded interval energy disagrees with held starting power")
        if (
            start.pack_current_a < power["auxiliary_current_a"]
            or not math.isclose(
                start.pack_power_w,
                start.pack_voltage_v * start.pack_current_a,
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
            or not math.isclose(
                start.auxiliary_power_w,
                start.pack_voltage_v * power["auxiliary_current_a"],
                rel_tol=1e-12,
                abs_tol=1e-12,
            )
        ):
            raise ValueError("power interval start disagrees with supply conditions")
        energy += extra["interval_terminal_energy_j"]
        auxiliary += extra["interval_auxiliary_energy_j"]
        peak = max(peak, start.pack_current_a)
        response_event = report["power_response"]
        if (
            response_event is not None
            and start.time_s >= response_event["time_s"]
            and list(start.driver_modes) != [response_event["action"]] * 2
        ):
            raise ValueError("power interval start disagrees with latched response mode")
    if (
        report["metrics"]["terminal_energy_j"] != energy
        or report["metrics"]["auxiliary_terminal_energy_j"] != auxiliary
        or report["metrics"]["peak_sampled_pack_current_a"] != peak
    ):
        raise ValueError("response power totals disagree with interval evidence")


def _power_projection(report, view, config):
    power = RoverPowerConditions.model_validate(
        {name: config[name] for name in RoverPowerConditions.model_fields if name in config}
    )
    if view.schema_version not in (9, 10, 11, 12):
        if not power.uses_default_power:
            raise ValueError("nondefault rover power conditions require schema 9")
        if (
            any(
                s.get("auxiliary_current_a", 0) != 0
                or s.get("auxiliary_power_w", 0) != 0
                or s.get("interval_auxiliary_energy_j", 0) != 0
                for s in report["samples"]
            )
            or report["metrics"].get("auxiliary_terminal_energy_j", 0) != 0
        ):
            raise ValueError("auxiliary power evidence requires schema 9")
        return None, None
    if view.schema_version == 9 and power.minimum_pack_voltage_v is not None:
        raise ValueError("declared voltage limits require rover schema 10")
    if view.schema_version in (10, 11) and power.minimum_pack_voltage_v is None:
        raise ValueError("rover schema 10 requires a declared voltage limit")
    if (view.schema_version == 11 and power.voltage_limit_action == "end") or (
        view.schema_version not in (11, 12) and power.voltage_limit_action != "end"
    ):
        raise ValueError("post-limit response requires rover schema 11 and a coast/brake action")
    if power.uses_default_power and view.schema_version != 12:
        raise ValueError("rover schema 9 requires explicit nondefault power conditions")
    if view.samples[0].time_s != 0 or view.samples[0].soc != power.initial_soc:
        raise ValueError("initial rover sample disagrees with declared power conditions")
    extras = [_PowerSample.model_validate(s) for s in report["samples"]]
    if extras[0].interval_terminal_energy_j != 0 or extras[0].interval_auxiliary_energy_j != 0:
        raise ValueError("initial rover sample cannot contain interval energy")
    auxiliary_energy = 0.0
    for sample, extra in zip(view.samples, extras):
        if extra.auxiliary_current_a != power.auxiliary_current_a:
            raise ValueError("recorded auxiliary current disagrees with power conditions")
        if sample.pack_current_a < extra.auxiliary_current_a or sample.pack_voltage_v < 0:
            raise ValueError("recorded pack cannot supply its declared auxiliary load")
        if not math.isclose(
            extra.auxiliary_power_w,
            sample.pack_voltage_v * extra.auxiliary_current_a,
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise ValueError("recorded auxiliary power disagrees with voltage and current")
        if not math.isclose(
            extra.pack_power_w,
            sample.pack_voltage_v * sample.pack_current_a,
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise ValueError("recorded pack power disagrees with voltage and current")
        if extra.interval_auxiliary_energy_j > extra.interval_terminal_energy_j:
            raise ValueError("auxiliary interval energy exceeds total terminal energy")
        auxiliary_energy = finite(
            auxiliary_energy + extra.interval_auxiliary_energy_j, "recorded auxiliary energy"
        )
    if any(a.soc < b.soc for a, b in zip(view.samples, view.samples[1:])):
        raise ValueError("discharge-only rover SOC cannot increase")
    recorded_energy = finite(
        report["metrics"]["auxiliary_terminal_energy_j"], "auxiliary terminal energy"
    )
    if recorded_energy < 0 or recorded_energy != auxiliary_energy:
        raise ValueError("auxiliary terminal energy disagrees with interval evidence")
    conditions = power.model_dump(mode="json", exclude_none=True)
    if power.voltage_limit_action == "end":
        conditions.pop("voltage_limit_action")
    return conditions, [s.model_dump(mode="json") for s in extras]


def _voltage_limit_projection(report, view, power):
    if view.schema_version not in (10, 11, 12):
        if (
            report.get("power_limit_event") is not None
            or view.status
            in {"power_limit_reached", "stopped_after_power_limit", "power_response_incomplete"}
            or report["metrics"].get("power_limit_time_s") is not None
        ):
            raise ValueError("power-limit evidence requires rover schema 10")
        return None
    if power.get("minimum_pack_voltage_v") is None:
        if (
            report.get("power_limit_event") is not None
            or report["metrics"].get("power_limit_time_s") is not None
            or view.status
            in {"power_limit_reached", "stopped_after_power_limit", "power_response_incomplete"}
        ):
            raise ValueError("power-limit evidence requires a declared threshold")
        return None
    raw = report["power_limit_event"]
    time = report["metrics"]["power_limit_time_s"]
    if time is not None:
        time = finite(time, "recorded power-limit time")
    threshold = power["minimum_pack_voltage_v"]
    if raw is None:
        if time is not None or view.status in {
            "power_limit_reached",
            "stopped_after_power_limit",
            "power_response_incomplete",
        }:
            raise ValueError("power-limit outcome requires recorded limit evidence")
        if any(s.pack_voltage_v <= threshold for s in view.samples):
            raise ValueError("voltage-limit run contains an unreported limit crossing")
        return None
    limit = VoltageLimitEvent.model_validate(raw)
    boundary = next((s for s in view.samples if s.time_s == time), None)
    if (
        limit.time_s != time
        or boundary is None
        or (
            power.get("voltage_limit_action", "end") == "end"
            and (view.status != "power_limit_reached" or time != view.samples[-1].time_s)
        )
    ):
        raise ValueError("power-limit outcome, time and final sample disagree")
    if limit.threshold_v != threshold:
        raise ValueError("recorded voltage limit differs from its configured threshold")
    if power.get("voltage_limit_action", "end") == "end" and (
        report["metrics"].get("stop_time_s") is not None
        or report["metrics"].get("wall_contact_time_s") is not None
    ):
        raise ValueError("power-limit boundary cannot claim a stopped or wall-contact outcome")
    if any(s.pack_voltage_v <= threshold for s in view.samples if s.time_s < time):
        raise ValueError("power-limit evidence ignored an earlier recorded crossing")
    if limit.phase == "command_boundary":
        if (
            limit.voltage_v > threshold
            or limit.bracket_end_time_s != time
            or limit.time_tolerance_s != 0
            or limit.bracket_end_voltage_v != limit.voltage_v
        ):
            raise ValueError("invalid command-boundary voltage limit")
    else:
        if limit.time_tolerance_s <= 0:
            raise ValueError("physics voltage limit requires positive time tolerance")
        if not limit.voltage_v > threshold >= limit.bracket_end_voltage_v:
            raise ValueError("physics voltage limit must bracket the threshold")
        if not 0 <= limit.bracket_end_time_s - time <= limit.time_tolerance_s * 1.001:
            raise ValueError("voltage-limit time bracket exceeds its declared tolerance")
        if not math.isclose(limit.voltage_v, boundary.pack_voltage_v, rel_tol=1e-10, abs_tol=1e-10):
            raise ValueError("voltage-limit boundary disagrees with its final pack sample")
    return limit.model_dump(mode="json")


def _power_response_projection(report, view, power, limit):
    if view.schema_version not in (11, 12) or power.get("voltage_limit_action", "end") == "end":
        if (
            report.get("power_response") is not None
            or any(
                report["metrics"].get(name) is not None
                for name in (
                    "distance_after_power_limit_m",
                    "energy_after_power_limit_j",
                    "power_response_elapsed_s",
                )
            )
            or view.status in {"stopped_after_power_limit", "power_response_incomplete"}
        ):
            raise ValueError("post-limit response evidence requires rover schema 11")
        return None
    raw = report["power_response"]
    metric_names = (
        "distance_after_power_limit_m",
        "energy_after_power_limit_j",
        "power_response_elapsed_s",
    )
    if limit is None:
        if raw is not None or any(report["metrics"][name] is not None for name in metric_names):
            raise ValueError("power response requires a recorded voltage-limit event")
        return None
    response = RoverPowerResponse.model_validate(raw)
    if response.time_s != limit["time_s"] or response.action != power["voltage_limit_action"]:
        raise ValueError("power response disagrees with its limit time or configured action")
    if any(command.resolve()[0] != response.action for command in response.motor_inputs):
        raise ValueError("power response motor inputs disagree with its action")
    boundary = next(s for s in view.samples if s.time_s == response.time_s)
    at_boundary = 0.0
    for sample in report["samples"]:
        if sample["time_s"] <= response.time_s:
            at_boundary += sample["interval_terminal_energy_j"]
    if response.distance_m != boundary.distance_m or response.terminal_energy_j != at_boundary:
        raise ValueError("power response disagrees with its recorded boundary state")
    final = view.samples[-1]
    stopped_speed = finite(report["config"]["stopped_speed_m_s"], "stopped speed")
    if stopped_speed <= 0 or any(s.speed_m_s < 0 for s in view.samples):
        raise ValueError("power response requires forward samples and a positive stop threshold")
    total_energy = finite(report["metrics"]["terminal_energy_j"], "recorded terminal energy")
    for name, expected in zip(
        metric_names,
        (
            final.distance_m - response.distance_m,
            total_energy - response.terminal_energy_j,
            final.time_s - response.time_s,
        ),
    ):
        if finite(report["metrics"][name], name) != expected or expected < 0:
            raise ValueError("post-limit metrics disagree with the recorded response")
    for sample in report["samples"]:
        if sample["time_s"] > response.time_s and (
            sample["driver_modes"] != [response.action] * 2
            or sample["driver_pwm_duty"] != [c.pwm_duty for c in response.motor_inputs]
        ):
            raise ValueError("post-limit driver override did not remain latched")
    if view.status not in {
        "stopped_after_power_limit",
        "power_response_incomplete",
        "wall_contact",
    }:
        raise ValueError("post-limit run requires an explicit response outcome")
    if view.status == "stopped_after_power_limit":
        if (
            view.metrics.stop_time_s != final.time_s
            or final.speed_m_s > stopped_speed
            or final.speed_m_s < 0
            or view.metrics.wall_contact_time_s is not None
        ):
            raise ValueError("stopped response disagrees with its final motion evidence")
    elif view.status == "wall_contact":
        if view.metrics.wall_contact_time_s != final.time_s or view.metrics.stop_time_s is not None:
            raise ValueError("wall-contact response disagrees with its final time")
    elif (
        view.metrics.stop_time_s is not None
        or view.metrics.wall_contact_time_s is not None
        or final.speed_m_s <= stopped_speed
        or final.time_s != finite(report["config"]["duration_s"], "run horizon")
    ):
        raise ValueError("incomplete response must end at the horizon while still moving")
    return response.model_dump(mode="json")


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
    config = report.get("config") or {}
    if not isinstance(config, dict):
        raise ValueError("recorded rover configuration must be an object")
    grade = finite(config.get("grade_rad", 0), "recorded rover grade")
    body_grade = finite((report.get("body") or {}).get("grade_rad", 0), "recorded body grade")
    if grade != body_grade:
        raise ValueError("recorded grade disagrees with the rolling body")
    if (view.schema_version == 7 and grade != 0) or (
        view.schema_version == 8 and not 0 < grade < math.pi / 2
    ):
        raise ValueError(
            "rover schema 7 requires level travel; schema 8 requires an explicit uphill grade"
        )
    if not 0 <= grade < math.pi / 2:
        raise ValueError("recorded rover grade requires nonnegative uphill travel")
    power, sample_power = _power_projection(report, view, config)
    voltage_limit = _voltage_limit_projection(report, view, power)
    power_response = _power_response_projection(report, view, power, voltage_limit)
    _response_interval_projection(report, view, power, sample_power)
    monitor = None
    if view.schema_version == 12:
        monitor = project_voltage_monitor(
            report["voltage_monitor"], config["voltage_monitor"], report["samples"]
        )
        firmware = report["firmware"]
        if (
            "voltage_callback" in firmware
            or "voltage_callback_count" in firmware
            or firmware.get("interface_version") == 5
        ):
            count = firmware.get("voltage_callback_count")
            if (
                firmware.get("voltage_callback") is not True
                or firmware.get("interface_version") != 5
                or type(count) is not int
                or count != len(monitor["readings"])
            ):
                raise ValueError(
                    "firmware voltage callback evidence disagrees with delivered readings"
                )
    elif config.get("voltage_monitor") is not None or report.get("voltage_monitor") is not None:
        raise ValueError("voltage monitor evidence requires rover schema 12")
    metrics = view.metrics.model_dump(mode="json")
    if view.schema_version in (11, 12):
        metrics.update(
            {
                name: report["metrics"][name]
                for name in (
                    "distance_after_power_limit_m",
                    "energy_after_power_limit_j",
                    "power_response_elapsed_s",
                )
            }
        )
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
    if voltage_limit is not None:
        event(
            voltage_limit["time_s"], "power", "Declared pack-voltage limit reached", voltage_limit
        )
    if power_response is not None:
        event(
            power_response["time_s"],
            "power",
            f"Driver {power_response['action']} response latched",
            power_response,
        )
    for capture in (monitor or {}).get("readings", []):
        reading = capture["observation"]
        event(reading["time_s"], "voltage", "Voltage conversion delivered", reading)
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
            "grade_rad": grade,
            **(
                {}
                if view.schema_version not in (10, 11, 12)
                else {"power_limit_event": voltage_limit}
            ),
            **({} if view.schema_version not in (11, 12) else {"power_response": power_response}),
            **({} if monitor is None else {"voltage_monitor": monitor}),
            **(
                {}
                if power is None
                else {
                    "power_conditions": power,
                    "samples": [
                        {**sample.model_dump(mode="json"), **extra}
                        for sample, extra in zip(view.samples, sample_power)
                    ],
                }
            ),
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
            '<section id="run"><p id="fixture-scope" class="hint"></p><p id="power-scope" class="hint"></p><div id="metrics" class="metrics"></div>'
            '<div class="transport"><label for="playhead">Inspect time <output id="time"></output></label>'
            '<input id="playhead" type="range" min="0" step="1" value="0" aria-label="Recorded sample">'
            '<span id="sample-index"></span></div><div id="readout" class="readout"></div>'
            '<div id="charts"></div><p class="hint">Click a plot or event to inspect the latest physics sample at or before its time. '
            "Large plots keep bucket minima and maxima in time order, up to 1,500 points per series. Smaller oscillations may be omitted; readouts use every saved sample. Lines join recorded points.</p>"
            '<section><h2>Acceptance checks</h2><div id="checks"></div></section>'
            '<section><h2>Event timeline</h2><label for="filter">Show </label>'
            '<select id="filter"><option value="all">All events</option><option value="fault">Counter faults</option>'
            '<option value="control">Braking</option><option value="power">Power limits</option><option value="sonar">Sonar</option>'
            '<option value="voltage">Voltage readings</option>'
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
