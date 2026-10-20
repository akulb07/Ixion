"""Compare recorded hardware suites only when their declared conditions match."""

import math

from roboforge.config import Nonnegative, Real, Schema
from roboforge.core import finite, nonnegative

from .design_identity import canonical_sha256, design_changes, project_identity
from .encoder_speed import EncoderSpeedConfig
from .project import RobotProject
from .rover_brake import RoverBrakeConfig
from .rover_firmware import ObstacleBrakeFirmware
from .rover_report import project_rover_evidence
from .rover_suite import RoverSuiteConfig, check_rover_acceptance
from .speed_firmware import EncoderSpeedFirmware
from .voltage_firmware import VoltageBrakeFirmware


class RecordedRoverMetrics(Schema):
    detection_time_s: Nonnegative | None
    brake_command_time_s: Nonnegative | None
    controller_response_delay_s: Nonnegative | None
    brake_position_m: Nonnegative | None
    distance_after_brake_m: Nonnegative | None
    stop_time_s: Nonnegative | None
    wall_contact_time_s: Nonnegative | None
    final_distance_m: Nonnegative
    final_speed_m_s: Nonnegative
    final_front_clearance_m: Real
    terminal_energy_j: Nonnegative
    peak_sampled_pack_current_a: Nonnegative
    auxiliary_terminal_energy_j: Nonnegative | None = None
    power_limit_time_s: Nonnegative | None = None
    distance_after_power_limit_m: Nonnegative | None = None
    energy_after_power_limit_j: Nonnegative | None = None
    power_response_elapsed_s: Nonnegative | None = None


METRIC_UNITS = {
    **{
        key: "s"
        for key in (
            "detection_time_s",
            "brake_command_time_s",
            "controller_response_delay_s",
            "stop_time_s",
            "wall_contact_time_s",
            "recorded_duration_s",
        )
    },
    **{
        key: "m"
        for key in (
            "brake_position_m",
            "distance_after_brake_m",
            "final_distance_m",
            "final_front_clearance_m",
        )
    },
    "final_speed_m_s": "m/s",
    "terminal_energy_j": "J",
    "peak_sampled_pack_current_a": "A",
    "final_soc": "1",
    "post_ramp_tracking_rmse_m_s": "m/s",
}


def validate_rover_suite_evidence(report):
    if report.get("experiment") != "rover_acceptance_suite" or report.get("schema_version") != 1:
        raise ValueError("design comparison requires rover suite schema 1")
    config = RoverSuiteConfig.model_validate(report["config"])
    project = RobotProject.model_validate(report["project"])
    if report["name"] != config.name:
        raise ValueError("saved suite name disagrees with its configuration")
    rows = report["cases"]
    if len(rows) != len(config.cases) or len({row["name"] for row in rows}) != len(rows):
        raise ValueError("saved suite cases must be complete and uniquely named")
    by_name = {row["name"]: row for row in rows}
    if set(by_name) != {case.name for case in config.cases}:
        raise ValueError("saved case names disagree with the suite configuration")
    project_rover_evidence(report)
    for case in config.cases:
        row = by_name[case.name]
        run = row["report"]
        if row["status"] == "error":
            if run is not None or row["checks"] or not row.get("error"):
                raise ValueError("model-error case must not contain evaluated run evidence")
            continue
        if run is None or row["status"] not in {"passed", "failed"}:
            raise ValueError("successful simulation requires an explicit case result")
        if project_identity(RobotProject.model_validate(run["project"])) != project_identity(
            project
        ):
            raise ValueError("case project differs from the suite project")
        if RoverBrakeConfig.model_validate(run["config"]) != case.scenario:
            raise ValueError("case scenario differs from the suite configuration")
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
            if run["speed_control"]["config"] != program.model_dump(mode="json"):
                raise ValueError("speed controller settings differ from declared suite settings")
        elif case.controller == "voltage_guard":
            program = VoltageBrakeFirmware(
                brake_threshold_m=case.scenario.brake_threshold_m,
                timeout_action=case.scenario.timeout_action,
                **case.voltage_guard.model_dump(),
            )
        expected_type = f"{type(program).__module__}.{type(program).__qualname__}"
        if run["firmware"]["program"] != expected_type or run["firmware"][
            "parameters"
        ] != program.model_dump(mode="json"):
            raise ValueError("recorded firmware differs from the declared suite controller")
        RecordedRoverMetrics.model_validate(run["metrics"])
        final = run["samples"][-1]
        for metric, sample in (
            ("final_distance_m", "distance_m"),
            ("final_speed_m_s", "speed_m_s"),
            ("final_front_clearance_m", "front_clearance_m"),
        ):
            if run["metrics"][metric] != final[sample]:
                raise ValueError("final metrics disagree with the last recorded sample")
        energy = 0.0
        observed_peak = 0.0
        for sample in run["samples"]:
            energy = finite(
                energy
                + nonnegative(sample["interval_terminal_energy_j"], "recorded interval energy"),
                "recorded terminal energy",
            )
            observed_peak = max(
                observed_peak, nonnegative(sample["pack_current_a"], "recorded discharge current")
            )
        if run["metrics"]["terminal_energy_j"] != energy:
            raise ValueError("terminal energy disagrees with the recorded interval energies")
        if run["metrics"]["peak_sampled_pack_current_a"] < observed_peak:
            raise ValueError("declared peak current is below a recorded current sample")
        metrics = run["metrics"]
        detection, brake = metrics["detection_time_s"], metrics["brake_command_time_s"]
        response = None if detection is None or brake is None else brake - detection
        distance_after = (
            None
            if metrics["brake_position_m"] is None
            else metrics["final_distance_m"] - metrics["brake_position_m"]
        )
        if (
            metrics["controller_response_delay_s"] != response
            or metrics["distance_after_brake_m"] != distance_after
        ):
            raise ValueError("brake response metrics disagree with recorded times or positions")
        if (brake is None) != (metrics["brake_position_m"] is None):
            raise ValueError("brake command time and position must be present together")
        checks = check_rover_acceptance(run, case.acceptance)
        expected_status = "passed" if all(check["passed"] for check in checks) else "failed"
        if row["checks"] != checks or row["status"] != expected_status:
            raise ValueError("saved acceptance results disagree with recorded evidence")
    counts = {
        status: sum(row["status"] == status for row in rows)
        for status in ("passed", "failed", "error")
    }
    if report["summary"] != {"total": len(rows), **counts} or report["status"] != (
        "passed" if counts["passed"] == len(rows) else "failed"
    ):
        raise ValueError("suite summary disagrees with individual case results")
    return config, project, by_name


def _metrics(row):
    if row["report"] is None:
        return {}
    run = row["report"]
    result = {
        **run["metrics"],
        "final_soc": run["samples"][-1]["soc"],
        "recorded_duration_s": run["samples"][-1]["time_s"] - run["samples"][0]["time_s"],
    }
    tracking = run.get("speed_control", {}).get("metrics", {}).get("post_ramp_tracking_rmse_m_s")
    result["post_ramp_tracking_rmse_m_s"] = (
        None if tracking is None else nonnegative(tracking, "tracking RMSE")
    )
    return result


def compare_rover_suites(
    baseline: dict, candidate: dict, *, baseline_sha256=None, candidate_sha256=None
) -> dict:
    try:
        base_config, base_project, base_cases = validate_rover_suite_evidence(baseline)
        next_config, next_project, next_cases = validate_rover_suite_evidence(candidate)

        def conditions(config):
            return {case.name: case.model_dump(mode="json") for case in config.cases}

        if conditions(base_config) != conditions(next_config):
            raise ValueError("rover suites have different case conditions or acceptance policies")
        cases = []
        for name in base_cases:
            before, after = base_cases[name], next_cases[name]
            base_metrics, next_metrics = _metrics(before), _metrics(after)
            metrics = {}
            units = dict(METRIC_UNITS)
            if (
                "auxiliary_terminal_energy_j" in base_metrics
                or "auxiliary_terminal_energy_j" in next_metrics
            ):
                units["auxiliary_terminal_energy_j"] = "J"
            if "power_limit_time_s" in base_metrics or "power_limit_time_s" in next_metrics:
                units["power_limit_time_s"] = "s"
            for key, unit in (
                ("distance_after_power_limit_m", "m"),
                ("energy_after_power_limit_j", "J"),
                ("power_response_elapsed_s", "s"),
            ):
                if key in base_metrics or key in next_metrics:
                    units[key] = unit
            for key, unit in units.items():
                a, b = base_metrics.get(key), next_metrics.get(key)
                delta = None if a is None or b is None else b - a
                availability = "available" if delta is not None else "missing_evidence"
                if delta is not None and not math.isfinite(delta):
                    delta, availability = None, "delta_out_of_range"
                metrics[key] = {
                    "unit": unit,
                    "baseline": a,
                    "candidate": b,
                    "delta": delta,
                    "availability": availability,
                }
            cases.append(
                {
                    "name": name,
                    "baseline_status": before["status"],
                    "candidate_status": after["status"],
                    "baseline_outcome": None
                    if before["report"] is None
                    else before["report"]["status"],
                    "candidate_outcome": None
                    if after["report"] is None
                    else after["report"]["status"],
                    "metrics": metrics,
                }
            )
        sources = {}
        for label, report, project, digest in (
            ("baseline", baseline, base_project, baseline_sha256),
            ("candidate", candidate, next_project, candidate_sha256),
        ):
            sources[label] = {
                "sha256": digest or canonical_sha256(report),
                "hash_scope": "source_file_bytes" if digest else "canonical_report_json",
                "project_sha256": canonical_sha256(project_identity(project)),
            }
        return {
            "schema_version": 1,
            "experiment": "rover_design_comparison",
            "result_type": "estimate",
            "status": candidate["status"],
            "gate": "candidate_suite_acceptance",
            "sources": sources,
            "conditions_sha256": canonical_sha256(conditions(base_config)),
            "design_changes": design_changes(base_project, next_project),
            "cases": cases,
            "runs": {"baseline": baseline, "candidate": candidate},
            "limitations": [
                "Recorded evidence comparison; no simulation is rerun and no source-code or physical-accuracy attestation is made",
                "Both suites must declare identical named cases, controller settings and acceptance policies",
                "Deltas are candidate minus baseline in the stated units; they are not a general better/worse score",
                "Cumulative energy covers each run's actual duration; inspect differing outcomes and termination times before interpretation",
                "Passing means the candidate meets the common suite policy, not that it improves every metric",
            ],
        }
    except (KeyError, TypeError, AttributeError, ValueError, RecursionError) as exc:
        raise ValueError(f"Invalid rover design comparison: {exc}") from exc
