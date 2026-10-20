"""Bounded, repeatable acceptance cases for the straight-line rover fixture."""

import math
from pathlib import Path
from typing import Literal

import yaml
from pydantic import Field, model_serializer, model_validator

from roboforge.config import Nonnegative, Positive, Schema, _UniqueKeyLoader

from .encoder_health import EncoderHealthConfig
from .encoder_speed import EncoderSpeedConfig
from .project import Identifier, RobotProject
from .rover_brake import RoverBrakeConfig, run_rover_brake
from .speed_firmware import EncoderSpeedFirmware
from .voltage_firmware import VoltageBrakeFirmware, VoltageGuardConfig

RoverStatus = Literal[
    "stopped_before_wall",
    "wall_contact",
    "brake_triggered",
    "brake_pending",
    "completed_without_brake",
    "power_limit_reached",
    "stopped_after_power_limit",
    "power_response_incomplete",
]


class RoverAcceptance(Schema):
    expected_status: RoverStatus = "stopped_before_wall"
    expected_brake_reason: Identifier | None = None
    minimum_front_clearance_m: Nonnegative | None = None
    maximum_peak_pack_current_a: Nonnegative | None = None
    maximum_fault_detection_delay_s: Nonnegative | None = None
    maximum_final_speed_m_s: Nonnegative | None = None
    maximum_terminal_energy_j: Nonnegative | None = None
    maximum_power_limit_time_s: Nonnegative | None = None
    maximum_distance_after_power_limit_m: Nonnegative | None = None
    maximum_power_response_elapsed_s: Nonnegative | None = None

    @property
    def uses_extended_bounds(self):
        return any(
            getattr(self, name) is not None
            for name in (
                "maximum_final_speed_m_s",
                "maximum_terminal_energy_j",
                "maximum_power_limit_time_s",
                "maximum_distance_after_power_limit_m",
                "maximum_power_response_elapsed_s",
            )
        )

    @model_serializer(mode="wrap")
    def preserve_legacy_policy(self, handler):
        data = handler(self)
        for name in (
            "maximum_final_speed_m_s",
            "maximum_terminal_energy_j",
            "maximum_power_limit_time_s",
            "maximum_distance_after_power_limit_m",
            "maximum_power_response_elapsed_s",
        ):
            if getattr(self, name) is None:
                data.pop(name, None)
        return data


class RoverSuiteCase(Schema):
    name: Identifier
    scenario: RoverBrakeConfig
    controller: Literal["sonar", "encoder_speed", "voltage_guard"] = "encoder_speed"
    voltage_guard: VoltageGuardConfig | None = None
    target_speed_m_s: Positive = 0.2
    target_ramp_s: Positive = 1
    encoder_health: EncoderHealthConfig | None = EncoderHealthConfig()
    acceptance: RoverAcceptance = RoverAcceptance()

    @model_serializer(mode="wrap")
    def preserve_legacy_controller(self, handler):
        data = handler(self)
        if self.voltage_guard is None:
            data.pop("voltage_guard", None)
        return data

    @model_validator(mode="after")
    def check_detection_policy(self):
        if self.controller == "voltage_guard":
            if self.voltage_guard is None or self.scenario.voltage_monitor is None:
                raise ValueError(
                    "voltage_guard controller requires voltage_guard settings and a voltage_monitor"
                )
        elif self.voltage_guard is not None:
            raise ValueError("voltage_guard settings require the voltage_guard controller")
        power_statuses = {
            "power_limit_reached",
            "stopped_after_power_limit",
            "power_response_incomplete",
        }
        if (
            self.acceptance.expected_status in power_statuses
            and self.scenario.minimum_pack_voltage_v is None
        ):
            raise ValueError("expected power-limit outcome requires a configured voltage limit")
        if self.acceptance.maximum_power_limit_time_s is not None and (
            self.scenario.minimum_pack_voltage_v is None
            or self.acceptance.expected_status not in power_statuses | {"wall_contact"}
        ):
            raise ValueError(
                "power-limit time bound requires the declared power-limit outcome and voltage limit"
            )
        if (
            self.acceptance.expected_status
            in {"stopped_after_power_limit", "power_response_incomplete"}
            and self.scenario.voltage_limit_action == "end"
        ):
            raise ValueError("post-limit outcome requires a coast or brake response")
        if (
            self.acceptance.expected_status == "power_limit_reached"
            and self.scenario.voltage_limit_action != "end"
        ):
            raise ValueError("power-limit boundary outcome requires voltage_limit_action end")
        if self.acceptance.maximum_distance_after_power_limit_m is not None and (
            self.scenario.voltage_limit_action == "end"
            or self.acceptance.expected_status
            not in {"stopped_after_power_limit", "power_response_incomplete", "wall_contact"}
        ):
            raise ValueError("post-limit distance bound requires a declared response outcome")
        if self.acceptance.maximum_power_response_elapsed_s is not None and (
            self.scenario.voltage_limit_action == "end"
            or self.acceptance.expected_status != "stopped_after_power_limit"
        ):
            raise ValueError("response elapsed bound requires a stopped-after-power-limit outcome")
        if (
            self.acceptance.maximum_fault_detection_delay_s is not None
            and not self.scenario.encoder_faults
        ):
            raise ValueError("fault detection delay requires an injected encoder fault")
        return self


class RoverSuiteConfig(Schema):
    schema_version: Literal[1] = 1
    name: Identifier
    cases: tuple[RoverSuiteCase, ...] = Field(min_length=1, max_length=16)

    @model_validator(mode="after")
    def check_budget(self):
        if len({case.name for case in self.cases}) != len(self.cases):
            raise ValueError("suite case names must be unique")
        steps = sum(
            math.ceil(case.scenario.duration_s / case.scenario.step_s) for case in self.cases
        )
        if steps > 40_000:
            raise ValueError("rover suite exceeds 40,000 total physics steps")
        return self


def load_rover_suite(path: str | Path) -> RoverSuiteConfig:
    path = Path(path)
    try:
        with path.open("rb") as stream:
            content = stream.read(131_073)
        if len(content) > 131_072:
            raise ValueError("suite exceeds 128 KiB")
        return RoverSuiteConfig.model_validate(
            yaml.load(content.decode("utf-8"), Loader=_UniqueKeyLoader)
        )
    except (OSError, ValueError, yaml.YAMLError, RecursionError) as exc:
        raise ValueError(f"Invalid rover suite {path}: {exc}") from exc


def check_rover_acceptance(report: dict, policy: RoverAcceptance) -> list[dict]:
    """Evaluate only saved evidence; missing measurements fail a requested check."""
    checks = []

    def check(name, actual, expected, passed):
        checks.append(
            {"name": name, "actual": actual, "expected": expected, "passed": bool(passed)}
        )

    check(
        "status",
        report["status"],
        policy.expected_status,
        report["status"] == policy.expected_status,
    )
    if policy.expected_brake_reason is not None:
        check(
            "brake_reason",
            report["brake_reason"],
            policy.expected_brake_reason,
            report["brake_reason"] == policy.expected_brake_reason,
        )
    for field, limit, direction in (
        ("final_front_clearance_m", policy.minimum_front_clearance_m, "minimum"),
        ("peak_sampled_pack_current_a", policy.maximum_peak_pack_current_a, "maximum"),
        ("final_speed_m_s", policy.maximum_final_speed_m_s, "maximum"),
        ("terminal_energy_j", policy.maximum_terminal_energy_j, "maximum"),
        ("distance_after_power_limit_m", policy.maximum_distance_after_power_limit_m, "maximum"),
        ("power_response_elapsed_s", policy.maximum_power_response_elapsed_s, "maximum"),
    ):
        if limit is not None:
            actual = report["metrics"].get(field)
            passed = actual is not None and (
                actual >= limit if direction == "minimum" else actual <= limit
            )
            check(field, actual, {direction: limit}, passed)
    if policy.maximum_power_limit_time_s is not None:
        actual = (report.get("power_limit_event") or {}).get("bracket_end_time_s")
        check(
            "power_limit_bracket_end_time_s",
            actual,
            {"maximum": policy.maximum_power_limit_time_s},
            actual is not None and actual <= policy.maximum_power_limit_time_s,
        )
    if policy.maximum_fault_detection_delay_s is not None:
        fault = (report.get("speed_control", {}).get("encoder_health") or {}).get("fault")
        activations = report["encoder_feedback"]["activations"]
        delay = None
        if fault is not None and activations:
            delay = fault["time_s"] - min(a["time_s"] for a in activations)
        check(
            "fault_detection_delay_s",
            delay,
            {"maximum": policy.maximum_fault_detection_delay_s},
            delay is not None and 0 <= delay <= policy.maximum_fault_detection_delay_s,
        )
    return checks


def run_rover_suite(project: RobotProject, config: RoverSuiteConfig) -> dict:
    cases = []
    for case in config.cases:
        try:
            controller = None
            if case.controller == "encoder_speed":
                controller = EncoderSpeedFirmware(
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
                controller = VoltageBrakeFirmware(
                    brake_threshold_m=case.scenario.brake_threshold_m,
                    timeout_action=case.scenario.timeout_action,
                    **case.voltage_guard.model_dump(),
                )
            report = run_rover_brake(project, case.scenario, firmware=controller)
            if case.controller == "encoder_speed":
                report["speed_control"] = controller.report()
            checks = check_rover_acceptance(report, case.acceptance)
            cases.append(
                {
                    "name": case.name,
                    "status": "passed" if all(c["passed"] for c in checks) else "failed",
                    "checks": checks,
                    "error": None,
                    "report": report,
                }
            )
        except ValueError as exc:
            cases.append(
                {
                    "name": case.name,
                    "status": "error",
                    "checks": [],
                    "error": str(exc)[:2000],
                    "report": None,
                }
            )
    counts = {
        status: sum(case["status"] == status for case in cases)
        for status in ("passed", "failed", "error")
    }
    return {
        "schema_version": 1,
        "experiment": "rover_acceptance_suite",
        "result_type": "estimate",
        "name": config.name,
        "status": "passed" if counts["passed"] == len(cases) else "failed",
        "config": config.model_dump(mode="json"),
        "project": project.model_dump(mode="json"),
        "summary": {"total": len(cases), **counts},
        "cases": cases,
        "limitations": [
            "Cases run sequentially with fresh controller, sensor, clock, motion and battery state",
            "One project shared by all cases; no component swaps, automatic tuning or general scenario scheduler",
            "Fault detection delay starts at the first actual counter-fault activation and requires a feedback-check trip",
            "Acceptance is against declared simulated evidence, not hardware readiness or physical validation",
        ],
    }
