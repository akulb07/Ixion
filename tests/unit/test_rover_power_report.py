from copy import deepcopy

import pytest

from roboforge.hardware.rover_report import project_rover_evidence
from tests.unit.test_rover_report import recorded


def power_recorded():
    report = recorded()
    report.update(schema_version=9, config={"initial_soc": 0.5, "auxiliary_current_a": 0.05})
    report["samples"][0].update(
        soc=0.5,
        auxiliary_current_a=0.05,
        auxiliary_power_w=0.37,
        interval_auxiliary_energy_j=0,
        interval_terminal_energy_j=0,
        pack_power_w=0.74,
    )
    report["metrics"]["auxiliary_terminal_energy_j"] = 0
    return report


def test_power_projection_preserves_conditions_and_load_evidence_without_mutation():
    report = power_recorded()
    before = deepcopy(report)
    run = project_rover_evidence(report)["cases"][0]["run"]
    assert run["power_conditions"] == {
        "initial_soc": 0.5,
        "auxiliary_current_a": 0.05,
        "vcc_v": 3.3,
    }
    assert run["samples"][0]["auxiliary_power_w"] == 0.37
    assert report == before


@pytest.mark.parametrize(
    "change",
    [
        "version",
        "defaults_only",
        "wrong_initial_soc",
        "wrong_current",
        "wrong_auxiliary_power",
        "wrong_pack_power",
        "unavailable_pack_current",
        "increasing_soc",
        "energy_too_large",
        "wrong_total_energy",
        "initial_interval_energy",
        "missing_evidence",
    ],
)
def test_inconsistent_power_context_is_rejected(change):
    report = power_recorded()
    sample = report["samples"][0]
    if change == "version":
        report["schema_version"] = 7
    elif change == "defaults_only":
        report["config"] = {}
    elif change == "wrong_initial_soc":
        sample["soc"] = 0.4
    elif change == "wrong_current":
        sample["auxiliary_current_a"] = 0.04
    elif change == "wrong_auxiliary_power":
        sample["auxiliary_power_w"] = 1
    elif change == "wrong_pack_power":
        sample["pack_power_w"] = 1
    elif change == "unavailable_pack_current":
        sample["pack_current_a"] = 0.01
    elif change == "increasing_soc":
        report["samples"].append({**sample, "time_s": 1, "soc": 0.6})
    elif change == "energy_too_large":
        report["samples"].append(
            {
                **sample,
                "time_s": 1,
                "interval_auxiliary_energy_j": 1,
                "interval_terminal_energy_j": 0.5,
            }
        )
    elif change == "wrong_total_energy":
        report["metrics"]["auxiliary_terminal_energy_j"] = 1
    elif change == "initial_interval_energy":
        sample["interval_terminal_energy_j"] = 1
    elif change == "missing_evidence":
        del sample["auxiliary_power_w"]
    with pytest.raises(ValueError):
        project_rover_evidence(report)


def test_legacy_report_cannot_hide_nonzero_auxiliary_telemetry():
    report = recorded()
    report["samples"][0]["auxiliary_current_a"] = 0.1
    with pytest.raises(ValueError, match="requires schema 9"):
        project_rover_evidence(report)
