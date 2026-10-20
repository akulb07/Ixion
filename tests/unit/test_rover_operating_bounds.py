import pytest

from roboforge.hardware.rover_brake import load_rover_scenario
from roboforge.hardware.rover_suite import RoverAcceptance, RoverSuiteCase, check_rover_acceptance
from tests.unit.test_hardware_project import REFERENCE
from tests.unit.test_rover_report import recorded


def report():
    value = recorded()
    value["metrics"].update(final_speed_m_s=0.02, terminal_energy_j=2)
    return value


def test_energy_and_final_speed_bounds_pass_and_fail_independently():
    source = report()
    policy = RoverAcceptance(
        expected_status="completed_without_brake",
        maximum_final_speed_m_s=0.03,
        maximum_terminal_energy_j=1,
    )
    checks = {c["name"]: c for c in check_rover_acceptance(source, policy)}
    assert checks["final_speed_m_s"]["passed"]
    assert not checks["terminal_energy_j"]["passed"]
    assert checks["terminal_energy_j"]["actual"] == 2
    assert checks["terminal_energy_j"]["expected"] == {"maximum": 1}


def test_voltage_time_acceptance_uses_the_conservative_end_of_the_time_bracket():
    source = report()
    source["status"] = "power_limit_reached"
    source["metrics"]["power_limit_time_s"] = 0.5
    source["power_limit_event"] = {"time_s": 0.5, "bracket_end_time_s": 0.50001}
    policy = RoverAcceptance(expected_status="power_limit_reached", maximum_power_limit_time_s=0.5)
    check = check_rover_acceptance(source, policy)[-1]
    assert not check["passed"]
    assert check["actual"] == 0.50001
    assert check["name"] == "power_limit_bracket_end_time_s"


@pytest.mark.parametrize(
    "field,missing",
    [
        ("maximum_final_speed_m_s", "final_speed_m_s"),
        ("maximum_terminal_energy_j", "terminal_energy_j"),
        ("maximum_power_limit_time_s", "power_limit_time_s"),
        ("maximum_distance_after_power_limit_m", "distance_after_power_limit_m"),
        ("maximum_power_response_elapsed_s", "power_response_elapsed_s"),
    ],
)
def test_missing_requested_operating_evidence_fails_the_check(field, missing):
    source = report()
    source["metrics"].pop(missing, None)
    checks = check_rover_acceptance(
        source, RoverAcceptance(expected_status="completed_without_brake", **{field: 1})
    )
    assert checks[-1]["actual"] is None and not checks[-1]["passed"]


def test_absent_operating_bounds_preserve_legacy_policy_encoding():
    policy = RoverAcceptance()
    assert not policy.uses_extended_bounds
    assert all(
        name not in policy.model_dump()
        for name in (
            "maximum_final_speed_m_s",
            "maximum_terminal_energy_j",
            "maximum_power_limit_time_s",
            "maximum_distance_after_power_limit_m",
            "maximum_power_response_elapsed_s",
        )
    )


@pytest.mark.parametrize(
    "policy",
    [
        {"expected_status": "power_limit_reached"},
        {"maximum_power_limit_time_s": 1},
    ],
)
def test_power_limit_policy_requires_a_declared_voltage_boundary(policy):
    scenario = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml"))
    with pytest.raises(ValueError, match="voltage limit"):
        RoverSuiteCase(name="invalid", scenario=scenario, acceptance=RoverAcceptance(**policy))


@pytest.mark.parametrize(
    "field",
    [
        "maximum_final_speed_m_s",
        "maximum_terminal_energy_j",
        "maximum_power_limit_time_s",
        "maximum_distance_after_power_limit_m",
        "maximum_power_response_elapsed_s",
    ],
)
@pytest.mark.parametrize("value", [-1, True, float("nan")])
def test_invalid_operating_bounds_are_rejected(field, value):
    with pytest.raises(ValueError):
        RoverAcceptance(**{field: value})


def test_response_distance_and_elapsed_bounds_are_inclusive_and_independent():
    source = report()
    source["status"] = "stopped_after_power_limit"
    source["metrics"].update(distance_after_power_limit_m=0.04, power_response_elapsed_s=0.3)
    checks = {
        c["name"]: c
        for c in check_rover_acceptance(
            source,
            RoverAcceptance(
                expected_status="stopped_after_power_limit",
                maximum_distance_after_power_limit_m=0.04,
                maximum_power_response_elapsed_s=0.2,
            ),
        )
    }
    assert checks["distance_after_power_limit_m"]["passed"]
    assert not checks["power_response_elapsed_s"]["passed"]


def test_response_time_bound_cannot_pass_an_unfinished_coast_as_a_stopped_response():
    scenario = load_rover_scenario(REFERENCE.with_name("rover_wall.yaml")).model_copy(
        update={
            "minimum_pack_voltage_v": 8,
            "voltage_limit_action": "coast",
        }
    )
    with pytest.raises(ValueError, match="stopped-after-power-limit"):
        RoverSuiteCase(
            name="invalid",
            scenario=scenario,
            acceptance=RoverAcceptance(
                expected_status="power_response_incomplete",
                maximum_power_response_elapsed_s=1,
            ),
        )
