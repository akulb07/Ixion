import math

import pytest

from roboforge.hardware import load_project
from roboforge.hardware.rover_brake import RoverBrakeConfig, load_rover_scenario, run_rover_brake
from roboforge.hardware.rover_firmware import DriveRequest
from roboforge.hardware.rover_report import project_rover_evidence
from roboforge.hardware.rover_suite import load_rover_suite, run_rover_suite
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def incline():
    original = load_rover_scenario(REFERENCE.with_name("rover_wall_imu.yaml"))
    return RoverBrakeConfig.model_validate({**original.model_dump(), "grade_rad": math.radians(5)})


@pytest.fixture(scope="module")
def report(model, incline):
    return run_rover_brake(model, incline)


def test_uphill_rover_keeps_sensor_braking_and_path_coordinates_explicit(report, incline):
    assert report["schema_version"] == 8 and report["status"] == "stopped_before_wall"
    assert report["brake_reason"] == "obstacle"
    assert report["body"]["grade_rad"] == incline.grade_rad
    assert report["imu"]["fixture_grade_rad"] == incline.grade_rad
    assert all(sample["speed_m_s"] >= 0 for sample in report["samples"])
    view = project_rover_evidence(report)
    assert view["cases"][0]["run"]["grade_rad"] == incline.grade_rad
    for sample in report["samples"]:
        assert sample["front_clearance_m"] == pytest.approx(
            incline.wall_distance_m - incline.front_offset_m - sample["distance_m"]
        )


def test_uphill_imu_registers_include_body_gravity_projection_at_capture_time(report, incline):
    samples = {sample["time_s"]: sample for sample in report["samples"]}
    assert report["imu"]["readings"]
    gravity = report["body"]["gravity_m_s2"]
    for reading in report["imu"]["readings"]:
        physical = reading["physical_sample"]
        sample = samples[physical["time_s"]]
        expected = (
            sample["acceleration_m_s2"] + gravity * math.sin(incline.grade_rad),
            0,
            gravity * math.cos(incline.grade_rad),
        )
        assert physical["specific_force_m_s2"] == pytest.approx(expected)
        assert reading["decoded"]["specific_force_m_s2"] == pytest.approx(
            expected, abs=gravity / 16384
        )
        assert physical["angular_velocity_rad_s"] == (0, 0, 0)


def test_weak_startup_reports_unsupported_rollback_instead_of_a_stopped_pass(model, incline):
    class WeakStartup:
        def on_tick(self, inputs):
            return DriveRequest(pwm_duty=0.001)

        def on_range(self, observation, inputs):
            return None

    with pytest.raises(ValueError, match="startup torque is insufficient.*rollback"):
        run_rover_brake(model, incline, firmware=WeakStartup())


@pytest.mark.parametrize("grade", [-0.01, math.pi / 2, True, float("nan")])
def test_forward_fixture_requires_a_finite_nonnegative_grade(incline, grade):
    with pytest.raises(ValueError):
        RoverBrakeConfig.model_validate({**incline.model_dump(), "grade_rad": grade})


def test_reference_grade_suite_keeps_separate_mission_conditions_and_compatible_reports(model):
    config = load_rover_suite(REFERENCE.with_name("rover_grade_suite.yaml"))
    report = run_rover_suite(model, config)
    assert report["status"] == "passed", [
        (case["name"], case.get("error"), case["checks"]) for case in report["cases"]
    ]
    assert report["summary"]["total"] == 3
    assert [case["report"]["schema_version"] for case in report["cases"]] == [7, 8, 8]
    data = project_rover_evidence(report)
    assert [case["run"]["grade_rad"] for case in data["cases"]] == pytest.approx(
        [0, math.radians(5), math.radians(15)]
    )
    assert "grade_rad" not in report["config"]["cases"][0]["scenario"]
    assert "grade_rad" not in report["cases"][0]["report"]["body"]
