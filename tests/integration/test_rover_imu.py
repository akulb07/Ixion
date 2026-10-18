import pytest

from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    return load_rover_scenario(REFERENCE.with_name("rover_wall_imu.yaml"))


@pytest.fixture(scope="module")
def report(model, scenario):
    return run_rover_brake(model, scenario)


def test_rover_initializes_imu_over_project_bus_and_delivers_completed_reads(report):
    imu = report["imu"]
    assert report["status"] == "stopped_before_wall" and report["schema_version"] == 7
    assert imu["wiring"]["sda_pin"] == "GPIO21" and imu["wiring"]["scl_pin"] == "GPIO22"
    assert imu["ready_time_s"] == pytest.approx(0.00099)
    initial = imu["completed_transfers"][:3]
    assert [(t["operation"], t["register_address"]) for t in initial] == [
        ("read", 0x75),
        ("write", 0x6B),
        ("write", 0x1B),
    ]
    assert initial[0]["data"] == (0x68,)
    assert imu["capture_count"] == len(report["samples"]) - 1
    assert imu["polls_skipped_busy"] == 0
    assert len(imu["readings"]) > 100
    assert all(
        t["completion_time_s"] <= report["metrics"]["stop_time_s"]
        for t in imu["completed_transfers"]
    )
    for reading in imu["readings"]:
        transfer = reading["transfer"]
        assert reading["time_s"] == transfer["completion_time_s"]
        assert transfer["sample_time_s"] <= transfer["start_time_s"] < reading["time_s"]
        assert reading["time_s"] - transfer["start_time_s"] == pytest.approx(0.00153)


def test_imu_specific_force_matches_motion_before_and_after_braking(report):
    states = {s["time_s"]: s for s in report["samples"]}
    force = []
    for reading in report["imu"]["readings"]:
        physical = reading["physical_sample"]
        state = states[physical["time_s"]]
        assert physical["specific_force_m_s2"] == (state["acceleration_m_s2"], 0, 9.80665)
        decoded = reading["decoded"]
        assert decoded["specific_force_m_s2"] == pytest.approx(
            physical["specific_force_m_s2"], abs=9.80665 / 16384
        )
        assert decoded["angular_velocity_rad_s"] == [0, 0, 0]
        assert decoded["temperature_c"] == pytest.approx(25, abs=1 / 340)
        force.append(decoded["specific_force_m_s2"][0])
    assert max(force) > 0 and min(force) < 0


def test_imu_observation_does_not_change_motion_encoder_or_sonar_results(model, scenario, report):
    without = run_rover_brake(model, scenario.model_copy(update={"imu": None}))
    assert without["imu"] is None
    for field in ("metrics", "samples", "control_decisions", "readings", "ultrasonic_pin_events"):
        assert report[field] == without[field]


def test_early_end_keeps_i2c_response_pending_without_publishing_future_readings(model, scenario):
    result = run_rover_brake(model, scenario.model_copy(update={"duration_s": 0.0105}))
    imu = result["imu"]
    assert imu["readings"] == []
    pending = imu["pending_transfer"]
    assert pending["register_address"] == 0x3B
    assert pending["start_time_s"] == 0.01
    assert pending["completion_time_s"] == pytest.approx(0.01153)
    assert len(imu["completed_transfers"]) == 3
    assert all(t["completion_time_s"] <= 0.0105 for t in imu["completed_transfers"])


def test_end_during_initialization_does_not_fabricate_a_ready_sensor(model, scenario):
    result = run_rover_brake(model, scenario.model_copy(update={"duration_s": 0.0001}))
    imu = result["imu"]
    assert imu["ready_time_s"] is None and imu["capture_count"] == 0
    assert imu["completed_transfers"] == [] and imu["readings"] == []
    assert imu["pending_transfer"]["register_address"] == 0x75


def test_nonblocking_polling_skips_busy_requests_while_other_peripherals_advance(model, scenario):
    fast_poll = scenario.imu.model_copy(update={"poll_every_steps": 1})
    result = run_rover_brake(
        model, scenario.model_copy(update={"imu": fast_poll, "duration_s": 0.03})
    )
    imu = result["imu"]
    assert imu["polls_skipped_busy"] > 0 and len(imu["readings"]) > 5
    assert result["samples"][-1]["time_s"] == 0.03
    assert result["encoder_edge_count"] > 0
    assert len(result["ultrasonic_pin_events"]) == 4
    for left, right in zip(imu["completed_transfers"], imu["completed_transfers"][1:]):
        assert right["start_time_s"] >= left["completion_time_s"]


def test_slow_bus_initialization_keeps_running_physics_and_sonar(model, scenario):
    data = model.model_dump()
    data["i2c_buses"][0]["frequency_hz"] = 100
    result = run_rover_brake(
        RobotProject.model_validate(data), scenario.model_copy(update={"duration_s": 0.1})
    )
    assert result["imu"]["ready_time_s"] is None
    assert result["imu"]["readings"] == []
    assert result["metrics"]["final_distance_m"] > 0
    assert result["encoder_edge_count"] > 0 and result["readings"]


def test_imu_requires_a_body_mount(model, scenario):
    data = model.model_dump()
    data["assembly"]["links"] = [
        p for p in data["assembly"]["links"] if not (p["kind"] == "mount" and p["target"] == "imu")
    ]
    with pytest.raises(ValueError, match="mount from a chassis"):
        run_rover_brake(RobotProject.model_validate(data), scenario)


def test_combined_rover_run_is_repeatable(model, scenario, report):
    assert run_rover_brake(model, scenario) == report


def test_response_completing_exactly_at_run_end_is_delivered(model, scenario):
    result = run_rover_brake(model, scenario.model_copy(update={"duration_s": 0.01153}))
    imu = result["imu"]
    assert len(imu["readings"]) == 1
    assert imu["readings"][0]["time_s"] == 0.01153
    assert imu["readings"][0]["transfer"]["sample_time_s"] == 0.01
    assert imu["pending_transfer"] is None


def test_rover_firmware_configures_ranges_before_decoding(model, scenario):
    imu_config = scenario.imu.model_copy(
        update={"accel_range_selector": 1, "gyro_range_selector": 2}
    )
    result = run_rover_brake(
        model, scenario.model_copy(update={"imu": imu_config, "duration_s": 0.03})
    )
    imu = result["imu"]
    assert imu["completed_transfers"][2]["data"] == (16, 8)
    assert imu["readings"][0]["decoded"]["raw_counts"][2] == 8192
    assert imu["readings"][0]["decoded"]["specific_force_m_s2"][2] == 9.80665
