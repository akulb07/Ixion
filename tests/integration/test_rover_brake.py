import math

import pytest

from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.rover_brake import load_rover_scenario, run_rover_brake
from tests.unit.test_hardware_project import REFERENCE


@pytest.fixture(scope="module")
def model():
    return load_project(REFERENCE)


@pytest.fixture(scope="module")
def scenario():
    return load_rover_scenario(REFERENCE.with_name("rover_wall.yaml"))


@pytest.fixture(scope="module")
def stopped(model, scenario):
    return run_rover_brake(model, scenario)


def test_echo_observation_changes_both_motor_commands_and_stops_before_wall(stopped, scenario):
    report = stopped
    assert report["status"] == "stopped_before_wall"
    assert report["brake_reason"] == "obstacle"
    metrics = report["metrics"]
    near = next(r for r in report["readings"] if r["distance_m"] <= scenario.brake_threshold_m)
    assert metrics["detection_time_s"] == near["time_s"]
    assert near["trigger_time_s"] < near["time_s"] <= metrics["brake_command_time_s"]
    assert 0 <= metrics["controller_response_delay_s"] < scenario.step_s
    assert metrics["wall_contact_time_s"] is None
    assert metrics["final_front_clearance_m"] > 0
    assert 0 < metrics["final_speed_m_s"] <= scenario.stopped_speed_m_s
    assert metrics["distance_after_brake_m"] > 0
    assert metrics["stop_time_s"] > metrics["brake_command_time_s"]
    assert all(
        s["driver_modes"] == ["brake", "brake"]
        for s in report["samples"]
        if s["time_s"] > metrics["brake_command_time_s"]
    )
    assert report["control_decisions"][0]["command"] == "drive"
    assert report["control_decisions"][-1]["command"] == "brake"


def test_range_comes_from_motion_at_trigger_and_echo_width(stopped, scenario):
    samples = {s["time_s"]: s for s in stopped["samples"]}
    distances = []
    for reading in stopped["readings"]:
        position = samples[reading["trigger_time_s"]]["distance_m"]
        expected = scenario.wall_distance_m - scenario.sensor_offset_m - position
        assert reading["distance_m"] == pytest.approx(expected)
        assert reading["trigger_distance_m"] == pytest.approx(expected)
        width = 2 * expected / scenario.sensor.sound_speed_m_s
        assert reading["time_s"] == pytest.approx(
            reading["trigger_time_s"]
            + scenario.sensor.minimum_trigger_s
            + scenario.sensor.response_delay_s
            + width
        )
        distances.append(reading["distance_m"])
    assert distances[-1] < distances[0]
    assert all(b <= a for a, b in zip(distances, distances[1:]))


def test_encoder_pins_decode_motion_and_shared_power_is_accounted(stopped, scenario, model):
    assert stopped["input_pin_map"] == {
        "GPIO32": ("left", "a"),
        "GPIO33": ("left", "b"),
        "GPIO34": ("right", "a"),
        "GPIO35": ("right", "b"),
    }
    assert stopped["encoder_counts_per_output_revolution"] == [4800, 4800]
    for sample in stopped["samples"]:
        expected = math.floor(
            sample["distance_m"] / model.assembly.wheel_radius_m / math.tau * 4800 + 0.5
        )
        assert sample["encoder_counts"] == {"left": expected, "right": expected}
        assert abs(sample["contact_force_per_wheel_n"]) <= sample["traction_limit_per_wheel_n"]
    assert stopped["encoder_edge_count"] == sum(stopped["samples"][-1]["encoder_counts"].values())
    initial = stopped["samples"][0]
    assert initial["pack_current_a"] == pytest.approx(sum(initial["motor_currents_a"]))
    assert initial["pack_voltage_v"] == pytest.approx(8.4 - 0.15 * initial["pack_current_a"])
    assert stopped["metrics"]["terminal_energy_j"] == pytest.approx(
        sum(s["interval_terminal_energy_j"] for s in stopped["samples"])
    )
    assert stopped["samples"][-1]["soc"] < initial["soc"]
    assert stopped["metrics"]["peak_sampled_pack_current_a"] >= initial["pack_current_a"]
    assert "continuous_current_rating_unknown" in stopped["power_diagnostics"]


def test_late_brake_reports_first_wall_contact_without_simulating_impact(model, scenario):
    report = run_rover_brake(model, scenario.model_copy(update={"brake_threshold_m": 0.3}))
    assert report["status"] == "wall_contact"
    metrics = report["metrics"]
    assert metrics["brake_command_time_s"] < metrics["wall_contact_time_s"]
    assert metrics["final_distance_m"] == pytest.approx(
        scenario.wall_distance_m - scenario.front_offset_m
    )
    assert metrics["final_front_clearance_m"] == pytest.approx(0, abs=1e-12)
    assert metrics["final_speed_m_s"] > scenario.stopped_speed_m_s
    assert metrics["stop_time_s"] is None
    assert report["samples"][-1]["time_s"] == metrics["wall_contact_time_s"]
    assert all(s["front_clearance_m"] >= -1e-12 for s in report["samples"])
    assert metrics["wall_contact_time_s"] / scenario.step_s != pytest.approx(
        round(metrics["wall_contact_time_s"] / scenario.step_s)
    )


def test_slow_controller_waits_until_its_next_control_tick(model, scenario):
    report = run_rover_brake(model, scenario.model_copy(update={"control_every_steps": 20}))
    assert report["status"] == "stopped_before_wall"
    metrics = report["metrics"]
    assert 0 <= metrics["controller_response_delay_s"] < 0.02
    assert metrics["brake_command_time_s"] / 0.02 == pytest.approx(
        round(metrics["brake_command_time_s"] / 0.02)
    )
    assert metrics["brake_command_time_s"] > 0.614


@pytest.mark.parametrize("action,reason", [("brake", "echo_timeout"), ("continue", None)])
def test_out_of_range_wall_has_no_invented_reading(model, scenario, action, reason):
    report = run_rover_brake(
        model,
        scenario.model_copy(
            update={"wall_distance_m": 5, "duration_s": 0.1, "timeout_action": action}
        ),
    )
    assert report["brake_reason"] == reason
    assert report["status"] == ("brake_triggered" if reason else "completed_without_brake")
    assert all(r["distance_m"] is None and r["status"] == "timeout" for r in report["readings"])
    assert all(
        r["trigger_distance_m"] > scenario.sensor.maximum_distance_m for r in report["readings"]
    )


def test_end_after_echo_before_next_control_reports_pending(model, scenario):
    report = run_rover_brake(
        model, scenario.model_copy(update={"wall_distance_m": 0.3, "duration_s": 0.0016})
    )
    assert report["status"] == "brake_pending"
    assert report["metrics"]["detection_time_s"] < 0.0016
    assert report["metrics"]["brake_command_time_s"] is None
    assert report["samples"][-1]["time_s"] == 0.0016


def test_end_during_echo_keeps_pending_measurement(model, scenario):
    report = run_rover_brake(model, scenario.model_copy(update={"duration_s": 0.0005}))
    assert report["status"] == "completed_without_brake"
    assert report["readings"] == []
    assert report["pending_measurement"]["trigger_distance_m"] == pytest.approx(0.92)


def test_repeat_run_is_identical(model, scenario, stopped):
    assert run_rover_brake(model, scenario) == stopped


@pytest.mark.parametrize(
    "issue", ["unequal_motor", "missing_wheel", "reversed_leads", "encoder_not_interrupt"]
)
def test_unsupported_assembly_is_rejected(model, scenario, issue):
    data = model.model_dump()
    if issue == "unequal_motor":
        motor = next(c for c in data["components"] if c["id"] == "right_motor")
        next(p for p in motor["parameters"] if p["name"] == "gear_ratio")["value"] = 90
    elif issue == "missing_wheel":
        data["assembly"]["links"] = [
            p
            for p in data["assembly"]["links"]
            if not (p["source"] == "left_motor" and p["kind"] == "shaft")
        ]
    elif issue == "reversed_leads":
        for net in data["nets"]:
            for endpoint in net["endpoints"]:
                if endpoint["component"] == "left_motor":
                    endpoint["pin"] = {"P": "N", "N": "P"}[endpoint["pin"]]
    else:
        next(a for a in data["assignments"] if a["endpoint"]["pin"] == "GPIO32")["function"] = (
            "DIGITAL_IN"
        )
    with pytest.raises(ValueError):
        run_rover_brake(RobotProject.model_validate(data), scenario)


def test_invalid_grip_does_not_return_a_no_slip_success(model, scenario):
    with pytest.raises(ValueError, match="traction exceeded"):
        run_rover_brake(model, scenario.model_copy(update={"friction_coefficient": 0.01}))


def test_minimum_accepted_poll_period_runs_on_integer_control_ticks(model, scenario):
    report = run_rover_brake(
        model, scenario.model_copy(update={"poll_every_steps": 60, "duration_s": 0.6})
    )
    assert len(report["readings"]) == 10
    assert report["status"] == "completed_without_brake"


def test_rover_records_actual_trigger_and_echo_edges_on_project_pins(stopped, scenario):
    assert stopped["schema_version"] == 7
    wiring = stopped["ultrasonic_wiring"]
    assert wiring["trigger_pin"] == "GPIO16" and wiring["echo_pin"] == "GPIO17"
    assert wiring["echo_nets"] == ("echo_high", "echo_low")
    events = stopped["ultrasonic_pin_events"]
    triggers = [e for e in events if e["pin"] == "GPIO16"]
    echoes = [e for e in events if e["pin"] == "GPIO17"]
    assert events == sorted(events, key=lambda e: e["time_s"])
    for reading in stopped["readings"]:
        high = next(e for e in triggers if e["time_s"] == reading["trigger_time_s"])
        fall = next(e for e in triggers if not e["level"] and e["time_s"] > high["time_s"])
        assert high["level"] is True
        assert fall["time_s"] - high["time_s"] == pytest.approx(scenario.sensor.minimum_trigger_s)
        echo_high = next(e for e in echoes if e["level"] and e["time_s"] > fall["time_s"])
        echo_low = next(e for e in echoes if not e["level"] and e["time_s"] > echo_high["time_s"])
        assert echo_low["time_s"] == reading["time_s"]
        assert (
            echo_low["time_s"] - echo_high["time_s"]
        ) * scenario.sensor.sound_speed_m_s / 2 == pytest.approx(reading["distance_m"])


def test_rover_adapts_to_reassigned_sonar_gpio_instead_of_using_a_hidden_pin(
    model, scenario, stopped
):
    data = model.model_dump()
    swap = {"GPIO16": "GPIO17", "GPIO17": "GPIO16"}
    for net in data["nets"]:
        for endpoint in net["endpoints"]:
            if endpoint["component"] == "esp32" and endpoint["pin"] in swap:
                endpoint["pin"] = swap[endpoint["pin"]]
    for assignment in data["assignments"]:
        if assignment["endpoint"]["pin"] in swap:
            assignment["endpoint"]["pin"] = swap[assignment["endpoint"]["pin"]]
    report = run_rover_brake(RobotProject.model_validate(data), scenario)
    assert report["ultrasonic_wiring"]["trigger_pin"] == "GPIO17"
    assert report["ultrasonic_wiring"]["echo_pin"] == "GPIO16"
    assert report["metrics"] == stopped["metrics"]
    assert report["ultrasonic_pin_events"] == [
        {**event, "pin": swap[event["pin"]]} for event in stopped["ultrasonic_pin_events"]
    ]


def test_rover_rejects_unknown_interface_model(model, scenario):
    data = model.model_dump()
    next(c for c in data["components"] if c["id"] == "echo_level_shifter")["type"] = (
        "unmodeled-circuit"
    )
    with pytest.raises(ValueError, match="supported logical buffer"):
        run_rover_brake(RobotProject.model_validate(data), scenario)


def test_end_during_trigger_has_no_fabricated_measurement(model, scenario):
    report = run_rover_brake(model, scenario.model_copy(update={"duration_s": 5e-6}))
    assert report["ultrasonic_pin_events"] == [{"time_s": 0, "pin": "GPIO16", "level": True}]
    assert report["readings"] == [] and report["pending_measurement"] is None
    assert report["samples"][-1]["ultrasonic_trigger_level"] is True
    assert report["samples"][-1]["ultrasonic_echo_level"] is False
