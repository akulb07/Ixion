import json
import math

import pytest

from roboforge.cli import main
from roboforge.hardware import RobotProject
from roboforge.hardware.feedback import FeedbackConfig, run_encoder_feedback
from roboforge.hardware.mounted_encoder import resolve_mounted_encoder
from roboforge.hardware.quadrature import AngularSample
from tests.unit.test_board_inputs import project
from tests.unit.test_hardware_project import REFERENCE
from tests.unit.test_motor_wiring import reversed_project


def config(**changes):
    return FeedbackConfig(inertia_kg_m2=0.01, target_count=40, **changes)


def test_reference_encoder_ratio_produces_4800_output_counts():
    encoder = resolve_mounted_encoder(project(), "left_motor")
    trace = encoder.quadrature.trace(
        (
            AngularSample(time_s=0, angle_rad=0),
            AngularSample(time_s=1, angle_rad=math.tau * encoder.angle_ratio),
        )
    )
    assert trace.final_count == 4800


@pytest.mark.parametrize(
    "field,value", [("cycles_per_rotor_revolution", 12.5), ("edge_multiplier", 2)]
)
def test_unsupported_resolution_rejected(field, value):
    data = project().model_dump()
    for component in data["components"]:
        if component["id"] == "left_encoder":
            for parameter in component["parameters"]:
                if parameter["name"] == field:
                    parameter["value"] = value
    with pytest.raises(ValueError):
        resolve_mounted_encoder(RobotProject.model_validate(data), "left_motor")


def test_missing_mount_rejected():
    data = project().model_dump()
    data["assembly"]["links"] = [
        link
        for link in data["assembly"]["links"]
        if not (link["source"] == "left_motor" and link["kind"] == "encoder_shaft")
    ]
    with pytest.raises(ValueError, match="exactly one"):
        resolve_mounted_encoder(RobotProject.model_validate(data), "left_motor")


def test_runner_decodes_pins_and_triggers_brake_on_following_interval():
    settings = config(duration_s=0.2)
    report = run_encoder_feedback(project(), settings)
    assert report == run_encoder_feedback(project(), settings)
    assert report["status"] == "brake_triggered"
    samples = report["samples"]
    index = next(i for i, s in enumerate(samples) if s["driver_mode"] == "brake")
    assert samples[index - 1]["encoder_count"] >= 40
    assert samples[index - 2]["encoder_count"] < 40
    assert samples[-1]["time_s"] == 0.2
    assert report["mounted_encoder"]["encoder"] == "left_encoder"


def test_reversed_motor_does_not_claim_positive_target_success():
    report = run_encoder_feedback(reversed_project(), config(duration_s=0.1))
    assert report["status"] == "target_not_reached"
    assert report["samples"][-1]["encoder_count"] < 0


def test_short_final_interval_and_bounded_run():
    report = run_encoder_feedback(project(), config(duration_s=0.012, step_s=0.005))
    assert [s["time_s"] for s in report["samples"]] == [0.005, 0.01, 0.012]
    with pytest.raises(ValueError, match="step budget"):
        run_encoder_feedback(project(), config(duration_s=10001, step_s=1))


def test_cli_saves_reproducible_inputs_and_refuses_overwrite(tmp_path):
    output = tmp_path / "feedback.json"
    args = [
        "encoder-feedback",
        str(REFERENCE),
        "--inertia",
        "0.01",
        "--target-count",
        "40",
        "--seconds",
        "0.2",
        "--output",
        str(output),
    ]
    assert main(args) == 0
    payload = output.read_bytes()
    report = json.loads(payload)
    assert report["project"]["name"] == project().name
    assert report["config"]["inertia_kg_m2"] == 0.01
    assert main(args) == 2
    assert output.read_bytes() == payload


def test_right_motor_uses_channel_b_and_right_encoder():
    report = run_encoder_feedback(project(), config(motor="right_motor", duration_s=0.2))
    assert report["status"] == "brake_triggered"
    assert report["motor_connection"]["channel"] == "B"
    assert report["mounted_encoder"]["encoder"] == "right_encoder"
    assert set(report["input_pin_map"]) == {"GPIO34", "GPIO35"}


def test_cli_invalid_run_leaves_no_report(tmp_path):
    output = tmp_path / "invalid.json"
    assert (
        main(
            [
                "encoder-feedback",
                str(REFERENCE),
                "--inertia",
                "-1",
                "--target-count",
                "40",
                "--output",
                str(output),
            ]
        )
        == 2
    )
    assert not output.exists()


def test_slower_controller_delays_braking_but_not_encoder_delivery():
    fast = run_encoder_feedback(project(), config(duration_s=0.2))
    slow = run_encoder_feedback(project(), config(duration_s=0.2, control_every_steps=20))
    assert slow["brake_command_time_s"] > fast["brake_command_time_s"]
    assert slow["metrics"]["target_crossing_time_s"] == pytest.approx(
        fast["metrics"]["target_crossing_time_s"]
    )
    assert (
        slow["metrics"]["controller_response_delay_s"]
        > fast["metrics"]["controller_response_delay_s"]
    )
    assert [d["time_s"] for d in slow["control_decisions"]] == [0, 0.1]
    assert slow["control_decisions"][0]["estimated_speed_rad_s"] is None


def test_encoder_estimated_average_speed_stays_within_quantization_bound():
    report = run_encoder_feedback(project(), config(duration_s=0.2, control_every_steps=10))
    samples = {s["time_s"]: s for s in report["samples"]}
    previous_time, previous_angle = 0, 0
    for decision in report["control_decisions"][1:]:
        current_angle = samples[decision["time_s"]]["angle_rad"]
        elapsed = decision["time_s"] - previous_time
        true_average = (current_angle - previous_angle) / elapsed
        bound = math.tau / report["metrics"]["encoder_counts_per_output_revolution"] / elapsed
        assert abs(decision["estimated_speed_rad_s"] - true_average) <= bound
        previous_time, previous_angle = decision["time_s"], current_angle


def test_pending_control_tick_is_not_reported_as_success():
    report = run_encoder_feedback(project(), config(duration_s=0.2, control_every_steps=100))
    assert report["status"] == "target_reached_brake_pending"
    assert report["metrics"]["target_crossing_time_s"] is not None
    assert report["metrics"]["controller_response_delay_s"] is None
    assert report["brake_command_time_s"] is None
    assert report["metrics"]["terminal_energy_j"] > 0
