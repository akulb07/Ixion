import base64
import hashlib
import json
import re
from copy import deepcopy

import pytest

from roboforge.cli import main
from roboforge.hardware.rover_report import load_rover_evidence, render_rover_evidence


def recorded():
    return {
        "schema_version": 7,
        "experiment": "straight_line_wall_brake",
        "status": "completed_without_brake",
        "brake_reason": None,
        "samples": [
            {
                "time_s": 0,
                "distance_m": 0,
                "speed_m_s": 0,
                "front_clearance_m": 0.9,
                "encoder_counts": {"left": 0, "right": 0},
                "decoded_encoder_counts": {"left": 0, "right": 0},
                "driver_pwm_duty": [0.02, 0.02],
                "pack_current_a": 0.1,
                "pack_voltage_v": 7.4,
                "soc": 1,
            }
        ],
        "metrics": {"final_front_clearance_m": 0.9, "peak_sampled_pack_current_a": 0.1},
    }


def embedded(html):
    script = re.search(r"<script>(.*?)</script>", html, re.S).group(1)
    payload = script.split("const ROVER_EVIDENCE = ", 1)[1].split(";\n", 1)[0]
    return json.loads(payload), script


def test_viewer_embeds_recorded_samples_and_only_hash_authorized_script():
    source = recorded()
    before = deepcopy(source)
    html = render_rover_evidence(source, "source_digest")
    data, script = embedded(html)
    assert data["source_sha256"] == "source_digest"
    assert data["cases"][0]["run"]["samples"] == source["samples"]
    digest = base64.b64encode(hashlib.sha256(script.encode()).digest()).decode()
    assert f"script-src 'sha256-{digest}'" in html
    assert "default-src 'none'" in html
    assert "<script src=" not in html and "<link " not in html
    assert 'id="playhead"' in html
    assert source == before


def test_suite_text_cannot_break_out_of_script_or_add_executable_markup():
    label = '</script><script>alert("injected")</script>&'
    report = {
        "schema_version": 1,
        "experiment": "rover_acceptance_suite",
        "name": label,
        "cases": [{"name": label, "status": "recorded", "checks": [], "report": recorded()}],
    }
    html = render_rover_evidence(report)
    assert label not in html and html.count("<script>") == 1
    data, _ = embedded(html)
    assert data["name"] == label and data["cases"][0]["name"] == label


@pytest.mark.parametrize(
    "change", ["version", "unordered", "nonfinite", "unsafe_integer", "bad_speed"]
)
def test_invalid_or_unsupported_recorded_data_is_rejected(change):
    report = recorded()
    if change == "version":
        report["schema_version"] = 6
    elif change == "unordered":
        report["samples"] *= 2
    elif change == "nonfinite":
        report["samples"][0]["pack_voltage_v"] = float("inf")
    elif change == "unsafe_integer":
        report["samples"][0]["encoder_counts"]["left"] = 2**53
    else:
        report["speed_control"] = {"schema_version": 1, "samples": []}
    with pytest.raises(ValueError, match="Unsupported or invalid"):
        render_rover_evidence(report)


def test_failed_and_error_suite_cases_remain_visible():
    report = {
        "schema_version": 1,
        "experiment": "rover_acceptance_suite",
        "name": "mixed",
        "cases": [
            {
                "name": "failed",
                "status": "failed",
                "checks": [
                    {
                        "name": "status",
                        "actual": "unfinished",
                        "expected": "stopped",
                        "passed": False,
                    }
                ],
                "report": recorded(),
            },
            {
                "name": "error",
                "status": "error",
                "checks": [],
                "report": None,
                "error": "traction limit",
            },
        ],
    }
    data, _ = embedded(render_rover_evidence(report))
    assert data["cases"][0]["checks"][0]["passed"] is False
    assert data["cases"][1]["run"] is None
    assert data["cases"][1]["error"] == "traction limit"
    report["cases"][0]["status"] = "passed"
    with pytest.raises(ValueError, match="passing acceptance evidence"):
        render_rover_evidence(report)


@pytest.mark.parametrize(
    "raw",
    [
        '{"schema_version":7,"schema_version":6}',
        '{"number":NaN}',
        '{"number":1e999}',
        "[]",
        '{"unfinished":',
    ],
    ids=["duplicate", "nan", "overflow", "array", "truncated"],
)
def test_bad_json_evidence_does_not_create_html(tmp_path, raw):
    source, output = tmp_path / "source.json", tmp_path / "view.html"
    source.write_text(raw)
    assert main(["rover-report", str(source), "--output", str(output)]) == 2
    assert not output.exists()


def test_cli_reads_saved_json_without_mutating_it_and_preserves_existing_html(tmp_path):
    source, output = tmp_path / "source.json", tmp_path / "view.html"
    raw = json.dumps(recorded()).encode()
    source.write_bytes(raw)
    report, digest = load_rover_evidence(source)
    assert report == recorded() and digest == hashlib.sha256(raw).hexdigest()
    args = ["rover-report", str(source), "--output", str(output)]
    assert main(args) == 0
    payload = output.read_bytes()
    assert main(args) == 2
    assert output.read_bytes() == payload and source.read_bytes() == raw


def test_pending_serial_event_is_at_request_time_not_future_completion():
    report = recorded()
    report["serial"] = {
        "pending": [
            {
                "requested_time_s": 0,
                "completion_time_s": 2,
                "text": "still pending",
            }
        ]
    }
    data, _ = embedded(render_rover_evidence(report))
    event = data["cases"][0]["run"]["events"][0]
    assert event["time_s"] == 0 and event["detail"]["completion_time_s"] == 2


def test_feedback_trip_event_exposes_the_controllers_observed_evidence():
    report = recorded()
    final = deepcopy(report["samples"][0])
    final["time_s"] = 1
    report["samples"].append(final)
    fault = {
        "time_s": 0.5,
        "since_s": 0.3,
        "code": "encoder_no_progress",
        "wheels": ["left", "right"],
        "target_speed_m_s": 0.2,
        "requested_pwm_duty": 0.3,
        "observation": {
            "time_s": 0.5,
            "interval_start_s": 0.48,
            "interval_s": 0.02,
            "count_delta": [0, 0],
            "wheel_speed_m_s": [0, 0],
            "mean_speed_m_s": 0,
        },
    }
    report["speed_control"] = {
        "schema_version": 2,
        "samples": [],
        "encoder_health": {"fault": fault},
    }
    data, _ = embedded(render_rover_evidence(report))
    [event] = data["cases"][0]["run"]["events"]
    assert event["title"] == "Feedback check tripped"
    assert event["detail"] == fault


def test_future_completed_events_are_rejected_instead_of_shown_as_delivered():
    report = recorded()
    report["serial"] = {"completed": [{"completion_time_s": 2, "text": "not delivered"}]}
    with pytest.raises(ValueError, match="outside the recorded sample horizon"):
        render_rover_evidence(report)
