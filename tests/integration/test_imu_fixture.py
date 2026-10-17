import json
import runpy

import pytest

from tests.unit.test_hardware_project import REFERENCE


def test_host_code_wakes_project_wired_imu_and_reads_known_motion(capsys):
    runpy.run_path(str(REFERENCE.with_name("imu_check.py")), run_name="__main__")
    report = json.loads(capsys.readouterr().out)
    assert report["wiring"]["address"] == 0x68
    assert report["wiring"]["sda_pin"] == "GPIO21" and report["wiring"]["scl_pin"] == "GPIO22"
    assert report["identity_transfer"]["data"] == [0x68]
    assert report["wake_transfer"]["data"] == [0]
    readings = {r["fixture"]: r for r in report["readings"]}
    assert readings["level_at_rest"]["decoded"]["specific_force_m_s2"] == pytest.approx(
        [0, 0, 9.80665]
    )
    assert readings["forward_acceleration"]["decoded"]["specific_force_m_s2"][0] == pytest.approx(
        2, abs=9.80665 / 16384
    )
    assert readings["positive_yaw"]["decoded"]["angular_velocity_rad_s"][2] == pytest.approx(
        0.5, abs=0.00014
    )
    end = report["wake_transfer"]["completion_time_s"]
    for reading in report["readings"]:
        transfer = reading["transfer"]
        assert transfer["start_time_s"] == end
        assert transfer["sample_time_s"] == reading["input"]["time_s"]
        assert transfer["completion_time_s"] - transfer["start_time_s"] == pytest.approx(0.00153)
        assert reading["diagnostics"] == []
        end = transfer["completion_time_s"]
