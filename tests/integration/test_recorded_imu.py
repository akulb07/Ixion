import hashlib
import json

import pytest

pytest.importorskip("mcap_ros2")
from mcap_ros2.writer import Writer

from roboforge.cli import main
from roboforge.recorded_imu import extract_imu

DEFINITION = """std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""


def message():
    return {
        "header": {"stamp": {"sec": 1_700_000_000, "nanosec": 21}, "frame_id": "imu_link"},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "orientation_covariance": [0.0] * 9,
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": -0.2},
        "angular_velocity_covariance": [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
        "linear_acceleration": {"x": 0.3, "y": 0.0, "z": 9.81},
        "linear_acceleration_covariance": [0.0] * 9,
    }


def write(path, messages, schema_name="sensor_msgs/msg/Imu"):
    with path.open("wb") as stream:
        writer = Writer(stream)
        schema = writer.register_msgdef(schema_name, DEFINITION)
        for i, value in enumerate(messages):
            writer.write_message(
                "/imu/data", schema, value, log_time=100 + i, publish_time=90 + i, sequence=i
            )
        writer.finish()


def test_cdr_measurements_preserve_units_covariance_and_timestamps(tmp_path):
    path = tmp_path / "imu.mcap"
    first, second = message(), message()
    second["header"]["stamp"]["nanosec"] = 11
    write(path, [first, second, second])
    before = path.read_bytes()
    report = extract_imu(path, "/imu/data")
    assert report["source_sha256"] == hashlib.sha256(before).hexdigest()
    assert report["frame_id"] == "imu_link"
    assert report["header_time_regressions"] == 1
    assert report["repeated_header_times"] == 1
    assert report["units"]["angular_velocity"] == "rad/s"
    assert report["units"]["linear_acceleration"] == "m/s^2"
    sample = report["samples"][0]
    assert sample["header_time_ns"] == 1_700_000_000_000_000_021
    assert sample["log_time_ns"] == 100 and sample["publish_time_ns"] == 90
    assert sample["orientation"]["covariance_status"] == "unknown"
    assert sample["angular_velocity"]["covariance_status"] == "provided"
    assert sample["angular_velocity"]["covariance"] == first["angular_velocity_covariance"]
    assert sample["linear_acceleration"]["value"]["z"] == 9.81
    assert path.read_bytes() == before
    output = tmp_path / "imu.json"
    assert main(["extract-imu", str(path), "--topic", "/imu/data", "--output", str(output)]) == 0
    assert json.loads(output.read_text())["samples"] == report["samples"]
    assert main(["extract-imu", str(path), "--topic", "/imu/data", "--output", str(output)]) == 2
    assert main(["extract-imu", str(path), "--topic", "/imu/data", "--output", str(path)]) == 2
    assert path.read_bytes() == before


@pytest.mark.parametrize("name", ["orientation", "angular_velocity", "linear_acceleration"])
def test_unavailable_estimates_ignore_placeholder_values(tmp_path, name):
    value = message()
    value[name]["x"] = float("nan")
    value[name + "_covariance"][0] = -1
    path = tmp_path / "imu.mcap"
    write(path, [value])
    report = extract_imu(path, "/imu/data")
    assert report["samples"][0][name]["value"] is None
    assert report["samples"][0][name]["availability"] == "unavailable"
    assert report["unavailable_counts"][name] == 1
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize(
    "issue", ["frame_change", "empty_frame", "nan", "quaternion", "timestamp", "covariance"]
)
def test_invalid_samples_fail_without_publishing_output(tmp_path, issue):
    first, second = message(), message()
    if issue == "frame_change":
        second["header"]["frame_id"] = "base_link"
    elif issue == "empty_frame":
        second["header"]["frame_id"] = ""
    elif issue == "nan":
        second["linear_acceleration"]["x"] = float("nan")
    elif issue == "quaternion":
        second["orientation"]["w"] = 0.0
    elif issue == "timestamp":
        second["header"]["stamp"]["nanosec"] = 1_000_000_000
    else:
        second["angular_velocity_covariance"][1] = float("nan")
    path, output = tmp_path / "bad.mcap", tmp_path / "out.json"
    write(path, [first, second])
    assert main(["extract-imu", str(path), "--topic", "/imu/data", "--output", str(output)]) == 2
    assert not output.exists()


def test_topic_schema_and_budgets(tmp_path):
    path = tmp_path / "imu.mcap"
    write(path, [message(), message()])
    with pytest.raises(ValueError, match="no recorded messages"):
        extract_imu(path, "/missing")
    for budget in (0, True, 1.5, 100_001, 1):
        with pytest.raises(ValueError, match="sample budget"):
            extract_imu(path, "/imu/data", max_samples=budget)
    write(path, [message()], schema_name="custom_msgs/Imu")
    with pytest.raises(ValueError, match="sensor_msgs/Imu"):
        extract_imu(path, "/imu/data")
