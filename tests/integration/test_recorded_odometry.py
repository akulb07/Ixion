import copy
import hashlib
import json

import pytest

pytest.importorskip("mcap_ros2")
from mcap_ros2.writer import Writer

from roboforge.cli import main
from roboforge.recorded_odometry import extract_odometry

DEFINITION = """std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/PoseWithCovariance
geometry_msgs/Pose pose
float64[36] covariance
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/TwistWithCovariance
geometry_msgs/Twist twist
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""


def message():
    return {
        "header": {"stamp": {"sec": 1_700_000_000, "nanosec": 21}, "frame_id": "odom"},
        "child_frame_id": "base_link",
        "pose": {
            "pose": {
                "position": {"x": 1.5, "y": -2.0, "z": 0.25},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "covariance": [0.1] * 36,
        },
        "twist": {
            "twist": {
                "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": -0.2},
            },
            "covariance": [0.2] * 36,
        },
    }


def write(path, messages, schema_name="nav_msgs/msg/Odometry"):
    with path.open("wb") as stream:
        writer = Writer(stream)
        schema = writer.register_msgdef(schema_name, DEFINITION)
        for i, value in enumerate(messages):
            writer.write_message(
                "/wheel/odom", schema, value, log_time=100 + i, publish_time=90 + i, sequence=i
            )
        writer.finish()


def test_cdr_estimates_preserve_values_frames_and_three_timestamps(tmp_path):
    path = tmp_path / "odom.mcap"
    first = message()
    second = copy.deepcopy(first)
    second["header"]["stamp"]["nanosec"] = 11
    write(path, [first, second, second])
    before = path.read_bytes()
    report = extract_odometry(path, "/wheel/odom")
    assert report["kind"] == "recorded_odometry_estimate"
    assert report["source_sha256"] == hashlib.sha256(before).hexdigest()
    assert report["pose_frame_id"] == "odom" and report["twist_frame_id"] == "base_link"
    assert report["header_time_regressions"] == 1 and report["repeated_header_times"] == 1
    sample = report["samples"][0]
    assert sample["header_time_ns"] == 1_700_000_000_000_000_021
    assert sample["log_time_ns"] == 100 and sample["publish_time_ns"] == 90
    assert sample["position_m"] == first["pose"]["pose"]["position"]
    assert sample["angular_velocity_rad_s"]["z"] == -0.2
    assert sample["pose_covariance"] == [0.1] * 36
    assert path.read_bytes() == before
    output = tmp_path / "trajectory.json"
    assert (
        main(["extract-odometry", str(path), "--topic", "/wheel/odom", "--output", str(output)])
        == 0
    )
    assert json.loads(output.read_text())["samples"] == report["samples"]


@pytest.mark.parametrize("issue", ["frames", "empty_frame", "nan", "quaternion", "timestamp"])
def test_invalid_or_incompatible_samples_fail_without_output(tmp_path, issue):
    first, second = message(), message()
    if issue == "frames":
        second["header"]["frame_id"] = "map"
    elif issue == "empty_frame":
        second["child_frame_id"] = ""
    elif issue == "nan":
        second["pose"]["pose"]["position"]["x"] = float("nan")
    elif issue == "quaternion":
        second["pose"]["pose"]["orientation"]["w"] = 0
    else:
        second["header"]["stamp"]["nanosec"] = 1_000_000_000
    path, output = tmp_path / "bad.mcap", tmp_path / "out.json"
    write(path, [first, second])
    assert (
        main(["extract-odometry", str(path), "--topic", "/wheel/odom", "--output", str(output)])
        == 2
    )
    assert not output.exists()


def test_topic_schema_and_sample_budget_are_explicit(tmp_path):
    path = tmp_path / "odom.mcap"
    write(path, [message(), message()])
    with pytest.raises(ValueError, match="no recorded messages"):
        extract_odometry(path, "/missing")
    with pytest.raises(ValueError, match="sample budget"):
        extract_odometry(path, "/wheel/odom", max_samples=1)
    write(path, [message()], schema_name="custom_msgs/Estimate")
    with pytest.raises(ValueError, match="nav_msgs/Odometry"):
        extract_odometry(path, "/wheel/odom")
