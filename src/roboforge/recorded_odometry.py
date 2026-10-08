"""Extract recorded ROS 2 odometry estimates without synthesizing ground truth."""

import math
from pathlib import Path

from roboforge.recordings import inspect_mcap


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("odometry measurements must be finite numbers")
    return value


def _vector(value, fields):
    return {name: _number(getattr(value, name)) for name in fields}


def _covariance(value):
    if len(value) != 36:
        raise ValueError("odometry covariance must contain 36 values")
    return [_number(item) for item in value]


def extract_odometry(path, topic, *, max_samples=100_000):
    if not isinstance(topic, str) or not topic or len(topic) > 1024:
        raise ValueError("select an exact nonempty odometry topic")
    if (
        isinstance(max_samples, bool)
        or not isinstance(max_samples, int)
        or not 1 <= max_samples <= 100_000
    ):
        raise ValueError("sample budget must be between 1 and 100,000")
    try:
        from mcap.reader import NonSeekingReader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError as exc:
        raise ValueError(
            'Install ROS 2 decoding with: pip install "ixion[ros2-recordings]"'
        ) from exc
    path = Path(path)
    before = path.stat()
    inventory = inspect_mcap(path)
    selected = [row for row in inventory["channels"] if row["topic"] == topic]
    if not selected:
        raise ValueError("selected topic has no recorded messages")
    if any(
        row["schema_name"] not in {"nav_msgs/msg/Odometry", "nav_msgs/Odometry"}
        or row["message_encoding"] != "cdr"
        or row["schema_encoding"] != "ros2msg"
        for row in selected
    ):
        raise ValueError("selected topic must contain CDR nav_msgs/Odometry with ros2msg schemas")
    if sum(row["messages"] for row in selected) > max_samples:
        raise ValueError("odometry topic exceeds sample budget; no partial trajectory written")
    factory = DecoderFactory()
    samples, frames = [], None
    previous_stamp, regressions, repeats = None, 0, 0
    try:
        with path.open("rb") as stream:
            reader = NonSeekingReader(stream, validate_crcs=True, record_size_limit=64 * 1024**2)
            for index, (schema, channel, message) in enumerate(
                reader.iter_messages(log_time_order=False)
            ):
                if index >= 1_000_000:
                    raise ValueError("recording changed or exceeds scan budget")
                if channel.topic != topic:
                    continue
                if len(samples) >= max_samples:
                    raise ValueError("odometry sample budget exceeded")
                if schema is None or len(schema.data) > 256 * 1024 or len(message.data) > 1024**2:
                    raise ValueError("odometry schema or payload exceeds decoder budget")
                decoder = factory.decoder_for(channel.message_encoding, schema)
                if decoder is None:
                    raise ValueError("odometry message encoding cannot be decoded")
                msg = decoder(message.data)
                pair = (msg.header.frame_id, msg.child_frame_id)
                if not all(isinstance(frame, str) and frame for frame in pair):
                    raise ValueError("odometry requires nonempty parent and child frame IDs")
                if frames is not None and pair != frames:
                    raise ValueError(
                        "odometry frame IDs change within the topic; split or transform the recording first"
                    )
                frames = pair
                sec, ns = msg.header.stamp.sec, msg.header.stamp.nanosec
                if type(sec) is not int or type(ns) is not int or not 0 <= ns < 1_000_000_000:
                    raise ValueError("invalid odometry header timestamp")
                stamp = sec * 1_000_000_000 + ns
                regressions += previous_stamp is not None and stamp < previous_stamp
                repeats += previous_stamp is not None and stamp == previous_stamp
                previous_stamp = stamp
                orientation = _vector(msg.pose.pose.orientation, ("x", "y", "z", "w"))
                norm = math.hypot(*orientation.values())
                if abs(norm - 1) > 0.001:
                    raise ValueError("odometry quaternion must be unit length within 0.001")
                samples.append(
                    {
                        "channel_id": channel.id,
                        "sequence": message.sequence,
                        "log_time_ns": message.log_time,
                        "publish_time_ns": message.publish_time,
                        "header_time_ns": stamp,
                        "position_m": _vector(msg.pose.pose.position, ("x", "y", "z")),
                        "orientation_xyzw": orientation,
                        "pose_covariance": _covariance(msg.pose.covariance),
                        "linear_velocity_m_s": _vector(msg.twist.twist.linear, ("x", "y", "z")),
                        "angular_velocity_rad_s": _vector(msg.twist.twist.angular, ("x", "y", "z")),
                        "twist_covariance": _covariance(msg.twist.covariance),
                    }
                )
    except (OSError, ValueError):
        raise
    except Exception as exc:
        raise ValueError(f"Cannot decode odometry: {type(exc).__name__}: {exc}") from exc
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError("recording changed during extraction")
    if not samples:
        raise ValueError("selected topic has no decodable odometry")
    return {
        "format_version": 1,
        "kind": "recorded_odometry_estimate",
        "source_sha256": inventory["file_sha256"],
        "topic": topic,
        "pose_frame_id": frames[0],
        "twist_frame_id": frames[1],
        "sample_count": len(samples),
        "header_time_regressions": regressions,
        "repeated_header_times": repeats,
        "samples": samples,
        "notes": [
            "Recorded estimates only; no ground truth or accuracy score is inferred.",
            "Pose is in header.frame_id; twist is in child_frame_id. No TF transforms applied.",
            "Samples remain in file order; all three timestamp fields retain integer nanoseconds.",
            "Covariances are preserved, not validated as calibrated uncertainty.",
        ],
    }
