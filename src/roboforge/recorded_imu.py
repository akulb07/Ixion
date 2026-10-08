"""Recorded IMU extraction with ROS availability and covariance semantics."""

import math
from pathlib import Path

from roboforge.recordings import inspect_mcap


def _measurement(msg, name, fields):
    covariance = list(getattr(msg, name + "_covariance"))
    if len(covariance) != 9 or any(not math.isfinite(x) for x in covariance):
        raise ValueError("IMU covariance must have nine finite entries")
    if covariance[0] == -1:
        # The associated estimate is explicitly invalid in ROS, including any
        # placeholder zero quaternion or NaNs. Do not promote it to a measurement.
        return {
            "value": None,
            "availability": "unavailable",
            "covariance": covariance,
            "covariance_status": "unavailable",
        }
    value = {field: getattr(getattr(msg, name), field) for field in fields}
    if any(not math.isfinite(x) for x in value.values()):
        raise ValueError("available IMU measurements must be finite")
    if name == "orientation" and abs(math.hypot(*value.values()) - 1) > 0.001:
        raise ValueError("available IMU orientation must be a unit quaternion within 0.001")
    return {
        "value": value,
        "availability": "available",
        "covariance": covariance,
        "covariance_status": "unknown" if all(x == 0 for x in covariance) else "provided",
    }


def extract_imu(path, topic, *, max_samples=100_000):
    if not isinstance(topic, str) or not topic or len(topic) > 1024:
        raise ValueError("select an exact nonempty IMU topic")
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
        row["schema_name"] not in {"sensor_msgs/msg/Imu", "sensor_msgs/Imu"}
        or row["message_encoding"] != "cdr"
        or row["schema_encoding"] != "ros2msg"
        for row in selected
    ):
        raise ValueError("selected topic must contain CDR sensor_msgs/Imu with ros2msg schemas")
    if sum(row["messages"] for row in selected) > max_samples:
        raise ValueError("IMU topic exceeds sample budget")
    factory = DecoderFactory()
    samples, frame = [], None
    previous_stamp, regressions, repeats = None, 0, 0
    unavailable = {name: 0 for name in ("orientation", "angular_velocity", "linear_acceleration")}
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
                    raise ValueError("IMU sample budget exceeded")
                if schema is None or len(schema.data) > 256 * 1024 or len(message.data) > 1024**2:
                    raise ValueError("IMU schema or payload exceeds decoder budget")
                decoder = factory.decoder_for(channel.message_encoding, schema)
                if decoder is None:
                    raise ValueError("IMU encoding cannot be decoded")
                msg = decoder(message.data)
                if not isinstance(msg.header.frame_id, str) or not msg.header.frame_id:
                    raise ValueError("IMU frame ID must be nonempty")
                if frame is not None and frame != msg.header.frame_id:
                    raise ValueError("IMU frame ID changes within topic; split or transform first")
                frame = msg.header.frame_id
                sec, ns = msg.header.stamp.sec, msg.header.stamp.nanosec
                if type(sec) is not int or type(ns) is not int or not 0 <= ns < 1_000_000_000:
                    raise ValueError("invalid IMU header timestamp")
                stamp = sec * 1_000_000_000 + ns
                regressions += previous_stamp is not None and stamp < previous_stamp
                repeats += previous_stamp is not None and stamp == previous_stamp
                previous_stamp = stamp
                measurements = {
                    name: _measurement(
                        msg,
                        name,
                        ("x", "y", "z", "w") if name == "orientation" else ("x", "y", "z"),
                    )
                    for name in unavailable
                }
                for name, measurement in measurements.items():
                    unavailable[name] += measurement["availability"] == "unavailable"
                samples.append(
                    {
                        "channel_id": channel.id,
                        "sequence": message.sequence,
                        "header_time_ns": stamp,
                        "log_time_ns": message.log_time,
                        "publish_time_ns": message.publish_time,
                        **measurements,
                    }
                )
    except (OSError, ValueError):
        raise
    except Exception as exc:
        raise ValueError(f"Cannot decode IMU: {type(exc).__name__}: {exc}") from exc
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError("recording changed during extraction")
    if not samples:
        raise ValueError("selected topic has no decodable IMU samples")
    return {
        "format_version": 1,
        "kind": "recorded_imu_measurements",
        "source_sha256": inventory["file_sha256"],
        "topic": topic,
        "frame_id": frame,
        "sample_count": len(samples),
        "unavailable_counts": unavailable,
        "header_time_regressions": regressions,
        "repeated_header_times": repeats,
        "units": {
            "orientation": "quaternion_xyzw",
            "angular_velocity": "rad/s",
            "linear_acceleration": "m/s^2",
        },
        "samples": samples,
        "notes": [
            "No TF transforms, bias correction, gravity removal or sensor fusion applied.",
            "Covariance[0] == -1 makes the estimate unavailable; all-zero covariance means unknown uncertainty.",
            "Provided covariance is preserved, not certified as calibrated or positive semidefinite.",
            "File order and integer nanosecond timestamps preserved; no clock alignment inferred.",
        ],
    }
