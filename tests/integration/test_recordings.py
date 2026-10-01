import hashlib
import json

import pytest

pytest.importorskip("mcap")
from mcap.writer import CompressionType, Writer

from roboforge.cli import main
from roboforge.recordings import inspect_mcap


def recording(path, compression=CompressionType.NONE, chunking=True, messages=True):
    with path.open("wb") as stream:
        writer = Writer(
            stream, compression=compression, use_chunking=chunking, enable_data_crcs=True
        )
        writer.start(profile="ros2", library="fixture")
        schema = writer.register_schema("sensor_msgs/msg/Imu", "ros2msg", b"float64 test")
        channel = writer.register_channel("/imu", "cdr", schema)
        writer.register_channel("/empty", "cdr", schema)
        if messages:
            origin = 1_700_000_000_000_000_000
            for offset, publish in [(20, 0), (10, origin + 15), (10, origin + 5)]:
                writer.add_message(channel, origin + offset, b"opaque CDR payload", publish)
        writer.finish()


@pytest.mark.parametrize(
    "compression,chunking",
    [
        (CompressionType.NONE, False),
        (CompressionType.NONE, True),
        (CompressionType.ZSTD, True),
        (CompressionType.LZ4, True),
    ],
)
def test_inventory_preserves_nanoseconds_and_observed_order(tmp_path, compression, chunking):
    path = tmp_path / "robot.mcap"
    recording(path, compression, chunking)
    before = path.read_bytes()
    report = inspect_mcap(path)
    assert report["file_sha256"] == hashlib.sha256(before).hexdigest()
    assert path.read_bytes() == before
    assert report["messages"] == 3 and len(report["channels"]) == 1
    row = report["channels"][0]
    assert row["schema_name"] == "sensor_msgs/msg/Imu" and row["message_encoding"] == "cdr"
    assert row["first_log_time_ns"] == 1_700_000_000_000_000_010
    assert row["log_time_regressions"] == 1 and row["repeated_log_times"] == 1
    assert row["publish_time_regressions"] == 1 and row["zero_publish_times"] == 1
    assert row["negative_log_publish_offsets"] == 1 and row["min_log_publish_offset_ns"] == -5


def test_budget_and_cli_outputs_are_explicit(tmp_path):
    path = tmp_path / "robot.mcap"
    output = tmp_path / "report.json"
    recording(path)
    args = ["inspect-recording", str(path), "--output", str(output)]
    assert main(args + ["--max-messages", "2"]) == 2 and not output.exists()
    assert main(args) == 0
    assert json.loads(output.read_text())["messages"] == 3
    before = output.read_bytes()
    assert main(args) == 2 and output.read_bytes() == before
    assert main(["inspect-recording", str(path), "--output", str(path)]) == 2


def test_empty_and_truncated_recordings(tmp_path):
    path = tmp_path / "empty.mcap"
    recording(path, messages=False)
    assert inspect_mcap(path)["channels"] == []
    path.write_bytes(path.read_bytes()[:-12])
    with pytest.raises(ValueError):
        inspect_mcap(path)


def test_data_crc_corruption_rejected(tmp_path):
    path = tmp_path / "corrupt.mcap"
    recording(path, chunking=False)
    data = path.read_bytes().replace(b"opaque CDR payload", b"changedCDR payload", 1)
    path.write_bytes(data)
    with pytest.raises(ValueError, match="CRC|crc"):
        inspect_mcap(path)
