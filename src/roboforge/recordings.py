"""Read-only MCAP inventory. Container timestamps are not sensor capture times."""

import hashlib
from pathlib import Path


def inspect_mcap(path, *, max_messages=1_000_000):
    if (
        isinstance(max_messages, bool)
        or not isinstance(max_messages, int)
        or not 1 <= max_messages <= 1_000_000
    ):
        raise ValueError("message budget must be between 1 and 1,000,000")
    try:
        from mcap.reader import NonSeekingReader
    except ImportError as exc:
        raise ValueError(
            'Install recording support with: pip install "ixion[recordings]"'
        ) from exc
    path = Path(path)
    before = path.stat()
    if before.st_size > 2 * 1024**3:
        raise ValueError("recording exceeds the current 2 GiB file limit")
    channels, count = {}, 0
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
            stream.seek(0)
            header = NonSeekingReader(stream, record_size_limit=64 * 1024**2).get_header()
            stream.seek(0)
            reader = NonSeekingReader(stream, validate_crcs=True, record_size_limit=64 * 1024**2)
            for schema, channel, message in reader.iter_messages(log_time_order=False):
                count += 1
                if count > max_messages:
                    raise ValueError(
                        "recording exceeds the message budget; no partial report written"
                    )
                if channel.id not in channels:
                    if len(channels) >= 4096:
                        raise ValueError("recording exceeds 4096 observed channels")
                    channels[channel.id] = {
                        "channel_id": channel.id,
                        "topic": channel.topic,
                        "message_encoding": channel.message_encoding,
                        "schema_name": schema.name if schema else None,
                        "schema_encoding": schema.encoding if schema else None,
                        "messages": 0,
                        "payload_bytes": 0,
                        "first_log_time_ns": message.log_time,
                        "last_log_time_ns": message.log_time,
                        "log_time_regressions": 0,
                        "repeated_log_times": 0,
                        "publish_time_regressions": 0,
                        "zero_publish_times": 0,
                        "negative_log_publish_offsets": 0,
                        "min_log_publish_offset_ns": None,
                        "max_log_publish_offset_ns": None,
                        "_previous_log": None,
                        "_previous_publish": None,
                    }
                row = channels[channel.id]
                for field, current, counter in [
                    ("_previous_log", message.log_time, "log_time_regressions"),
                    ("_previous_publish", message.publish_time, "publish_time_regressions"),
                ]:
                    previous = row[field]
                    if previous is not None and current < previous:
                        row[counter] += 1
                    row[field] = current
                if row.get("_last_log") == message.log_time:
                    row["repeated_log_times"] += 1
                row["_last_log"] = message.log_time
                row["messages"] += 1
                row["payload_bytes"] += len(message.data)
                row["first_log_time_ns"] = min(row["first_log_time_ns"], message.log_time)
                row["last_log_time_ns"] = max(row["last_log_time_ns"], message.log_time)
                row["zero_publish_times"] += message.publish_time == 0
                offset = message.log_time - message.publish_time
                row["negative_log_publish_offsets"] += offset < 0
                for key, operation in [
                    ("min_log_publish_offset_ns", min),
                    ("max_log_publish_offset_ns", max),
                ]:
                    row[key] = offset if row[key] is None else operation(row[key], offset)
    except (OSError, ValueError):
        raise
    except Exception as exc:
        raise ValueError(f"Cannot inspect MCAP: {type(exc).__name__}: {exc}") from exc
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError("recording changed during inspection")
    rows = []
    for row in channels.values():
        duration = row["last_log_time_ns"] - row["first_log_time_ns"]
        row["mean_log_rate_hz"] = (row["messages"] - 1) * 1e9 / duration if duration > 0 else None
        rows.append({key: value for key, value in row.items() if not key.startswith("_")})
    return {
        "format_version": 1,
        "kind": "mcap_inventory",
        "file_sha256": digest.hexdigest(),
        "file_bytes": before.st_size,
        "profile": header.profile,
        "writer": header.library,
        "messages": count,
        "channels": sorted(rows, key=lambda row: row["channel_id"]),
        "notes": [
            "Only channels with messages are listed; payloads have not been decoded.",
            "Timestamp counters use file order per channel; nanoseconds are retained as integers.",
            "Log-minus-publish offsets are not measured latency without a shared clock contract.",
            "No ground truth, sensor header timestamps or coordinate transforms are inferred.",
            "CRCs are checked where present; missing CRCs do not establish integrity.",
        ],
    }
