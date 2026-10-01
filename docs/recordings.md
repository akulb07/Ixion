# MCAP recording inspection

The first real-data feature is a read-only container inventory. It works without
ROS installed and leaves message payloads opaque, including ROS 2 CDR messages.

```sh
python -m pip install -e ".[recordings]"
roboforge inspect-recording robot.mcap --output results/robot-inventory.json
```

The JSON includes the input SHA256, MCAP profile/writer, observed channel IDs,
topics, message/schema encodings, message counts and payload sizes. Timing fields
keep exact integer nanoseconds. Each channel reports earliest/latest log time,
backward log/publish timestamp transitions, consecutive repeated log timestamps,
zero publish timestamps and signed log-minus-publish offset ranges. Channels with
no messages are not listed in this first version.

Counters use file order, not timestamp-sorted order, so backward timestamps remain
visible. Reordering can come from recording/buffering behaviour and does not by
itself prove a sensor fault. The mean rate is `(count - 1) / (latest - earliest)`;
it is unavailable for zero duration and is not a jitter or packet-loss estimate.
Offsets are not latency measurements unless the recording's clocks are known to
be comparable. Zero publish timestamps are counted, not silently discarded.

The scanner uses the upstream MCAP reader with CRC checks enabled where CRCs are
present. It supports uncompressed, Zstd and LZ4 recordings. The input is hashed
before scanning; size and modification time are checked afterward to reject normal
concurrent edits. Work with a closed recording, not a live file being appended.

Limits: 2 GiB input file, 64 MiB encoded records, 4096 observed channels and one
million messages. `--max-messages` can reduce the message budget. Exceeding a budget
fails without publishing a partial report. These limits are not a strict memory
sandbox: compressed chunks can expand beyond their encoded size. Use trusted local
recordings. Outputs must be new and cannot replace the input. Exit 0 means the
inventory was produced; exit 2 means inspection or output failed. Timestamp warnings
are measurements in the report, not an acceptance verdict.

The inspector itself does not decode message values. The separate
[ROS 2 odometry extractor](recorded-odometry.md) reads a selected topic and its
header timestamps. IMU/LiDAR decoding, transforms, algorithm topic mapping and
workspace replay remain pending. Recorded poses are never assumed to be ground truth.

Reader behaviour: [MCAP Python reader documentation](https://mcap.dev/docs/python/mcap-apidoc/mcap.reader).
