# Extracting recorded ROS 2 odometry

The CLI can decode an explicitly selected `nav_msgs/msg/Odometry` topic from MCAP
without ROS installed. This produces a portable JSON trajectory of recorded
estimates, not a simulator run or ground-truth reference.

```sh
python -m pip install -e ".[ros2-recordings]"
roboforge inspect-recording robot.mcap --output results/inventory.json
roboforge extract-odometry robot.mcap --topic /wheel/odom --output results/odometry.json
```

Choose the exact topic from the inventory. Messages must use CDR encoding and an
embedded `ros2msg` schema for `nav_msgs/msg/Odometry` (the legacy spelling
`nav_msgs/Odometry` is also accepted). IDL schemas are not supported by the current
decoder. Other message types and unknown schemas are rejected.

The output preserves:

- Source recording SHA256, topic and channel IDs.
- Log, publish and header timestamps as exact integer nanoseconds.
- Pose frame (`header.frame_id`) and twist frame (`child_frame_id`).
- Position in metres, quaternion components, linear/angular velocities and both
  36-element covariance arrays.
- File order, sequence numbers, backward header-time transitions and consecutive
  duplicate header times.

No TF transforms, clock alignment, interpolation or coordinate conversion is
applied. Nonempty, consistent frame IDs are required throughout the selected
topic. Samples from different frames cannot silently become one trajectory.
Measurements must be finite and quaternion norm must be within 0.001 of one;
values are rejected rather than silently normalized. Covariances are retained but
are not asserted to be positive semidefinite or calibrated uncertainty.

The complete container is checked first using the inspector's limits and CRC
validation, then selected payloads are decoded. The file must remain unchanged.
There is a maximum of 100,000 decoded samples; `--max-samples` can lower this.
Schemas are limited to 256 KiB and selected payloads to 1 MiB. The recording's
compression limits still apply; this is for trusted local logs, not a hostile-file
sandbox. Oversized or invalid inputs do not publish partial trajectories.

Exit 0 means extraction succeeded; exit 2 means invalid input, missing optional
dependencies or an output error. The output must be new and cannot overwrite the
recording. No live robot connection, workspace replay, ground-truth error score,
IMU/LiDAR decoder or TF resolver is included yet.

References: [ROS odometry message definition](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg),
[MCAP ROS 2 decoder](https://mcap.dev/docs/python/mcap-ros2-apidoc/mcap_ros2.decoder).
