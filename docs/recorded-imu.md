# Recorded IMU extraction

This import feature is preserved alongside the hardware-prototyping work.
It reads a selected CDR `sensor_msgs/Imu` topic from a closed MCAP recording:

```sh
python -m pip install -e ".[ros2-recordings]"
ixion extract-imu robot.mcap --topic /imu/data --output results/imu.json
```

The JSON retains frame ID, exact header/log/publish nanosecond timestamps,
quaternion orientation, angular velocity in rad/s, acceleration in m/s² and all
three covariance arrays. File order is retained; backwards and repeated header
timestamps are counted. The source hash links the extraction to its recording.

Following the [ROS IMU definition](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg),
covariance element zero equal to -1 makes the associated value unavailable (`null`),
even if the payload contains placeholder values. An all-zero covariance is unknown
uncertainty, not zero error. Available values must be finite and available
orientation must have quaternion norm within 0.001 of one. Covariance must be
finite; provided covariance is not certified as calibrated or positive semidefinite.

Only `ros2msg` schemas are supported. Frame changes reject the extraction.
The default maximum is 100,000 samples, reducible with `--max-samples`. Schema
and message budgets are 256 KiB and 1 MiB; the container limits and compressed
chunk caveats from [recording inspection](recordings.md) also apply.
Outputs must be new and cannot overwrite the source. Exit 0 means exported;
exit 2 means extraction/output failed. No partial decoding report is published.

There is no TF transform, clock alignment, gravity removal, bias calibration,
sensor fusion or workspace replay in this extractor.
