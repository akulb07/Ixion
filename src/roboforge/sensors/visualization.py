"""Optional plots of actual measurement records, never fabricated sensor signals."""

from pathlib import Path

import numpy as np


def plot_sensors(readings, path: str | Path) -> Path:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    figure = Figure(figsize=(11, 7), layout="constrained")
    FigureCanvasAgg(figure)
    encoder_ax = figure.add_subplot(221)
    gyro_ax = figure.add_subplot(222)
    lidar_ax = figure.add_subplot(223, projection="polar")
    acceleration_ax = figure.add_subplot(224)
    encoders = [r for r in readings if r.kind == "encoder"]
    imus = [r for r in readings if r.kind == "imu"]
    lidars = [r for r in readings if r.kind == "lidar"]
    for label in ("left_ticks", "right_ticks"):
        encoder_ax.plot(
            [r.capture_time for r in encoders],
            [getattr(r, label) if getattr(r, label) is not None else np.nan for r in encoders],
            label=label.removesuffix("_ticks"),
        )
    encoder_ax.set(
        title="Encoder measurements", xlabel="Capture time (s)", ylabel="Cumulative ticks"
    )
    encoder_ax.legend()
    gyro_ax.plot([r.capture_time for r in imus], [r.gyro_z for r in imus], color="#7c3aed")
    gyro_ax.set(title="IMU yaw rate", xlabel="Capture time (s)", ylabel="rad/s")
    if lidars:
        scan = lidars[-1]
        lidar_ax.scatter(
            scan.angles, [r if r is not None else np.nan for r in scan.ranges], s=5, color="#087f8c"
        )
        invalid = sum(r is None for r in scan.ranges)
        lidar_ax.set_title(f"Last LiDAR scan · {invalid}/{len(scan.ranges)} invalid", pad=18)
    for label in ("acceleration_x", "acceleration_y"):
        acceleration_ax.plot(
            [r.capture_time for r in imus], [getattr(r, label) for r in imus], label=label[-1]
        )
    acceleration_ax.set(title="IMU planar acceleration", xlabel="Capture time (s)", ylabel="m/s²")
    acceleration_ax.legend()
    for axis in (encoder_ax, gyro_ax, acceleration_ax):
        axis.grid(alpha=0.2)
    figure.suptitle("Ixion | Seeded sensor laboratory", fontsize=16)
    figure.supxlabel(
        "Actual measurement records · capture and delivery times saved separately", fontsize=9
    )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150)
    return path
