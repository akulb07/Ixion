"""Optional static scientific plot; consumes simulation results without changing them."""

from pathlib import Path

import numpy as np

from roboforge.config import Circle
from roboforge.simulation import SimulationResult
from roboforge.trajectory import sample_trajectory


def plot_trajectory(result: SimulationResult, path: str | Path) -> Path:
    """Plot world geometry, trajectory, orientation arrows, and heading history."""
    try:
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        from matplotlib.patches import Circle as CirclePatch
        from matplotlib.patches import Rectangle as RectanglePatch
    except ImportError as exc:
        raise ValueError(
            'Plotting requires the optional dependency: pip install ".[plot]"'
        ) from exc
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure = Figure(figsize=(11, 5.3), layout="constrained")
    FigureCanvasAgg(figure)
    world, heading = figure.subplots(1, 2, gridspec_kw={"width_ratios": [1.2, 1]})
    environment = result.config.environment
    for obstacle in environment.obstacles:
        if isinstance(obstacle, Circle):
            patch = CirclePatch(
                (obstacle.x, obstacle.y), obstacle.radius, color="#94a3b8", alpha=0.6
            )
        else:
            patch = RectanglePatch(
                (obstacle.x, obstacle.y),
                obstacle.width,
                obstacle.height,
                color="#94a3b8",
                alpha=0.6,
            )
        world.add_patch(patch)
    data = np.array(
        [[time, pose.x, pose.y, pose.theta] for time, pose in sample_trajectory(result)]
    )
    world.plot(
        data[:, 1], data[:, 2], color="#087f8c", linewidth=2, label="Kinematic path (resampled)"
    )
    sample = data[np.unique(np.linspace(0, len(data) - 1, min(12, len(data))).astype(int))]
    world.quiver(
        sample[:, 1],
        sample[:, 2],
        np.cos(sample[:, 3]),
        np.sin(sample[:, 3]),
        angles="xy",
        scale_units="xy",
        scale=3,
        color="#334155",
        width=0.006,
    )
    world.scatter(data[0, 1], data[0, 2], color="#16a34a", s=60, label="Start", zorder=5)
    world.scatter(
        data[-1, 1], data[-1, 2], color="#dc2626", s=60, marker="x", label="End", zorder=6
    )
    last = result.states[-1]
    for event in result.collisions:
        point = event.report.nearest.point_on_obstacle
        world.scatter(
            point.x,
            point.y,
            marker="*",
            s=150,
            color="#d97706",
            zorder=8,
            label="Contact candidate",
        )
    world.quiver(
        last.pose.x,
        last.pose.y,
        np.cos(last.pose.theta),
        np.sin(last.pose.theta),
        angles="xy",
        scale_units="xy",
        scale=3,
        color="#dc2626",
        width=0.008,
        zorder=7,
    )
    world.add_patch(
        CirclePatch(
            (last.pose.x, last.pose.y),
            result.config.robot.footprint_radius,
            fill=False,
            edgecolor="#dc2626",
            linewidth=1.5,
        )
    )
    # Include out-of-bounds trajectories in disabled mode as well.
    world.set(
        xlim=(min(0, data[:, 1].min() - 0.3), max(environment.width, data[:, 1].max() + 0.3)),
        ylim=(min(0, data[:, 2].min() - 0.3), max(environment.height, data[:, 2].max() + 0.3)),
        xlabel="World x (m)",
        ylabel="World y (m)",
        title="Position and body heading",
        aspect="equal",
    )
    world.legend(loc="upper right", fontsize=8)
    # Segment resampling limits angular increments, making unwrapping unambiguous.
    heading.plot(data[:, 0], np.unwrap(data[:, 3]), color="#7c3aed", linewidth=2)
    heading.set(xlabel="Simulation time (s)", ylabel="Unwrapped heading (rad)", title="Orientation")
    for axis in (world, heading):
        axis.grid(alpha=0.2)
    figure.suptitle(f"Ixion | {result.config.name}", fontsize=15, fontweight="bold")
    figure.supxlabel(
        f"Ideal kinematics · collision mode: {result.config.simulation.collision.mode} · status: {result.status}",
        fontsize=9,
    )
    figure.savefig(path, dpi=150)
    return path
