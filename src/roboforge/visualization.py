"""Optional static scientific plot; consumes simulation results without changing them."""

from pathlib import Path

import numpy as np

from roboforge.config import Circle
from roboforge.simulation import SimulationResult


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
        [[state.time, state.pose.x, state.pose.y, state.pose.theta] for state in result.states]
    )
    world.plot(
        data[:, 1], data[:, 2], color="#087f8c", linewidth=2, label="Ground-truth trajectory"
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
    # Include out-of-bounds trajectories; do not conceal the absence of collision.
    world.set(
        xlim=(min(0, data[:, 1].min() - 0.3), max(environment.width, data[:, 1].max() + 0.3)),
        ylim=(min(0, data[:, 2].min() - 0.3), max(environment.height, data[:, 2].max() + 0.3)),
        xlabel="World x (m)",
        ylabel="World y (m)",
        title="Position and body heading",
        aspect="equal",
    )
    world.legend(loc="upper right", fontsize=8)
    # Wrapped samples alone cannot disambiguate rotations greater than pi/step.
    # Show canonical values for those runs instead of silently aliasing rotation.
    max_rotation = max(
        (abs(state.omega) * result.config.simulation.dt for state in result.states[1:]), default=0.0
    )
    continuous = max_rotation < np.pi
    angles = np.unwrap(data[:, 3]) if continuous else data[:, 3]
    angle_label = "Unwrapped heading (rad)" if continuous else "Wrapped heading (rad)"
    heading.plot(data[:, 0], angles, color="#7c3aed", linewidth=2)
    heading.set(xlabel="Simulation time (s)", ylabel=angle_label, title="Orientation")
    for axis in (world, heading):
        axis.grid(alpha=0.2)
    figure.suptitle(f"RoboForge | {result.config.name}", fontsize=15, fontweight="bold")
    figure.supxlabel(
        "Ideal kinematics · prescribed wheel commands · geometry only, collision disabled",
        fontsize=9,
    )
    figure.savefig(path, dpi=150)
    return path
