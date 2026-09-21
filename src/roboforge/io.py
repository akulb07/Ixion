"""Small versioned export for foundation demos, not the future replay engine."""

import csv
import json
import platform
from pathlib import Path

import numpy as np

from roboforge import __version__
from roboforge.simulation import SimulationResult


def save_result(result: SimulationResult, directory: str | Path) -> Path:
    """Save validated configuration, numerical trajectory, and runtime versions."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "config.json").write_text(
        result.config.model_dump_json(indent=2), encoding="utf-8"
    )
    metadata = {
        "format_version": 1,
        "roboforge_version": __version__,
        "model_version": "ideal-differential-drive-v1",
        "python_version": platform.python_version(),
        "numpy_version": np.__version__,
        "seed": result.config.seed,
        "stochastic_components": [],
        "integrator": result.config.simulation.integrator,
        "dt_s": result.config.simulation.dt,
        "steps": len(result.states) - 1,
        "duration_s": result.states[-1].time,
        "collision_enabled": False,
    }
    (directory / "metadata.json").write_text(
        json.dumps(metadata, indent=2, allow_nan=False), encoding="utf-8"
    )
    with (directory / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "x_m",
                "y_m",
                "theta_rad",
                "vx_m_s",
                "vy_m_s",
                "omega_rad_s",
                "left_rad_s",
                "right_rad_s",
            ]
        )
        for state in result.states:
            writer.writerow(
                [
                    state.time,
                    state.pose.x,
                    state.pose.y,
                    state.pose.theta,
                    state.vx,
                    state.vy,
                    state.omega,
                    state.wheels.left,
                    state.wheels.right,
                ]
            )
    return directory
