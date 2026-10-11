"""Brake a shaft from echo measurements in a scripted target fixture."""

import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.ultrasonic import UltrasonicConfig
from roboforge.hardware.ultrasonic_feedback import (
    TargetDistance,
    UltrasonicFeedbackConfig,
    run_ultrasonic_feedback,
)


def main():
    project = load_project(Path(__file__).with_name("robot.yaml"))
    config = UltrasonicFeedbackConfig(
        inertia_kg_m2=0.01,
        sensor=UltrasonicConfig(response_delay_s=0.0002),
        targets=(
            TargetDistance(time_s=0, distance_m=0.5),
            TargetDistance(time_s=0.2, distance_m=0.1),
        ),
    )
    print(json.dumps(run_ultrasonic_feedback(project, config), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
