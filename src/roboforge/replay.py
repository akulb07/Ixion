"""Read-only replay of exported states, motion segments and sensor delivery."""

import bisect
import copy
import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

from pydantic import TypeAdapter

from roboforge.config import RunConfig
from roboforge.core import finite
from roboforge.geometry import Pose2
from roboforge.physics import KinematicMotion
from roboforge.robot import RobotState
from roboforge.robotics import BodyTwist2, DifferentialDrive, WheelSpeeds
from roboforge.sensors.readings import SensorReading


@dataclass(frozen=True, slots=True)
class ReplayFrame:
    state: RobotState
    readings: tuple[SensorReading, ...]
    control: dict | None
    actuator: dict | None
    navigation: dict | None = None


class ReplayLog:
    """Snapshot replay does not create a Simulator or execute controller algorithms."""

    def __init__(self, directory: str | Path):
        try:
            self._load(directory)
        except (KeyError, TypeError, IndexError, ValueError) as exc:
            raise ValueError(f"Invalid replay {directory}: {exc}") from exc

    def _load(self, directory: str | Path):
        directory = Path(directory).resolve()
        required = {
            "config.json",
            "metadata.json",
            "trajectory.csv",
            "motion_segments.json",
            "sensors.jsonl",
        }
        manifest_path = directory / "manifest.json"
        if manifest_path.exists():
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest.get("format_version") != 1 or not required.issubset(manifest["files"]):
                raise ValueError(
                    "replay manifest has an unsupported format or missing required files"
                )
            for name, expected in manifest["files"].items():
                path = (directory / name).resolve()
                if path.parent != directory or Path(name).name != name:
                    raise ValueError("manifest must contain only local filenames")
                if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                    raise ValueError(f"replay integrity check failed: {name}")
        self.config = RunConfig.model_validate_json(
            (directory / "config.json").read_text(encoding="utf-8")
        )
        metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
        if metadata.get("format_version") != 2:
            raise ValueError("unsupported replay format")
        drive = DifferentialDrive(
            self.config.robot.wheel_radius, self.config.robot.wheel_separation
        )
        states = []
        with (directory / "trajectory.csv").open(encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                wheels = WheelSpeeds(float(row["left_rad_s"]), float(row["right_rad_s"]))
                states.append(
                    RobotState(
                        Pose2(float(row["x_m"]), float(row["y_m"]), float(row["theta_rad"])),
                        wheels,
                        BodyTwist2(
                            float(row["body_linear_m_s"])
                            if "body_linear_m_s" in row
                            else drive.forward(wheels).linear,
                            float(row["omega_rad_s"]),
                        ),
                        float(row["time_s"]),
                    )
                )
        if (
            not states
            or states[0].time != 0
            or any(b.time <= a.time for a, b in zip(states, states[1:]))
        ):
            raise ValueError("replay state times must start at zero and strictly increase")
        motions = []
        for index, record in enumerate(
            json.loads((directory / "motion_segments.json").read_text(encoding="utf-8"))
        ):
            motion = KinematicMotion(
                Pose2(**record["start"]),
                DifferentialDrive(**record["drive"]),
                WheelSpeeds(**record["wheels"]),
                record["dt"],
                record["method"],
                WheelSpeeds(**record["encoder_wheels"])
                if record.get("encoder_wheels") is not None
                else None,
            )
            if (
                index + 1 >= len(states)
                or not math.isclose(record["start_time"], states[index].time, abs_tol=1e-12)
                or not math.isclose(
                    record["start_time"] + motion.dt, states[index + 1].time, abs_tol=1e-12
                )
            ):
                raise ValueError("replay motion intervals do not match states")
            if motion.start != states[index].pose:
                raise ValueError("replay motion start differs from recorded state")
            end = motion.pose_at(1)
            expected = states[index + 1].pose
            from roboforge.core import wrap_angle

            if (
                math.hypot(end.x - expected.x, end.y - expected.y) > 1e-9
                or abs(wrap_angle(end.theta - expected.theta)) > 1e-9
            ):
                raise ValueError("replay motion endpoint differs from recorded state")
            motions.append(motion)
        if len(motions) != len(states) - 1:
            raise ValueError("replay must have one motion per state interval")
        self.states, self.motions = tuple(states), tuple(motions)
        self._times = tuple(s.time for s in states)
        adapter = TypeAdapter(SensorReading)
        self.readings = tuple(
            adapter.validate_json(line)
            for line in (directory / "sensors.jsonl").read_text(encoding="utf-8").splitlines()
            if line.strip()
        )
        self._control = (
            json.loads((directory / "control.json").read_text(encoding="utf-8"))
            if (directory / "control.json").exists()
            else []
        )
        self._actuators = (
            json.loads((directory / "actuators.json").read_text(encoding="utf-8"))
            if (directory / "actuators.json").exists()
            else []
        )

        self._navigation = (
            json.loads((directory / "navigation.json").read_text(encoding="utf-8"))["samples"]
            if (directory / "navigation.json").exists()
            else []
        )

    @property
    def control_samples(self):
        """Isolated saved controller telemetry for offline analysis."""
        return copy.deepcopy(self._control)

    def state_at(self, time: float) -> RobotState:
        """Interpolate truth without scanning sensor or controller histories."""
        time = finite(time, "replay time")
        if not 0 <= time <= self.states[-1].time:
            raise ValueError("replay time is outside the recorded interval")
        index = bisect.bisect_right(self._times, time) - 1
        state = self.states[index]
        if time != state.time:
            motion = self.motions[index]
            fraction = (time - state.time) / motion.dt
            state = RobotState(
                motion.pose_at(fraction),
                motion.encoder_wheels or motion.wheels,
                motion.drive.forward(motion.wheels),
                time,
            )
        return state

    def at(self, time: float) -> ReplayFrame:
        state = self.state_at(time)
        time = state.time
        ready = tuple(
            sorted(
                (r for r in self.readings if r.delivery_time <= time),
                key=lambda r: (r.delivery_time, r.sensor, r.sequence),
            )
        )

        def latest(records):
            found = next((record for record in reversed(records) if record["time"] <= time), None)
            return json.loads(json.dumps(found)) if found is not None else None

        return ReplayFrame(
            state, ready, latest(self._control), latest(self._actuators), latest(self._navigation)
        )
