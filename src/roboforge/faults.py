"""Explicit timed faults with independent named random streams."""

from collections import deque
from dataclasses import dataclass

from roboforge.config import RunConfig
from roboforge.robotics import WheelSpeeds
from roboforge.sensors.noise import generator
from roboforge.sensors.readings import EncoderReading, LidarReading


@dataclass(frozen=True, slots=True)
class FaultEvent:
    time: float
    name: str
    kind: str
    target: str
    active: bool


class FaultEngine:
    def __init__(self, config: RunConfig):
        self.faults = tuple(sorted(config.faults, key=lambda fault: fault.name))
        self.seed = config.seed
        self._generators = {}
        self._commands = deque(maxlen=1 + sum(f.delay_steps for f in self.faults))

    def active(self, time):
        return (f for f in self.faults if f.start <= time and (f.end is None or time < f.end))

    def events(self, until_time):
        events = []
        for fault in self.faults:
            for time, active in ((fault.start, True), (fault.end, False)):
                if time is not None and time <= until_time:
                    events.append(FaultEvent(time, fault.name, fault.kind, fault.target, active))
        return tuple(sorted(events, key=lambda event: (event.time, event.name, event.active)))

    def command(self, wheels: WheelSpeeds, time: float) -> WheelSpeeds:
        self._commands.append(wheels)
        delay = sum(f.delay_steps for f in self.active(time) if f.kind == "actuator_delay")
        return self._commands[-delay - 1] if delay < len(self._commands) else WheelSpeeds(0, 0)

    def wheels(self, wheels: WheelSpeeds, time: float, kind: str) -> WheelSpeeds:
        values = [wheels.left, wheels.right]
        for fault in self.active(time):
            if fault.kind != kind:
                continue
            for index, side in enumerate(("left", "right")):
                if fault.target in (side, "both"):
                    if kind == "wheel_slip":
                        values[index] *= 1 - fault.magnitude
                    elif kind == "actuator_saturation":
                        values[index] = max(-fault.magnitude, min(fault.magnitude, values[index]))
        return WheelSpeeds(*values)

    def _rng(self, name, channel):
        key = (name, channel)
        if key not in self._generators:
            self._generators[key] = generator(self.seed, f"fault:{name}:{channel}")
        return self._generators[key]

    def reading(self, reading):
        values = reading.model_dump()
        for fault in self.active(reading.capture_time):
            if fault.target != reading.sensor:
                continue
            if fault.kind == "encoder_scale":
                for key in ("left_ticks", "right_ticks"):
                    if values[key] is not None:
                        values[key] = round(values[key] * (1 + fault.magnitude))
            elif fault.kind in ("gyro_bias", "gyro_drift"):
                if values["gyro_z"] is not None:
                    values["gyro_z"] += fault.magnitude * (
                        reading.capture_time - fault.start if fault.kind == "gyro_drift" else 1
                    )
            elif fault.kind == "lidar_noise":
                ranges, hits = list(values["ranges"]), list(values["hits"])
                for i, distance in enumerate(ranges):
                    if distance is not None and hits[i]:
                        ranges[i] = distance + float(
                            self._rng(fault.name, str(i)).normal(0, fault.magnitude)
                        )
                        if not 0 <= ranges[i] <= reading.max_range:
                            ranges[i], hits[i] = None, False
                values["ranges"], values["hits"] = ranges, hits
            elif fault.kind == "sensor_dropout":
                if isinstance(reading, LidarReading):
                    ranges, hits = list(values["ranges"]), list(values["hits"])
                    for i in range(len(ranges)):
                        if self._rng(fault.name, str(i)).random() < fault.magnitude:
                            ranges[i], hits[i] = None, False
                    values["ranges"], values["hits"] = ranges, hits
                elif self._rng(fault.name, "packet").random() < fault.magnitude:
                    keys = (
                        ("left_ticks", "right_ticks")
                        if isinstance(reading, EncoderReading)
                        else ("gyro_z", "acceleration_x", "acceleration_y")
                    )
                    for key in keys:
                        values[key] = None
        return type(reading).model_validate(values)
