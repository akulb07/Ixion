"""Inspect recorded PID feedback, without replaying control against new measurements."""

import math

from roboforge import __version__
from roboforge.service import ServiceError


def analyze_control(replay, max_points=1000):
    if replay.config.wheel_controller is None:
        raise ServiceError("this run has no wheel feedback controller", 422)
    samples = replay.control_samples
    if not samples:
        raise ServiceError(
            "no recorded controller updates; two complete encoder samples are needed", 422
        )
    if len(samples) > 100000 or not 2 <= max_points <= 2000:
        raise ServiceError("controller analysis exceeds sample budget", 422)
    fields = (
        "setpoint",
        "measurement",
        "error",
        "proportional",
        "integral",
        "derivative",
        "feedforward",
        "unclamped",
        "output",
    )
    try:
        for i, sample in enumerate(samples):
            for key in ("time", "capture_time", "measurement_dt"):
                if not isinstance(sample[key], (float, int)) or not math.isfinite(sample[key]):
                    raise ValueError("nonfinite controller timestamp")
            if (
                not 0 <= sample["capture_time"] <= sample["time"] <= replay.states[-1].time + 1e-9
                or sample["measurement_dt"] <= 0
            ):
                raise ValueError("invalid controller time interval")
            if i and (
                sample["capture_time"] <= samples[i - 1]["capture_time"]
                or sample["time"] < samples[i - 1]["time"]
            ):
                raise ValueError("controller samples must increase in capture order")
            for wheel in ("left", "right"):
                values = sample[wheel]
                if any(
                    not isinstance(values[k], (float, int)) or not math.isfinite(values[k])
                    for k in fields
                ):
                    raise ValueError("nonfinite PID telemetry")
                if not isinstance(values["saturated"], bool):
                    raise ValueError("invalid saturation flag")
        metrics = {}
        for wheel in ("left", "right"):
            errors = [s[wheel]["setpoint"] - s[wheel]["measurement"] for s in samples]
            scale = max(map(abs, errors)) or 1
            metrics[wheel] = {
                "tracking_rmse_rad_s": scale
                * math.sqrt(math.fsum((e / scale) ** 2 for e in errors) / len(errors)),
                "mean_absolute_error_rad_s": math.fsum(abs(e) / len(errors) for e in errors),
                "peak_absolute_error_rad_s": max(map(abs, errors)),
                "final_error_rad_s": errors[-1],
                "saturated_update_percent": 100
                * sum(s[wheel]["saturated"] for s in samples)
                / len(samples),
                "peak_absolute_output_rad_s": max(abs(s[wheel]["output"]) for s in samples),
            }
            if not all(math.isfinite(v) for v in metrics[wheel].values()):
                raise ValueError("metrics exceed finite arithmetic")
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ServiceError(f"invalid controller telemetry: {exc}", 422) from exc
    count = min(max_points, len(samples))
    indices = (
        [round(i * (len(samples) - 1) / (count - 1)) for i in range(count)] if count > 1 else [0]
    )
    return {
        "format_version": 1,
        "software_version": __version__,
        "algorithm": "recorded_pid_response",
        "settings": replay.config.wheel_controller.model_dump(),
        "metrics": metrics,
        "total_updates": len(samples),
        "sampled": count < len(samples),
        "first_update_time": samples[0]["time"],
        "last_update_time": samples[-1]["time"],
        "run_end_time": replay.states[-1].time,
        "samples": [samples[i] for i in indices],
    }
