"""Three-state EKF and synchronized encoder/gyro fusion; no simulator dependencies."""

import math
from dataclasses import dataclass

import numpy as np

from roboforge.core import finite, nonnegative, positive, wrap_angle
from roboforge.geometry import Pose2, Vector2
from roboforge.sensors.readings import EncoderReading, ImuReading


def covariance(value, size: int, *, definite: bool = False) -> np.ndarray:
    """Validate covariance without silently repairing non-PSD inputs."""
    matrix = np.array(value, dtype=float, copy=True)
    if matrix.shape != (size, size) or not np.isfinite(matrix).all():
        raise ValueError(f"covariance must be a finite {size}x{size} matrix")
    if not np.allclose(matrix, matrix.T, rtol=0, atol=1e-12):
        raise ValueError("covariance must be symmetric")
    matrix = (matrix + matrix.T) / 2
    smallest = np.linalg.eigvalsh(matrix)[0]
    if smallest < -1e-12 or (definite and smallest <= 0):
        raise ValueError(
            "covariance must be positive definite" if definite else "covariance must be PSD"
        )
    return matrix


def motion_model(pose: Pose2, distance: float, yaw: float):
    """Exact SE(2) increment and analytic Jacobians F (pose), G (distance,yaw)."""
    distance, yaw = finite(distance, "distance increment"), finite(yaw, "yaw increment")
    z = yaw / 2
    if abs(z) < 1e-4:
        sinc = 1 - z * z / 6 + z**4 / 120
        dsinc_dyaw = -z / 6 + z**3 / 60 - z**5 / 1680
    else:
        sinc = math.sin(z) / z
        dsinc_dyaw = (z * math.cos(z) - math.sin(z)) / (2 * z * z)
    c, s = math.cos(pose.theta + z), math.sin(pose.theta + z)
    dx, dy = distance * sinc * c, distance * sinc * s
    updated = Pose2(pose.x + dx, pose.y + dy, pose.theta + yaw)
    f = np.array([[1, 0, -dy], [0, 1, dx], [0, 0, 1]], dtype=float)
    g = np.array(
        [
            [sinc * c, distance * (dsinc_dyaw * c - sinc * s / 2)],
            [sinc * s, distance * (dsinc_dyaw * s + sinc * c / 2)],
            [0, 1],
        ]
    )
    return updated, f, g


@dataclass(frozen=True, slots=True)
class PoseEstimate:
    time: float
    pose: Pose2
    covariance: tuple[tuple[float, ...], ...]


@dataclass(frozen=True, slots=True)
class Innovation:
    kind: str
    residual: tuple[float, ...]
    covariance: tuple[tuple[float, ...], ...]
    mahalanobis_squared: float
    accepted: bool


def _matrix_tuple(matrix):
    return tuple(tuple(float(v) for v in row) for row in matrix)


def landmark_model(pose: Pose2, landmark: Vector2):
    dx, dy = landmark.x - pose.x, landmark.y - pose.y
    squared = dx * dx + dy * dy
    if squared < 1e-16:
        raise ValueError("landmark model is undefined at the robot position")
    distance = math.sqrt(squared)
    expected = np.array([distance, wrap_angle(math.atan2(dy, dx) - pose.theta)])
    h = np.array([[-dx / distance, -dy / distance, 0], [dy / squared, -dx / squared, -1]])
    return expected, h


class PoseEKF:
    """Pose prior + uncertain body increments + optional known-landmark observations."""

    def __init__(self, pose: Pose2, initial_covariance, time: float = 0.0):
        self._pose = pose
        self._covariance = covariance(initial_covariance, 3)
        self.time = nonnegative(time, "estimate time")
        self.innovations: list[Innovation] = []

    @property
    def estimate(self) -> PoseEstimate:
        return PoseEstimate(self.time, self._pose, _matrix_tuple(self._covariance))

    def predict(
        self, distance: float, yaw: float, increment_covariance, time: float
    ) -> PoseEstimate:
        time = nonnegative(time, "prediction time")
        if time <= self.time:
            raise ValueError("prediction time must strictly increase")
        q = covariance(increment_covariance, 2)
        pose, f, g = motion_model(self._pose, distance, yaw)
        p = covariance(f @ self._covariance @ f.T + g @ q @ g.T, 3)
        self._pose, self._covariance, self.time = pose, p, time
        return self.estimate

    def correct_landmark(
        self,
        landmark: Vector2,
        measured_range: float,
        bearing: float,
        measurement_covariance,
        *,
        time: float,
        gate: float = 9.210340371976184,
    ) -> Innovation:
        """Known associated point landmark in world frame; bearing in base frame.

        Corrections must match current capture time. Delayed updates require an
        external replay/buffering layer; applying old data to current pose is rejected.
        """
        time = nonnegative(time, "measurement time")
        if time != self.time:
            raise ValueError("landmark capture time must match the current estimate")
        measured_range = positive(measured_range, "landmark range")
        bearing, gate = finite(bearing, "landmark bearing"), positive(gate, "innovation gate")
        r = covariance(measurement_covariance, 2, definite=True)
        expected, h = landmark_model(self._pose, landmark)
        residual = np.array([measured_range - expected[0], wrap_angle(bearing - expected[1])])
        s = covariance(h @ self._covariance @ h.T + r, 2, definite=True)
        nis = finite(float(residual @ np.linalg.solve(s, residual)), "innovation NIS")
        accepted = nis <= gate
        if accepted:
            k = np.linalg.solve(s, h @ self._covariance).T
            delta = k @ residual
            pose = Pose2(
                self._pose.x + delta[0], self._pose.y + delta[1], self._pose.theta + delta[2]
            )
            a = np.eye(3) - k @ h
            p = covariance(a @ self._covariance @ a.T + k @ r @ k.T, 3)
            self._pose, self._covariance = pose, p
        record = Innovation(
            "landmark", tuple(float(v) for v in residual), _matrix_tuple(s), nis, accepted
        )
        self.innovations.append(record)
        return record


class EncoderImuEKF:
    """Fuse synchronized interval-end encoders and gyro under constant-rate motion.

    Gyro is a yaw-rate observation of the motion increment, not absolute heading.
    This Gaussian increment update followed by nonlinear pose prediction preserves
    the encoder translation/rotation cross covariance. No artificial global fix.
    """

    def __init__(
        self,
        wheel_radius: float,
        wheel_separation: float,
        initial_pose: Pose2,
        initial_covariance,
        *,
        wheel_variance_per_m: float = 0.001,
        gyro_stddev: float = 0.01,
        gyro_gate: float = 25.0,
    ):
        self.radius = positive(wheel_radius, "wheel radius")
        self.separation = positive(wheel_separation, "wheel separation")
        self.wheel_variance_per_m = nonnegative(wheel_variance_per_m, "wheel variance per metre")
        self.gyro_stddev = positive(gyro_stddev, "gyro standard deviation")
        self.gate = positive(gyro_gate, "gyro gate")
        self.filter = PoseEKF(initial_pose, initial_covariance)
        self._previous: EncoderReading | None = None
        self._seen: EncoderReading | None = None
        self._imu_identity: tuple[str, str] | None = None
        self.innovations: list[Innovation] = []

    def update(self, encoder: EncoderReading, imu: ImuReading | None = None) -> PoseEstimate | None:
        if imu is not None:
            if imu.capture_time != encoder.capture_time:
                raise ValueError("encoder and IMU capture times must match")
            if self._imu_identity is not None and (imu.sensor, imu.frame) != self._imu_identity:
                raise ValueError("IMU stream changed")
        if self._seen is not None:
            if (encoder.sensor, encoder.frame, encoder.ticks_per_revolution) != (
                self._seen.sensor,
                self._seen.frame,
                self._seen.ticks_per_revolution,
            ):
                raise ValueError("encoder stream changed")
            if (
                encoder.capture_time <= self._seen.capture_time
                or encoder.sequence <= self._seen.sequence
            ):
                raise ValueError("encoder time and sequence must strictly increase")
        if encoder.left_ticks is None or encoder.right_ticks is None:
            self._seen = encoder
            return None
        previous = self._previous
        if previous is None:
            self.filter.time = encoder.capture_time
        else:
            dt = encoder.capture_time - previous.capture_time
            metres_per_tick = math.tau * self.radius / encoder.ticks_per_revolution
            dl = (encoder.left_ticks - previous.left_ticks) * metres_per_tick
            dr = (encoder.right_ticks - previous.right_ticks) * metres_per_tick
            transform = np.array([[0.5, 0.5], [-1 / self.separation, 1 / self.separation]])
            u = transform @ np.array([dl, dr])
            # Difference of two independent quantization errors: variance tick²/6.
            variances = metres_per_tick**2 / 6 + self.wheel_variance_per_m * np.abs([dl, dr])
            q = transform @ np.diag(variances) @ transform.T
            record = None
            # An endpoint gyro cannot represent a missing multi-sample interval.
            if (
                imu is not None
                and imu.gyro_z is not None
                and encoder.sequence == previous.sequence + 1
            ):
                residual = finite(imu.gyro_z * dt - u[1], "gyro residual")
                r = (self.gyro_stddev * dt) ** 2
                s = finite(q[1, 1] + r, "gyro innovation variance")
                nis = finite(residual * residual / s, "gyro NIS")
                accepted = nis <= self.gate
                if accepted:
                    k = q[:, 1] / s
                    u = u + k * residual
                    a = np.eye(2) - np.outer(k, [0, 1])
                    q = a @ q @ a.T + np.outer(k, k) * r
                record = Innovation("gyro_increment", (residual,), ((s,),), nis, accepted)
            self.filter.predict(float(u[0]), float(u[1]), q, encoder.capture_time)
            if record is not None:
                self.innovations.append(record)
        self._previous = self._seen = encoder
        if imu is not None:
            self._imu_identity = (imu.sensor, imu.frame)
        return self.filter.estimate
