import math

import numpy as np
import pytest

from roboforge.geometry import Pose2, Vector2
from roboforge.localization import EncoderImuEKF, PoseEKF, covariance, landmark_model, motion_model
from roboforge.sensors.readings import EncoderReading, ImuReading


def vector(pose):
    return np.array([pose.x, pose.y, pose.theta])


@pytest.mark.parametrize("yaw", [0, 1e-8, 0.4, -0.7])
def test_motion_jacobians_against_finite_differences(yaw):
    pose, distance, epsilon = Pose2(1, 2, 0.3), 0.7, 1e-6
    _, f, g = motion_model(pose, distance, yaw)
    for i in range(3):
        offset = np.eye(3)[i] * epsilon
        plus = vector(motion_model(Pose2(*(vector(pose) + offset)), distance, yaw)[0])
        minus = vector(motion_model(Pose2(*(vector(pose) - offset)), distance, yaw)[0])
        np.testing.assert_allclose(f[:, i], (plus - minus) / (2 * epsilon), atol=2e-9)
    for i in range(2):
        offset = np.eye(2)[i] * epsilon
        plus = vector(motion_model(pose, distance + offset[0], yaw + offset[1])[0])
        minus = vector(motion_model(pose, distance - offset[0], yaw - offset[1])[0])
        np.testing.assert_allclose(g[:, i], (plus - minus) / (2 * epsilon), atol=2e-9)


def test_landmark_jacobian():
    pose, point, epsilon = Pose2(0.5, -0.2, 0.7), Vector2(3, 4), 1e-6
    _, h = landmark_model(pose, point)
    for i in range(3):
        offset = np.eye(3)[i] * epsilon
        plus = landmark_model(Pose2(*(vector(pose) + offset)), point)[0]
        minus = landmark_model(Pose2(*(vector(pose) - offset)), point)[0]
        np.testing.assert_allclose(h[:, i], (plus - minus) / (2 * epsilon), atol=1e-9)


def test_straight_prediction_covariance_and_immutable_snapshot():
    source = np.diag([0.1, 0.1, 0.01])
    ekf = PoseEKF(Pose2(0, 0, 0), source)
    source[:] = 999
    old = ekf.estimate
    new = ekf.predict(1, 0, np.diag([0.04, 0.02]), 1)
    assert new.pose == Pose2(1, 0, 0)
    assert old.pose == Pose2(0, 0, 0)
    assert new.covariance[0][0] == pytest.approx(0.14)
    assert new.covariance[2][2] == pytest.approx(0.03)
    assert old.covariance[0][0] == 0.1


def test_wrapped_bearing_gating_and_joseph_psd():
    ekf = PoseEKF(Pose2(0, 0, 0), np.eye(3) * 0.1)
    record = ekf.correct_landmark(Vector2(-2, 0.001), 2, -math.pi + 0.001, np.eye(2) * 0.01, time=0)
    assert abs(record.residual[1]) < 0.01 and record.accepted
    before = ekf.estimate
    rejected = ekf.correct_landmark(Vector2(2, 0), 100, 2, np.eye(2) * 0.01, time=0)
    assert not rejected.accepted and ekf.estimate == before
    for tick in range(1, 100):
        ekf.predict(0.01, 0.002, np.eye(2) * 1e-5, tick * 0.01)
        predicted, _ = landmark_model(ekf.estimate.pose, Vector2(3, 3))
        ekf.correct_landmark(Vector2(3, 3), *predicted, np.eye(2) * 0.01, time=tick * 0.01)
        p = np.array(ekf.estimate.covariance)
        np.testing.assert_allclose(p, p.T, atol=1e-14)
        assert np.linalg.eigvalsh(p).min() >= -1e-14


@pytest.mark.parametrize(
    "matrix",
    [np.diag([1, -1, 1]), np.ones((2, 2)), [[1, 1, 0], [0, 1, 0], [0, 0, 1]], np.eye(3) * np.nan],
)
def test_invalid_covariance(matrix):
    with pytest.raises(ValueError):
        covariance(matrix, 3)


def test_old_measurements_and_failed_prediction_leave_state():
    ekf = PoseEKF(Pose2(0, 0, 0), np.eye(3), time=1)
    before = ekf.estimate
    with pytest.raises(ValueError):
        ekf.predict(1, 0, np.eye(2), 1)
    with pytest.raises(ValueError):
        ekf.correct_landmark(Vector2(1, 0), 1, 0, np.eye(2), time=0)
    assert ekf.estimate == before


def encoder(t, left, right):
    return EncoderReading(
        sensor="encoders",
        frame="base",
        sequence=t,
        capture_time=t * 0.1,
        delivery_time=t * 0.1,
        ticks_per_revolution=1000,
        left_ticks=left,
        right_ticks=right,
    )


def imu(t, rate):
    return ImuReading(
        sensor="imu",
        frame="imu",
        sequence=t,
        capture_time=t * 0.1,
        delivery_time=t * 0.1,
        gyro_z=rate,
        acceleration_x=None,
        acceleration_y=None,
    )


def test_gyro_fusion_and_dropout_gap_policy():
    ekf = EncoderImuEKF(0.05, 0.3, Pose2(0, 0, 0), np.eye(3) * 0.01)
    ekf.update(encoder(0, 0, 0), imu(0, 0))
    result = ekf.update(encoder(1, 100, 120), imu(1, 0))
    assert abs(result.pose.theta) < 0.001
    assert len(ekf.innovations) == 1
    assert ekf.update(encoder(2, None, 240), imu(2, 0)) is None
    result = ekf.update(encoder(3, 300, 360), imu(3, 0))
    assert result.pose.theta > 0.04  # gap uses encoders, not a single endpoint gyro
    assert len(ekf.innovations) == 1


def test_fusion_rejects_mixed_times_and_streams():
    ekf = EncoderImuEKF(0.05, 0.3, Pose2(0, 0, 0), np.eye(3))
    with pytest.raises(ValueError):
        ekf.update(encoder(0, 0, 0), imu(1, 0))
    ekf.update(encoder(0, 0, 0))
    with pytest.raises(ValueError):
        ekf.update(encoder(0, 0, 0))
