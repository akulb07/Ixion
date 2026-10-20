import pytest

from roboforge.hardware.rover_firmware import (
    BrakeRequest,
    FirmwareInputs,
    HostRoverFirmware,
    IMUObservation,
    ObstacleBrakeFirmware,
    RangeObservation,
)
from roboforge.hardware.voltage_monitor import VoltageObservation


def inputs(time=0):
    return FirmwareInputs(time_s=time, left_encoder_count=0, right_encoder_count=0)


def test_range_request_waits_for_a_tick_and_first_reason_stays_latched():
    host = HostRoverFirmware(ObstacleBrakeFirmware(brake_threshold_m=0.6))
    host.on_range(RangeObservation(time_s=0.01, status="echo", distance_m=0.6), inputs(0.01))
    assert host.command == "drive"
    assert host.request.reason == "obstacle" and host.detection_time_s == 0.01
    host.on_range(RangeObservation(time_s=0.03, status="timeout"), inputs(0.03))
    assert host.tick(inputs(0.04)) == "brake"
    assert host.tick(inputs(0.05)) == "brake"
    assert host.request.reason == "obstacle" and host.detection_time_s == 0.01


@pytest.mark.parametrize("action,reason", [("brake", "echo_timeout"), ("continue", None)])
def test_default_policy_uses_echo_distance_and_explicit_timeout_action(action, reason):
    host = HostRoverFirmware(ObstacleBrakeFirmware(brake_threshold_m=0.6, timeout_action=action))
    host.on_range(RangeObservation(time_s=0, status="echo", distance_m=0.61), inputs())
    assert host.tick(inputs()) == "drive"
    host.on_range(RangeObservation(time_s=0.03, status="timeout"), inputs(0.03))
    assert (None if host.request is None else host.request.reason) == reason


def test_tick_request_can_brake_at_the_same_boundary():
    class Program:
        def on_range(self, observation, frame):
            return None

        def on_tick(self, frame):
            return BrakeRequest(reason="count_limit")

    host = HostRoverFirmware(Program())
    assert host.tick(inputs(0.1)) == "brake"
    assert host.detection_time_s == 0.1
    assert host.report()["parameters"] is None


@pytest.mark.parametrize("method", ["on_range", "on_tick", "on_imu", "on_voltage"])
@pytest.mark.parametrize("failure", ["exception", "wrong_result", "invalid_request"])
def test_callback_failures_are_explicit_and_cannot_resume(method, failure):
    class Broken(ObstacleBrakeFirmware):
        pass

    def fail(*args):
        if failure == "exception":
            raise RuntimeError("firmware crashed")
        if failure == "wrong_result":
            return {"reason": "not_a_typed_request"}
        return BrakeRequest.model_construct(reason="bad reason")

    setattr(Broken, method, fail)
    host = HostRoverFirmware(Broken(brake_threshold_m=0.6))
    with pytest.raises(ValueError, match=f"host firmware {method} failed"):
        if method == "on_range":
            host.on_range(RangeObservation(time_s=0, status="timeout"), inputs())
        elif method == "on_imu":
            host.on_imu(imu_observation(), inputs())
        elif method == "on_voltage":
            host.on_voltage(
                VoltageObservation(
                    time_s=0.001, sample_time_s=0, raw_count=1, voltage_v=7, over_range=False
                ),
                inputs(0.001),
            )
        else:
            host.tick(inputs())
    with pytest.raises(ValueError, match="failed host firmware"):
        host.tick(inputs())


def test_bad_program_and_backwards_delivery_are_rejected():
    with pytest.raises(ValueError, match="on_range and on_tick"):
        HostRoverFirmware(object())
    host = HostRoverFirmware(ObstacleBrakeFirmware(brake_threshold_m=0.6))
    host.tick(inputs(0.1))
    with pytest.raises(ValueError, match="rewind"):
        host.tick(inputs(0.09))
    with pytest.raises(ValueError, match="delivery time"):
        host.on_range(RangeObservation(time_s=0.2, status="timeout"), inputs(0.21))


def test_callback_cannot_reenter_the_host_loop():
    class Reentrant(ObstacleBrakeFirmware):
        def on_tick(self, frame):
            return host.tick(frame)

    host = HostRoverFirmware(Reentrant(brake_threshold_m=0.6))
    with pytest.raises(ValueError, match="host firmware on_tick failed: cannot reenter"):
        host.tick(inputs())
    with pytest.raises(ValueError, match="failed host firmware"):
        host.tick(inputs())


def test_input_frames_are_immutable_and_cannot_contain_a_fixture_or_invented_range():
    frame = inputs()
    with pytest.raises(ValueError, match="frozen"):
        frame.left_encoder_count = 10
    with pytest.raises(ValueError):
        FirmwareInputs(**frame.model_dump(), wall_distance_m=1)
    with pytest.raises(ValueError):
        RangeObservation(time_s=0, status="timeout", distance_m=0.5)
    with pytest.raises(ValueError):
        RangeObservation(time_s=0, status="echo")


def test_report_describes_host_contract_and_builtin_policy_parameters():
    host = HostRoverFirmware(ObstacleBrakeFirmware(brake_threshold_m=0.6))
    report = host.report()
    assert report["interface_version"] == 4
    assert report["execution"] == "host_python"
    assert report["policy"] == "forward_pwm_then_latched_brake"
    assert report["parameters"] == {"brake_threshold_m": 0.6, "timeout_action": "brake"}
    report["parameters"]["brake_threshold_m"] = 9
    assert host.report()["parameters"]["brake_threshold_m"] == 0.6


def imu_observation(time=0, sample_time=0):
    return IMUObservation(
        time_s=time,
        sample_time_s=sample_time,
        specific_force_m_s2=(2, 0, 9.80665),
        angular_velocity_rad_s=(0, 0, 0),
        temperature_c=25,
    )


def test_optional_imu_callback_latches_at_delivery_and_defers_output_until_tick():
    class Program(ObstacleBrakeFirmware):
        def on_imu(self, observation, frame):
            assert observation.time_s == frame.time_s
            assert observation.sample_time_s == 0.01
            with pytest.raises(ValueError, match="frozen"):
                observation.temperature_c = 99
            with pytest.raises(TypeError):
                observation.specific_force_m_s2[0] = 99
            return BrakeRequest(reason="acceleration_limit")

    host = HostRoverFirmware(Program(brake_threshold_m=0.6))
    host.on_imu(imu_observation(0.01153, 0.01), inputs(0.01153))
    assert host.command == "drive" and host.request.reason == "acceleration_limit"
    assert host.detection_time_s == 0.01153
    assert host.tick(inputs(0.02)) == "brake"
    host.on_range(RangeObservation(time_s=0.03, status="timeout"), inputs(0.03))
    assert host.request.reason == "acceleration_limit" and host.detection_time_s == 0.01153
    assert host.report()["imu_callback_count"] == 1


def test_program_without_imu_handler_still_runs_and_wrong_handler_type_fails():
    host = HostRoverFirmware(ObstacleBrakeFirmware(brake_threshold_m=0.6))
    host.on_imu(imu_observation(), inputs())
    assert host.tick(inputs()) == "drive"
    assert host.report()["imu_callback"] is False
    assert host.report()["imu_callback_count"] == 0

    class Invalid(ObstacleBrakeFirmware):
        @property
        def on_imu(self):
            return 42

    with pytest.raises(ValueError, match="on_imu firmware method must be callable"):
        HostRoverFirmware(Invalid(brake_threshold_m=0.6))


def test_imu_observation_rejects_future_samples_wrong_shape_and_nonfinite_values():
    with pytest.raises(ValueError, match="after its delivery time"):
        imu_observation(0, 0.001)
    data = imu_observation().model_dump()
    for invalid in ((1, 2), (1, 2, float("nan")), (True, 0, 0)):
        with pytest.raises(ValueError):
            IMUObservation.model_validate({**data, "specific_force_m_s2": invalid})
    host = HostRoverFirmware(ObstacleBrakeFirmware(brake_threshold_m=0.6))
    with pytest.raises(ValueError, match="share delivery time"):
        host.on_imu(imu_observation(0.01, 0), inputs(0.02))
