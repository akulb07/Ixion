"""Read known physical inputs through the reference robot's logical I2C IMU."""

import json
from pathlib import Path

from roboforge.hardware import load_project
from roboforge.hardware.board_i2c import MPU6050I2C, resolve_mpu6050_wiring
from roboforge.hardware.mpu6050 import MPU6050, IMUSample, decode_motion_burst


def main():
    project = load_project(Path(__file__).with_name("robot.yaml"))
    wiring = resolve_mpu6050_wiring(project, "esp32", "sensors", "imu")
    device = MPU6050()
    bus = MPU6050I2C(wiring, device)
    identity = bus.read_registers(wiring.address, 0x75, 1)
    wake = bus.write_registers(wiring.address, 0x6B, b"\x00")
    cases = (
        ("level_at_rest", (0, 0, 9.80665), (0, 0, 0)),
        ("forward_acceleration", (2, 0, 9.80665), (0, 0, 0)),
        ("positive_yaw", (0, 0, 9.80665), (0, 0, 0.5)),
    )
    readings = []
    for label, force, angular in cases:
        physical = IMUSample(
            time_s=bus.time_s,
            specific_force_m_s2=force,
            angular_velocity_rad_s=angular,
            temperature_c=25,
        )
        device.publish(physical)
        transfer = bus.read_registers(wiring.address, 0x3B, 14)
        readings.append(
            {
                "fixture": label,
                "input": physical.model_dump(),
                "transfer": transfer.model_dump(),
                "decoded": decode_motion_burst(bytes(transfer.data)),
                "diagnostics": device.diagnostics,
            }
        )
    print(
        json.dumps(
            {
                "schema_version": 1,
                "result_type": "ideal_peripheral_fixture",
                "project": project.model_dump(mode="json"),
                "wiring": wiring.model_dump(),
                "identity_transfer": identity.model_dump(),
                "wake_transfer": wake.model_dump(),
                "readings": readings,
                "limitations": [
                    "Known fixture inputs, not coupled to rover motion or controller feedback",
                    "Aligned sensor frame; supplied acceleration is specific force, including gravity",
                    "Independent sequential I2C clock; no shared rover/GPIO scheduling",
                    "Instant wake/reset; no internal sample clock, filtering, noise, bias, FIFO, DMP or interrupts",
                    "Logical bus timing only; power, pull-ups, line capacitance and clock stretching are unmodeled",
                ],
            },
            indent=2,
            allow_nan=False,
        )
    )


if __name__ == "__main__":
    main()
