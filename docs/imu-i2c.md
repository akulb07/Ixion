# Reading the reference IMU

```sh
python examples/esp32_diff_drive/imu_check.py
```

This runs host Python code against a small MPU6050 register model. It reads the
identity, wakes the sensor and captures three known fixtures: level at rest,
forward acceleration and positive yaw. The JSON keeps the supplied physical
inputs, bytes, decoded readings and transaction timestamps. The reference bus
resolves GPIO21/GPIO22 and address 0x68 from the project.

This standalone fixture owns a sequential bus clock. There is also a combined
rover scenario using a shared timeline, described below.

## Register subset

Addresses and conversions follow the TDK InvenSense
[register map, revision 4.2](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map.pdf)
and [product specification, revision 3.4](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf).

| Address | Supported behavior |
|---|---|
| 0x75 | Read identity 0x68, including when the declared bus address is 0x69. |
| 0x6B | Read power state; write 0x00/0x01 to wake, 0x40/0x41 to sleep, or 0x80 to reset. |
| 0x1B | Read/write gyroscope range selector using 0x00, 0x08, 0x10 or 0x18. |
| 0x1C | Read/write accelerometer range selector using the same four values. |
| 0x3B–0x48 | Read the latched acceleration, temperature and gyro bytes, including one coherent 14-byte burst. |

Motion values are signed 16-bit, most significant byte first. Range selectors
cover ±2/4/8/16 g and ±250/500/1000/2000 degrees per second. Unsupported registers,
self-test bits and other power modes raise errors. DLPF, sample-rate divider,
FIFO, DMP and interrupts are not emulated, so existing device libraries may need
behavior outside this subset.

## Inputs and model choices

`IMUSample` supplies timestamp, three-axis specific force in m/s², angular
velocity in rad/s and temperature in °C, all in sensor axes. Specific force
already includes the effect of gravity: a level stationary sensor with z upward
has z = +9.80665 m/s² in this fixture. The model does not infer sensor mounting,
orientation, gravity or acceleration from a robot body.

`MPU6050.publish(sample)` latches one measurement using the configured ranges.
Reads retain that sample and its timestamp until another publication. There is
no internal sampling clock. Sleep rejects publication. Configuration changes and
reset clear the measurement; a data read then requires a fresh publication after
waking. This avoids inventing measurements during unsupported states. Wake and
reset are immediate modeling choices, without oscillator settling time.

Quantization rounds to the nearest integer with ties to even. Out-of-range
values saturate to signed 16-bit limits and produce named clipping diagnostics.
These ideal choices do not model noise, drift, bandwidth, calibration or exact
physical overrange behavior. `decode_motion_burst` converts bytes using the
range selectors chosen by host code; choosing the wrong range can decode the
wrong units, as it would in firmware.

## Bus contract

`resolve_mpu6050_wiring` checks the selected bus, sensor type, pin assignments,
SDA/SCL paths and address. This first adapter models one selected MPU6050 at
0x68 or 0x69, with frequency at most 400 kHz. Other declared devices do not become
executable models automatically. AD0 is represented by the declared address;
its electrical pin state is not simulated. Static electrical errors reject the
project, while missing ratings remain warnings.

`MPU6050I2C.read_registers` and `write_registers` return `I2CTransfer` records.
Data and writes take effect at transfer start. A read snapshots the latched data
coherently, then the adapter reserves the bus through `completion_time_s`.
`sample_time_s` identifies the measurement's age for sensor-data reads.

The approximate durations are `9 * (3 + bytes) / frequency` for register reads
and `9 * (2 + bytes) / frequency` for writes, plus the project's fixed bus latency.
This counts address/register/data phases and acknowledgment bits. START/STOP
setup times, arbitration, clock stretching, line capacitance and pull-ups are
excluded. The durations are model estimates, not measured bus performance.

Pass `at_s` to start after an idle interval; otherwise the next transfer begins
when the last one completed. Overlapping or backwards requests are rejected.
Unsupported transfers fail before changing the device or reserving bus time;
they do not emulate a timed electrical NACK. Transfers use 1–14 bytes and the
adapter accepts at most 10,000 successful transfers. Its host API returns a
record rather than claiming compatibility with an ESP32 I2C library or binary.

## Reading while the rover moves

```sh
ixion rover-brake examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_wall_imu.yaml --output rover-imu-report.json
```

The optional `imu` scenario block selects the component and bus, a poll interval
in physics steps, range selectors and an assumed constant temperature. Its
component must have exactly one mount from a chassis. The fixture explicitly
assumes that the sensor sits at the body reference with x forward and z upward;
the mount graph has no rotation/offset geometry yet.

At each physics endpoint after waking, the fixture publishes
`(body_acceleration, 0, gravity)` as specific force and zero angular velocity.
Thus the raw acceleration changes sign when the rover brakes. The source is the
rolling dynamics, including the current motor command, mass and battery estimate.
No noise, filter or internal MPU sampling clock is added.

`ScheduledMPU6050I2C` wraps the same bus adapter. It snapshots a request at its
start and delivers its response through `SignalClock.call_at` only at completion.
Encoder and sonar edges continue while that transfer is outstanding. Completion
callbacks can chain the identity, wake and range-configuration transfers; polling
starts after initialization and the first available physical sample.

Polling is nonblocking: a request attempted while the bus is busy is skipped and
counted. With the reference's 100 kHz bus and zero fixed bus latency, a 14-byte
read takes 1.53 ms in this timing model. The default polls every 10 ms. A sample
captured at 0.010 s is read at 0.010 s and delivered at 0.01153 s; it remains the
same coherent frame even if a newer sample is captured before delivery.

The `imu` report block keeps configuration in the enclosing scenario, resolved
wiring, readiness time, capture/skipped-poll counts, completed transactions and
delivered readings. Each reading retains its physical source sample, raw bytes,
decoded values and clipping diagnostics. A transfer that outlives the run remains
explicitly pending; its response is not delivered. Completion exactly at the run
end is included. GPIO and peripheral callbacks share one 200,000-event budget;
the I2C adapter's 10,000-transfer budget still applies.

This is a fixed-step coordinator for the reference fixture. Commands remain
constant during each physics interval and update on control ticks. It is not a
CPU/RTOS model. The default controller still brakes from sonar. The
[replacement controller interface](rover-firmware.md#using-the-imu-in-a-controller)
can now receive completed, decoded IMU frames through an optional `on_imu` callback.
These frames include sample/delivery times and scaled sensor values, without the
ideal physical source sample. A controller can reject old frames using the supplied
simulation timing metadata. Pending reads never reach the callback early, and
serial-log settings don't change controller delivery.
The optional [logical debug console](serial-console.md) now timestamps
IMU readiness and selected completed readings alongside controller/sonar events.
IMU power consumption, arbitrary mount poses, steering, physical serial interfaces
and a general user-firmware runtime remain separate work.
