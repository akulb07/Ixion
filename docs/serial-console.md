# Debug output during a rover run

```sh
ixion rover-brake examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_wall_debug.yaml --output rover-debug.json
```

This scenario adds a simulated debug console to the rover/IMU run. It records
start/end, status, sonar echo/timeout, brake, PWM changes, IMU-ready and selected IMU-read
messages. These are newline-delimited JSON strings in the report's
`serial.completed` and `serial.pending` arrays. Start/end messages come from the
run harness; the host also records sensor deliveries and controller decisions.
Custom controller callbacks can now return [debug messages](rover-firmware.md#debug-messages-from-controller-code)
inside `FirmwareResponse`. These become `firmware_log` events with their callback
name, level and text, using the same queue as host messages.

Each message has an event timestamp inside its text. Its transmission record
also keeps the request, transmission start and completion times, encoded byte
count and FIFO sequence. That distinction matters: a brake message describes
when the command was issued, even if the console delivers it later.

For an example that logs its own decisions:

```sh
python examples/esp32_diff_drive/rover_imu_firmware.py --log-decisions --output imu-controller-debug.json
```

The controller's explanation is timestamped when its IMU callback runs. The
separate host `brake` event records when the next control tick changes the output.
These can be different times. Controller messages can also be dropped or left
pending. The `firmware` report counts their accepted/rejected submissions; all
traffic shares the console's byte and attempt budgets. Disabling serial rejects
controller messages without creating a console or delaying control.

## Scenario settings

```yaml
serial:
  console:
    baud_rate: 115200
    buffer_bytes: 2048
    max_total_bytes: 65536
    max_writes: 10000
  status_every_steps: 100
  imu_every_readings: 10
```

The block is optional. `status_every_steps` counts physics steps;
`imu_every_readings` logs the first delivered IMU reading and then every Nth
reading after it. All IMU readings remain in the normal telemetry regardless of
console sampling. Sonar and brake events are each submitted when observed.

The console encodes text as UTF-8 and assumes 8N1 framing: ten modeled bits per
encoded byte. A queued message starts after the previous message's transmission
ends, or at its request time if the transmitter was idle. Transmission completion
runs on the shared GPIO/I2C timeline. There is no wall-clock sleep.

## Full buffers and unfinished output

`SerialConsole.write(text)` is nonblocking. It queues the whole write and returns
`True`, or returns `False` when the configured buffer cannot hold it. The report
keeps dropped-write timestamps and byte counts. The rover continues controlling
its motors and reading its sensors; this chosen policy does not delay control.

Pending messages occupy their entire byte count until whole-message completion.
This is a conservative message queue approximation, not a byte-level UART FIFO.
No partial message is reported as delivered. A very slow console can therefore
drop messages while the robot and its other peripherals continue running.

The simulation never advances the robot just to drain this queue. Messages whose
completion lies beyond the run end remain under `serial.pending`. The final
`run_end` message is normally pending because it is submitted at that boundary.
A completion exactly at the boundary is included in `serial.completed`.

## Bounds and limits

Each nonempty write must fit within 65,536 UTF-8 bytes. Empty writes are no-ops.
Accepted and dropped attempts both consume `max_total_bytes` and `max_writes`.
Exceeding either lifetime budget raises an error; queue overflow instead returns
`False`. Configuration limits are 1–3,000,000 baud, 1–65,536 buffer bytes,
at most 1,048,576 attempted bytes and 10,000 nonempty attempts. These are software
model bounds, not ESP32 or interface ratings.

Accepted writes each reserve one completion callback within the shared clock's
200,000-event limit. A failed reservation leaves the console queue unchanged.
Reports use schema version 7, retaining the previous motion, GPIO and I2C fields;
`serial` is `null` when logging is disabled.

`drive_pwm` messages mark applied forward-duty changes. They include the source
request time as well as command time. Unchanged duty generates no extra change
event, while repeated control decisions remain available in the normal report.

Optional [encoder counter faults](encoder-faults.md) submit `encoder_counter_fault`
events at activation, including the channel, kind, offset and decoded count at
that instant. The speed controller submits an error-level `firmware_log` when a
feedback check requests braking. Both use the same bounded queue; persistent
fault and brake evidence also remains in the normal report. Count fields in
console status and brake messages are the readings delivered to firmware.

This is a logical debug console. It does not open a physical serial port, execute
UART pin wiring, emulate USB, or model CPU formatting/transmit overhead. Firmware
binary execution, serial input, command parsing and a general task scheduler remain
separate roadmap work. Logging here helps inspect timing; it does not validate
the behavior of a real board's serial driver.
