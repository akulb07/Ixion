# Rover braking toward a wall

This fixture joins the hardware motor, driver and battery estimates to a moving
body, ideal encoder signals and ultrasonic feedback. Run it from the repository
root after installing the checkout:

```sh
ixion rover-brake examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_wall.yaml --output rover-report.json
ixion rover-brake examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_late_brake.yaml --output late-brake-report.json
ixion rover-brake examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_wall_imu.yaml --output rover-imu-report.json
```

The first scenario brakes earlier; the second uses a shorter distance threshold
and can reach the wall while still moving. Compare the saved outcomes, clearance
and speed rather than treating a brake command as a successful stop. These are
estimates from assumed component data, not measured stopping-distance guarantees.
Use fresh output filenames; the command does not overwrite an existing report.

## Fixture and controller

The default rover starts at rest on level ground, with equal forward commands on two
identical motors. Project wiring resolves the two channels of one TB6612 driver,
their control pins, wheels and mounted encoders. Both motor connections must have
positive lead polarity. The rolling model rejects traction demands outside its
no-slip validity bound; turning, slipping wheels and unequal motors are unsupported.

The scenario specifies a perpendicular wall and two offsets from the starting
body reference: `front_offset_m` is the rover's front boundary, and
`sensor_offset_m` locates the sensor. The sensor must sit at or behind the front,
and the front must start clear of the wall. These are explicit fixture geometry,
not geometry inferred from a chassis mesh.

At each poll, the sensor receives the wall range at trigger time. It schedules
an ideal echo pulse using the [ultrasonic timing model](ultrasonic-model.md).
The host measures that pulse, and the default Python controller requests braking
when distance is at most `brake_threshold_m`. A missing pulse follows `timeout_action` (`brake`
or `continue`). The first reason remains latched. Driver writes happen at the
next eligible control tick, so `control_every_steps` can add response delay.

Both encoders deliver their project-mounted resolution through A/B transitions
on the resolved MCU input pins. Encoders, trigger output and echo input share a
simulation clock. The host queues a trigger high/low pulse on the wired
MCU output and measures the echo at the wired MCU input. The reference resolves
GPIO16 and GPIO17; changing the wiring changes those pin choices.

`ultrasonic` selects the sensor component. `ideal_echo_interface` selects one
explicit, non-inverting logical buffer between ECHO and the MCU. The reference
selects `echo_level_shifter`, whose placeholder is treated as instantaneous ideal
logic. This does not estimate voltage conversion, delay or whether that circuit
would work electrically. Set the field to `null` only for a direct ECHO-to-MCU net.
The adapter accepts two-endpoint signal nets, an `hc-sr04-style` sensor, and either
`level-interface-placeholder` or `ideal-digital-buffer` for the selected interface.
Unsupported paths or parts are rejected; they cannot silently produce an echo.
Declared voltage incompatibilities still reject the project. Unknown ratings
remain warnings. This is no claim of ESP32 binary execution. Battery supply and
3.3 V logic are model inputs, not a simulation of regulator and power nets.

The baseline uses a wall 1 m away, front offset 0.1 m, sensor offset 0.08 m,
0.002 kg m² rotating inertia per wheel, friction coefficient 0.8 and 80% driven
weight. It runs for up to 3 s with 1 ms physics steps, sensor polls every 61 steps,
control every step, an assumed 200 microsecond response delay and a 0.6 m brake
threshold. The late-brake scenario changes that threshold to 0.3 m. Edit copies
to explore the assumptions; the report keeps the full project and configuration.

## Outcomes and report

`stopped_before_wall` means braking reduced speed to `stopped_speed_m_s` while
front clearance remained positive. The baseline threshold is 0.005 m/s. This is
a numerical stopping criterion: rolling resistance, static friction and a holding
model are absent, so it is not proof of permanent physical rest.

`wall_contact` ends at the first front-boundary contact and retains the approach
speed. There are no impact forces, deformation or post-impact motion. Other valid
outcomes are `brake_triggered` if the duration ends after commanding braking,
`brake_pending` if detection had no following control tick, and
`completed_without_brake` if no brake was requested.

Reports include samples of travel, speed, front clearance, encoder counts, driver
modes, motor current/torque, pack voltage/current, state of charge and traction
demand. Echo readings retain their trigger distance and time. Metrics include
detection and command times, response delay, distance travelled after braking,
final clearance/speed, terminal energy and peak sampled pack current. Energy and
current figures inherit the shared powertrain model's assumptions.

Report schema version 7 retains `ultrasonic_wiring`, `ultrasonic_pin_events` and
sampled trigger/echo levels. The event list records the delivered MCU pin changes
with exact simulation timestamps; it does not invent edges beyond the run end.
A valid completed trigger starts a pending measurement. A run ending while TRIG
is still high records that level without fabricating a reading or timeout.
It retains physical acceleration samples and an optional `imu` report block;
it is `null` when the scenario omits IMU polling.

Optional [counter-read faults](encoder-faults.md) can freeze a wheel's delivered
count or add an offset at a scheduled time. `encoder_feedback` records requested
faults and activations. `encoder_counts` in samples and decisions are the observed
readings; `decoded_encoder_counts` preserve the underlying GPIO decoder. With no
faults these agree. The normal sonar-only controller doesn't act on count faults;
the separate speed controller has optional plausibility checks.

The third scenario reads the [body-coupled IMU](imu-i2c.md#reading-while-the-rover-moves)
while the rover approaches the wall. Its explicit aligned mount and constant
temperature are assumptions. I2C initialization and completed responses share
the encoder/sonar timeline. The IMU observer receives a coherent frame only when
its transfer completes, with the original sample timestamp retained. Motion and
sonar braking remain identical with IMU observation disabled.

`rover_wall_debug.yaml` also enables a [logical serial console](serial-console.md).
The optional `serial` block includes completed/pending messages and
dropped-write metadata. The console shares event time with the sensors, while
its nonblocking queue leaves the motion and sensor results unchanged.

The default braking policy now uses a [host controller interface](rover-firmware.md).
Python callers can replace it with a controller that reacts to delivered ranges
and encoder counts. Reports include controller identity, available parameter fields
and the latched request in `firmware`. It now supports equal forward PWM changes
as well as braking; arbitrary firmware control and CPU timing are still pending.
Its optional `on_imu` callback can now request braking from completed decoded reads.
The supplied acceleration-limit example retains sonar fallback and checks sample age.

Version 6 adds `driver_pwm_duty` to samples and duty/request-time fields to control
decisions. The first control tick now establishes the command before computing
the initial electrical and traction state. The [ramped-start example](rover-firmware.md#changing-motor-power)
uses this to start at reduced power. A sharp reduction at speed can still hit the
power model's explicit regeneration limit; that is an error, not a clipped result.

The command returns 0 only for `stopped_before_wall`, 3 for the other valid
outcomes, and 2 for invalid inputs or runtime/file errors. Failed simulations do
not produce a complete report. Scenarios accept YAML or JSON up to 64 KiB, reject
duplicate keys and unknown fields, and allow at most 10,000 physics steps. The
shared clock allows at most 200,000 GPIO events and peripheral callbacks. Poll timing must
cover the sensor's minimum cycle and timeout window.

The mechanics use explicit Euler and averaged PWM. Check smaller steps when
changing the setup and compare against measurements before relying on the numbers.
This is a partial Phase E checkpoint, with one simple wall and assumed parts;
general scenes, acoustic effects, a general firmware runtime and physical reference
validation remain on the [roadmap](roadmap.md).
