# ESP32 differential-drive reference

From the repository root:

```sh
ixion inspect-project examples/esp32_diff_drive/robot.yaml
```

The project contains 15 component instances and 24 electrical nets, plus mechanical
links and an I2C bus. There are 15 MCU assignments; missing verified electrical
envelopes produce explicit warnings. Edit the YAML and run the check again.
Removing the standby net produces required-pin and unconnected-assignment errors.
Duplicating an endpoint rejects the graph instead
of silently changing connectivity. JSON round-trips through the same schema.

Exit codes: 0 means no errors in the implemented topology rules; 3 means design
diagnostics contain errors; 2 means invalid input. A warning always identifies the
unimplemented engineering checks. Zero is not approval to purchase or wire hardware.

The `inspect-project` command checks the netlist. The separate
[wall-brake demo](../../docs/rover-brake.md) connects the reference to straight-line
physics, encoder GPIO, sonar and optional I2C IMU and serial output. Host Python
controllers can change equal forward PWM and request a latched brake; this doesn't
execute ESP32 binaries. The [speed demo](../../docs/encoder-speed.md) adds count
feedback, with [counter fault cases](../../docs/encoder-faults.md) for checking
controller responses. The `motor-point` command still evaluates just a prescribed
operating point.

The generic motor values are assumed. Regulator and echo interface entries are placeholders requiring real
part selection. I2C pull-ups and board-specific wiring have not been validated.

See [the architecture plan](../../docs/hardware-architecture.md) for the model
boundaries and the route to the first obstacle-response firmware demo.
