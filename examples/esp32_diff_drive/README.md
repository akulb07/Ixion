# ESP32 differential-drive reference

From the repository root:

```sh
ixion inspect-project examples/esp32_diff_drive/robot.yaml
```

The project contains 15 component instances and 24 electrical nets, plus mechanical
links and an I2C bus. Edit the YAML and run the check again. Removing the standby
net produces required-pin errors. Duplicating an endpoint rejects the graph instead
of silently changing connectivity. JSON round-trips through the same schema.

Exit codes: 0 means no errors in the implemented topology rules; 3 means design
diagnostics contain errors; 2 means invalid input. A warning always identifies the
unimplemented engineering checks. Zero is not approval to purchase or wire hardware.

This is a netlist fixture, not the closed-loop demo yet. There is no firmware,
register interface, motor physics or backend attached. The generic motor values
are assumed. Regulator and echo interface entries are placeholders requiring real
part selection. I2C pull-ups and board-specific wiring have not been validated.

See [the architecture plan](../../docs/hardware-architecture.md) for the model
boundaries and the route to the first obstacle-response firmware demo.
