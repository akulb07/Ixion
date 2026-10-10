# TB6612 driver estimate

The driver model now connects one channel to `DCMotor`. Try the reference example
after installing this checkout:

```sh
python examples/esp32_diff_drive/driver_check.py
```

It compares commands at a prescribed 150 RPM and 6 V motor supply, with 3.3 V
logic supply. It does not move the robot or draw from the battery model yet.

Control modes follow Toshiba's [TB6612FNG datasheet, pages 3–5](https://toshiba.semicon-storage.com/info/datasheet_en_20141001.pdf?did=10660):
STBY low disables the outputs; opposite direction inputs select drive; PWM-low
during directional drive shorts the motor; both direction inputs high brake.
Both low with PWM high is high-impedance stop. The unspecified both-low/PWM-low
combination is rejected. `DriverInputs.standby` names the STBY pin level, so
`True` means enabled. Booleans must be explicit, not integers.

The supported supply range is VM 2.5–13.5 V and VCC 2.7–5.5 V. A configurable
0.5-ohm bridge approximation defaults to the typical resistance; lower-voltage
extrapolation is flagged. The non-PWM operating-current reference is 1 A above
or equal to 4.5 V VM, otherwise 0.4 A. Exceeding it produces a diagnostic, not
current limiting. PWM, thermal conditions and pulse duration need separate analysis.

For directional drive the averaged command is `direction * duty * VM`, and the
coupled current is `(command - back_emf) / (motor_resistance + bridge_resistance)`.
Motor terminal voltage subtracts the bridge drop. Average supply current includes
direction and duty; it is not necessarily equal to motor winding current.
Braking circulates current without drawing idealized supply current. The same
resistance is assumed for braking and driving. A high-impedance output develops
back EMF with zero armature current in this simplified model.

This approximation preserves signed average power balance, but it has no inductance,
switching events, ripple, dead time, diode transitions or accurate RMS heating.
Conduction loss based on average current can underestimate real PWM losses.
Regenerative supply current is reported, not clipped or assumed acceptable to a
battery. High-speed open-circuit diode clamping is flagged but not simulated.
Logic-supply current, protection behavior and shared two-channel package heating
are not modeled. Each channel can be evaluated separately; that is not a complete
two-channel power or thermal model. Input voltage thresholds belong to the separate
electrical checks; this API receives already resolved logic values.
