# Static electrical checks

`ixion inspect-project robot.yaml` now checks declared supply envelopes,
digital logic guarantees and active MCU functions as well as topology.
The checks run without a simulator or firmware. They use the values in the
project; Ixion does not yet supply verified part ratings automatically.

Pins can declare `accepted_voltage` and `driven_voltage`, each containing
`minimum_v`, `maximum_v` and a `reference` describing the source and conditions.
Use the full operating envelope, including battery charge and regulator tolerance,
rather than just nominal voltage. Every source envelope must fit inside every
receiver envelope on that net. Missing limits produce `voltage_limits_unknown`.
A connected power input without an explicit source produces `power_source_missing`.
This does not prove that a regulator is actually powered or enabled upstream.

Digital receivers can declare `input_levels` with `low_max_v` (VIL) and
`high_min_v` (VIH). Drivers declare `output_levels` with the same field names
for VOL and VOH guarantees. Each declaration needs a reference including the
relevant supply, temperature and load conditions. A connection is compatible
under those declarations only if VOL <= VIL and VOH >= VIH, with a compatible
voltage envelope. Missing guarantees are unknown, not ideal rails. These are
static limits; rise time, transient overshoot and load currents are not calculated.

Example pin assignment:

```yaml
assignments:
  - endpoint: {component: esp32, pin: GPIO27}
    function: PWM
```

Functions currently supported by the schema are DIGITAL_IN, DIGITAL_OUT, PWM,
INTERRUPT, I2C_SDA and I2C_SCL. Assignment references must be unique existing MCU
pins. A missing capability, impossible direction or unconnected assignment is
an error. Configured digital outputs participate in output-conflict detection.
I2C remains open-drain and is not interpreted as a push-pull output.

Pins can also declare `ground_reference: GND`, naming a ground pin on the same
component. Pins sharing a single-ended signal or supply net must have connected
references on the same ground net. Missing declarations produce a warning;
unconnected references and different reference nets produce errors. Multiple
ground pins on a component are not silently treated as internally connected.
Passive motor terminals are excluded. Isolated and differential interfaces need
a separate future model; these checks do not prescribe joining isolated grounds.

The reference robot contains assignments and ground references but no verified electrical ratings yet.
Its warnings are expected. Exact board variants, MCU pin-matrix restrictions,
I2C pull-ups, ground impedance, source activation, current limits and power
conversion still need models and rules. An unassigned GPIO or open-drain net
without an explicit signal driver reports unresolved drive analysis.

The command retains exit 0 for no detected errors, 3 for diagnostic errors and
2 for invalid input. Engineering readiness stays `not_assessed`; a check with no
errors is not a claim that the robot will work. The next step is verified reference
part definitions and electromechanical models.
