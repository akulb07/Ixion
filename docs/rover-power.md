# Running the rover with a different power budget

The moving rover can now start partly charged and carry a constant auxiliary
current allowance. Those conditions affect shared battery sag, motor torque,
motion, sensor timing and the firmware response in the same loop.

This is useful for checking a question like: "Does this braking controller still
meet the clearance check at half charge with another 0.25 A drawn from the pack?"

From the repository root, with this checkout installed:

```sh
ixion rover-suite examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_power_suite.yaml --output power-suite.json --junit power-suite.xml
ixion rover-report power-suite.json --output power-suite.html
```

The same `rover_power_suite.yaml` ships in the [starter project](starter-rover.md).
`new-project --demo` still runs the original four normal/encoder-fault cases;
run this separate suite when you want the power comparison.

The supplied cases use the same sonar controller and geometry, with a 0.75 m
brake threshold. Each independently requires at least 0.4 m final clearance and
at most 5 A sampled peak pack current. With the current assumed reference values:

| Condition | Initial charge | Auxiliary current | Final clearance |
|---|---:|---:|---:|
| Full charge | 100% | 0 A | about 0.474 m |
| Half charge | 50% | 0 A | about 0.535 m |
| Half charge with extra load | 50% | 0.25 A | about 0.476 m |

All three pass their declared checks. These are simulated reference results,
not measured hardware limits. A lower speed doesn't guarantee more clearance:
the changed approach can move detection to the next sonar poll. Inspect the
sensor and brake events along with the motion trace.

## Scenario fields

Add these directly to an ordinary `rover-brake` scenario, or to a suite case's
`scenario` mapping:

```yaml
initial_soc: 0.5
auxiliary_current_a: 0.25
vcc_v: 3.3
```

`initial_soc` must be greater than zero and at most one. It defaults to one.
`auxiliary_current_a` is a nonnegative, constant **pack-side** allowance and
defaults to zero. `vcc_v` supplies the TB6612 logic model, defaults to 3.3 V and
must be in its supported 2.7–5.5 V range. These are finite numeric inputs;
booleans, nonfinite values and unsupported ranges are rejected.

The auxiliary current enters the existing shared-supply equation at every
electrical solve. The pack current includes both PWM-averaged motor supply
currents plus this allowance. Battery sag therefore changes the available motor
voltage too. Battery charge integrates the total current, including during
braking when the ideal motor supply currents are zero.
The selected pack must share the driver's direct VM and ground nets; selecting
an unrelated numerical battery model is rejected in both sizing and moving runs.

VCC is an externally supplied model input. Changing it doesn't simulate a
regulator, MCU brownout, sensor supply voltage, changed GPIO thresholds or CPU
behavior. The allowance isn't inferred from the wiring or a regulator efficiency;
it must already be expressed as current drawn at the battery terminals.

## Saved evidence

Runs with nondefault power conditions use rover schema 9, for either level or
uphill travel. The configuration retains the declared values. Each saved sample
also includes `auxiliary_current_a`, `auxiliary_power_w` and
`interval_auxiliary_energy_j`. The metric `auxiliary_terminal_energy_j` totals
that allowance's terminal energy. `terminal_energy_j` continues to include the
whole pack-terminal load, so the auxiliary total is already part of it.
Adding a [declared voltage limit](rover-voltage-limits.md) instead uses schema 10
and retains the boundary event or an explicit unreached result.
A [coast/brake continuation](rover-power-response.md) uses schema 11 and keeps
the interval's starting power as well as its endpoint state, so command changes
and integrated energy can be inspected separately.

The viewer labels initial charge, pack-side allowance and driver VCC. Power-aware
cases add battery voltage and charge plots, plus an auxiliary-current trace.
All plots share the existing recorded-time cursor. Selecting another case
rebuilds the traces from its own saved conditions.

Schema-9 projection checks the declared initial SOC, fixed auxiliary current,
power/current/voltage relationships, nonincreasing SOC and auxiliary energy
totals. It rejects missing or inconsistent power evidence. This checks internal
consistency, not instrument authenticity or physical accuracy.

The [design comparison](rover-comparison.md) still requires identical conditions
and acceptance policies in both designs. A full-charge run and a half-charge
run are different conditions; compare both designs with the same multi-case
power suite. Auxiliary energy deltas appear for cases that record that metric.

[Build-reference packets](build-reference.md) use manifest version 3 when their
included suite has any nondefault power conditions, even if the selected case
uses full charge. Version-1 level and version-2 incline packets remain verifiable.
Suites with voltage limits or the new operating bounds use manifest version 4.
Suites with a post-limit driver response use manifest version 5.
Level runs with all legacy defaults still use schema 7; grade-only runs use
schema 8. Default power fields are omitted when serializing older cases so the
original packet contents and semantic comparisons remain compatible.

## Sizing and runtime use

The [sizing tool](rover-sizing.md) uses `soc` for its prescribed operating point.
The moving scenario calls this `initial_soc` because it then changes over time.
Use the same `auxiliary_current_a` and `vcc_v` values to align the two setups.
Sizing checks one speed and acceleration; it doesn't replace the moving test.

For a direct incremental rolling run, the same conditions can be passed to
`RollingRuntime`:

```python
runtime = RollingRuntime(
    system, BatteryState(soc=0.5), body,
    vcc_v=3.3, auxiliary_current_a=0.25,
)
```

VCC and auxiliary current stay fixed through command changes. A rejected step
leaves the runtime's battery, motion and command state unchanged.

This does not add a BMS, physical voltage cutoff, charging, temperature, automatic power
budgeting or a full mission-duration prediction. Depletion and unsupported
electrical states still abort the run; suites retain an explicit model-error
case rather than calling it a successful physical shutdown. Real measurements,
verified part ratings and physical cutoff behavior are still ahead. A declared
simulation voltage boundary can now end a run before a supported supply is lost;
see its [scope and time brackets](rover-voltage-limits.md).
