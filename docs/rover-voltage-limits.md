# Ending a run at a declared voltage limit

The moving rover can end an estimate when the shared pack supply reaches a
declared lower voltage. It keeps the position, speed, sensor events and energy
recorded up to that boundary. Reaching it is called `power_limit_reached`.

This is a simulation boundary. It doesn't disconnect a battery, simulate a BMS,
or say that a moving rover has stopped. Final speed remains visible and can be
checked separately.

To continue the motion with a declared coast or brake override, use the separate
[post-limit response](rover-power-response.md). The default action here is `end`.

```sh
ixion rover-suite examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_voltage_limit_suite.yaml --output voltage-suite.json --junit voltage-suite.xml
ixion rover-report voltage-suite.json --output voltage-suite.html
```

The suite also ships in the starter. It has a normal braking case with a 6 V
limit, an 8 V startup-limit case, and a case with a 2 A pack-side auxiliary
allowance and a 7.6 V limit. Those numbers are illustrative test conditions,
not recommended physical cutoff settings or verified part ratings.

The normal case checks clearance, final speed and terminal energy. The two
startup cases explicitly expect the declared boundary at time zero, with no
travel or integrated energy. A passing suite means those declared outcomes
were observed; it doesn't turn a voltage boundary into a working shutdown circuit.

## Configuring a run

Add this to a normal rover scenario or a suite case's `scenario` mapping:

```yaml
minimum_pack_voltage_v: 6.4
```

The value must be greater than 2.5 V and at most 13.5 V, inside the modeled
TB6612 supply range with room above its lower boundary. Choose a limit from
the actual system requirements when working with real hardware. The default
is `null`, which leaves the earlier behavior unchanged. Initial SOC, driver
VCC and auxiliary current still follow the [power-condition rules](rover-power.md).

## How the boundary is found

At each held-command interval the runtime first solves the prospective pack
voltage for the proposed motor command. If it is already at or below the limit,
the estimate ends at that command boundary without integrating another motion
interval. The event retains the proposed motor inputs and prospective voltage.
The last recorded motion sample can still show the preceding command's voltage;
the viewer labels the proposed supply separately.

Otherwise, speed and SOC are projected with the same initial derivatives as one
explicit-Euler step. The solver checks all battery OCV knots reached within that
interval, so an interior voltage dip can't be hidden by endpoint recovery.
It then bisects the first crossing, retaining an above-threshold lower time and
an at-or-below-threshold upper time. Only the lower side is committed to the
rolling runtime. It never runs the driver below its supported supply range just
to discover a later cutoff.

Events retain `phase`, `time_s`, `threshold_v`, `voltage_v`,
`bracket_end_time_s`, `bracket_end_voltage_v`, `time_tolerance_s` and the
proposed `motor_inputs`. For an interior crossing the bracket tolerance is the
largest of 1 ps, one billionth of the requested step, and four ULPs of its end
time. Search is bounded to 80 bisections. An unresolvable time boundary is an
explicit model error. The upper bracket is a mathematical preview, not another
delivered sensor event or simulated motion sample.

This only locates the boundary of the chosen Euler approximation. Smaller
physics steps are still needed to check numerical convergence. Tests cover an
analytic constant-current crossing, an interior OCV-knot dip, command-induced
sag, and convergence of a moving rover fixture. The small-capacity test fixture
is deliberately artificial; it isn't a measured battery.

## Checking the outcome

Suite policies can now include:

```yaml
acceptance:
  expected_status: power_limit_reached
  minimum_front_clearance_m: 0.4
  maximum_final_speed_m_s: 0.1
  maximum_terminal_energy_j: 5
  maximum_power_limit_time_s: 1
```

These are example criteria, not a promise that a particular design meets them.
`maximum_final_speed_m_s` and `maximum_terminal_energy_j` work for any outcome.
The time bound requires a configured voltage limit and an explicit limit/response
outcome (or deliberate wall-contact outcome). It checks the conservative **upper** edge of the
time bracket, rather than passing a deadline from the slightly earlier lower
edge. Missing requested evidence fails a check. All bounds are inclusive.

An individual `rover-brake` command returns exit code 3 for this outcome and
still saves the complete evidence, because it didn't finish the normal
wall-brake task. A suite returns zero only if every case meets its explicit
policy. JUnit distinguishes failed criteria from model errors as before.

## Saved reports and scope

Configured-limit runs with the default `end` action use rover schema 10, even when the limit isn't reached.
They include `power_limit_event` and metric `power_limit_time_s`, both `null`
when unreached. The viewer shows the threshold, limit event and time bracket
alongside the recorded trace. Its power-event filter uses the same time cursor.
Saved comparisons retain the source reports and optional limit-time deltas.

Build-reference manifest version 4 preserves suites with a declared voltage
limit or the new energy/final-speed/time criteria. Older manifests 1–3 and rover
schemas 7–9 remain supported. Absent new criteria and an absent voltage limit
are omitted from serialized older inputs to retain their packet contents.

The initial rover still needs a valid supported electrical operating point.
An already unsupported initial supply remains a model error. Depletion without
a prior voltage crossing also remains an error. Other unsupported physics is
not converted into a successful shutdown. No restart, hysteresis, debounce,
contactor, BMS delay or charging is modeled. The default `end` action doesn't
continue motion after the boundary; a supplied-power coast/brake continuation
requires the explicit response configuration linked above.
The prospective algebraic command voltage alone doesn't establish physical
driver or regenerative behavior. Physical cutoff behavior needs its own model
and reference measurements.
