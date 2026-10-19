# Can this rover handle the load?

`rover-size` works backward from a speed, acceleration and uphill grade to the
torque, current and PWM the two motors would need. It's a quick way to try a
payload or battery charge change before setting up another moving experiment.

From the repository root, with this checkout installed:

```sh
ixion rover-size examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_sizing.yaml --output sizing.json
```

Or create a starter and run its included input:

```sh
ixion new-project my-rover
ixion rover-size my-rover/robot.yaml my-rover/rover_sizing.yaml --output sizing.json
```

The supplied example asks for 0.3 m/s and 0.2 m/s² acceleration on a 5° uphill
lane, at 50% charge, with an 80% PWM ceiling and an explicit 0.1 A pack-side
auxiliary allowance. With the current assumed reference parts, the estimate is
about 44.95% PWM, 0.402 A per motor and 0.461 A from a pack sagged to 7.131 V.
The remaining traction margin is about 5.98 N per driven wheel. Motor current
and pack current differ because the driver model averages PWM power.

That example passes the known screening checks. The pack current ratings are
missing and the fitted motor has no continuous-current rating, so those checks
stay `not_assessed`. Engineering readiness stays `not_assessed` in every report.

## Changing the demand

The YAML has these fields. Field names include their units, and grade is in
radians: `0.08726646259971647` is 5°.

| Field | Meaning |
|---|---|
| `speed_m_s` | Required forward speed at this operating point, including zero. |
| `acceleration_m_s2` | Required nonnegative acceleration at that speed; zero asks for steady travel. |
| `grade_rad` | Constant uphill grade, from zero up to but excluding π/2. |
| `soc` | Battery state of charge from zero to one. Zero fails the charge check. |
| `max_pwm_duty` | Declared controller ceiling, greater than zero and at most one. |
| `auxiliary_current_a` | Constant pack-side current allowance. The wiring does not infer this load. |
| `vcc_v` | Supplied driver logic voltage, 2.7–5.5 V; defaults to 3.3 V. |
| `rotating_inertia_per_wheel_kg_m2` | Rotating inertia referred to each wheel shaft. |
| `friction_coefficient` | Assumed contact coefficient used for the no-slip check. |
| `driven_weight_fraction` | Fraction of total normal load shared equally by the two driven wheels. |

Selectors `left_motor`, `right_motor` and `battery` default to the reference
component IDs. Mass and wheel radius come from the project's assembly. The two
motors must have identical derived models, drive distinct linked wheels through
separate channels on one TB6612, and use positive lead polarity. The selected
pack must directly share the driver's VM net and ground. This does not execute
the MCU, sensors, regulators or every electrical connection.

For a payload comparison, save a copy of `robot.yaml`, change
`assembly.total_mass_kg`, and run both designs against the same sizing input.
For a battery comparison, keep the design and demand fixed and change `soc` in
a copy of the sizing input. Keep each output name new; existing files and input
files are never overwritten.

## Reading a result

Reports have artifact type `ixion_rover_sizing`, schema version 1. They preserve
the complete project snapshot, normalized design fingerprint, demand, derived
models, selected connections and the original parameter evidence labels.

The `result` contains force, wheel torque, motor current, required PWM and signed
traction/PWM margins. Its `point`, when available, is an evaluation through the
existing shared-battery solver with the calculated command. It includes motor,
driver and battery power, losses and diagnostics. The driver VM check retains
the mathematical candidate voltage even when driver execution is unavailable.

Each check retains actual value, bounds, units, status and an explanation:

- `checks_passed` means no implemented bound was exceeded. Missing ratings can
  still leave individual checks unassessed.
- `checks_failed` means at least one bound was exceeded. The command returns
  exit code 3 and still saves the full evidence.
- Invalid input or unsupported project structure returns exit code 2 without
  creating a report. Normal successful screening returns zero.

A required duty above the declared ceiling fails the PWM check. Values above
100% remain visible as an impossible demand; they are not clamped into a working
command. A traction failure likewise preserves the electrical estimate when
one exists, while failing the no-slip requirement. If the battery equation has
no positive higher-voltage solution, required duty and electrical point are
`null`; the required mechanical torque and motor current remain available.

The driver's non-PWM current reference is a screening value from the existing
model, not a PWM RMS or thermal rating. Known battery current exceedances also
fail screening without simulating a fuse, BMS or current limiter. A mathematical
electrical point can therefore remain present in a failed report. Its command
is the calculated demand, not a recommendation to apply it to hardware.

## Equations

These invert the same [rolling mechanics](rolling-model.md),
[motor fit](motor-model.md) and [shared-supply model](powertrain.md) used by the
moving fixture. For mass `m`, wheel radius `r`, rotating inertia per wheel `J`,
grade `θ`, desired speed `v` and acceleration `a`:

```text
force_per_wheel = (m*a + m*g*sin(θ)) / 2
torque_per_wheel = J*a/r + force_per_wheel*r
traction_limit_per_wheel = μ*m*g*cos(θ)*driven_weight_fraction / 2
rotor_speed = gear_ratio*v/r
motor_current = (torque_per_wheel/(gear_ratio*gear_efficiency)
                 + rotor_friction*rotor_speed) / motor_constant
required_bridge_voltage U = back_EMF + motor_current*(motor_resistance + bridge_resistance)
```

With `I` the required current in **each** identical motor, pack resistance `Rb`,
open-circuit voltage `OCV`, auxiliary current `Iaux` and duty `d`:

```text
pack_current = 2*d*I + Iaux
pack_voltage = OCV - Rb*pack_current
d*pack_voltage = U
C = OCV - Rb*Iaux
D = C² - 8*Rb*I*U
d = 2*U / (C + sqrt(D))
```

For positive motoring current, `C` must be positive and `D` nonnegative. The
rationalized expression avoids subtractive cancellation and also works when
pack resistance is zero. It selects the lower-duty, higher-voltage branch;
the alternative low-voltage branch is not searched. This is not a battery
stability model. An ideal lossless, zero-current demand uses coasting instead
of synthesizing an active drive or tiny numerical regenerative current.

Tests check force/torque equations, whole-system power balance, both battery
roots, the zero-resistance limit, invalid demands and the existing runtime's
acceleration. A steady uphill point also retains its speed in a rolling trace.

## What this still leaves out

This checks one instantaneous operating point. It doesn't establish that the
rover can accelerate from rest to that point, complete a route, stay parked on a
slope, or run for a claimed mission duration. The battery point's charge-limited
runtime assumes its current never changes and has no cutoff-aware mission model.
For a time history, use the [closed-loop incline fixture](rover-inclines.md).

Only forward speed, nonnegative acceleration and a constant uphill grade are
supported. Reverse, downhill braking, turning, rolling resistance, load transfer,
wheel slip and temperature are still outside this sizing model. VCC and auxiliary
current are explicit allowances; the current rover runtime doesn't yet accept
those allowances, so matching that runtime requires its existing 3.3 V/zero-auxiliary
conditions. The default 0.5 Ω driver resistance remains an assumption. Real
measurements and exact part variants are still needed to validate predictions.
