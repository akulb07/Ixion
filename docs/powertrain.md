# Shared battery and motor load

The battery, TB6612 channel and DC motor models now have a shared operating-point
solver. From the repository root, after installing this checkout:

```sh
python examples/esp32_diff_drive/power_check.py
```

The example holds both reference shafts at zero speed with full forward drive.
It reports battery sag, pack current and each channel's motor/driver telemetry,
including current warnings. The assumed component parameters are included in the
JSON. This is a stalled-load estimate, not a simulated startup current waveform.

## Discharge over time

```sh
python examples/esp32_diff_drive/power_check.py --seconds 60 --step 1
```

`Powertrain.discharge_trace` advances SOC using explicit Euler charge integration,
holds current for each interval, then solves voltage and current again. It uses
simulation time, permits at most 10,000 steps and shortens the last interval to
reach the requested end time. Samples include time, battery state and channel
estimates. Smaller steps reduce numerical error; this is not an exact integrator.
Depletion or unsupported operating conditions abort the trace without publishing
a complete-looking partial run. Speed and commands remain fixed, with no mechanical
acceleration. The operating-point API below remains a separate instantaneous solve.

## Coupling

For a driven channel, let `a` be signed PWM duty, `E` the motor back EMF and `R`
the sum of motor and bridge resistance. Under the averaged model:

```text
winding current = (a * pack_voltage - E) / R
channel supply current = a * winding current
total pack current = A * pack_voltage + B
A = sum(a^2 / R)
B = auxiliary_current - sum(a * E / R)
pack_voltage = (open_circuit_voltage - battery_resistance * B)
               / (1 + battery_resistance * A)
```

This algebraic solve avoids evaluating each motor against an independent ideal
battery. Brake and high-impedance modes draw zero ideal motor-supply current;
braking can still dissipate mechanical energy in motor and bridge resistance.
The solver then evaluates the original driver and battery models at the shared
solution, retaining their diagnostics. Tests check supply current and signed power
balance, including auxiliary consumption and battery internal losses.

`Powertrain` accepts one or two `MotorChannel` entries with prescribed shaft speeds.
Auxiliary current is an explicit **pack-side** constant current. VCC is externally
specified; logic supply/regulator consumption is not automatically inferred.
Changing SOC changes the OCV used in the solve. This function does not advance
SOC, shaft speed or time.

Any regenerative channel is rejected until supply sink paths are modeled, even
if another channel would consume its generated power. Driver undervoltage and
depleted batteries also reject the estimate. No silent voltage/current clamping
or protection behavior is invented. Current warnings do not mean protection acts.

The existing averaged-PWM, constant-resistance, thermal and component-assumption
limitations still apply. There is no wheel force, firmware execution,
switching transient or wiring-graph execution here.

## Rotating shaft bench

```sh
python examples/esp32_diff_drive/shaft_check.py --seconds 1 --step 0.01 --inertia 0.01
```

`hardware.shaft_dynamics.run_shaft_bench` adds independent shaft acceleration to
the shared electrical solve. Each shaft needs an explicit effective output-axis
inertia in kg m², including rotating parts referred to that axis. The example's
0.01 value is an assumption, not a measured wheel or rotor property. It does not
derive inertia from chassis mass.

At each interval, `acceleration = (output torque - load torque) / inertia`.
Explicit Euler advances speed and battery charge using the interval-start values,
then resolves back EMF, current and voltage. Samples include speed, acceleration,
SOC and electrical telemetry; the returned trace records the load assumptions.
Angular displacement starts at zero for each shaft and integrates interval-start
speed, also using explicit Euler. It is relative shaft rotation, not robot position.
Load torque is signed and constant: a positive value is subtracted even when the
shaft reverses or starts at rest. It is not friction that automatically opposes
motion. Initial speeds and fixed commands come from the supplied motor channels.

The run uses simulation time, at most 10,000 steps and a shortened final interval.
Reduce the step and compare results to check numerical convergence. Coarse steps
can overshoot; unsupported regeneration, depletion or supply voltage aborts the
run instead of clamping the result. Initial input objects are left unchanged.
Tests compare acceleration against an analytical constant-supply motor response,
check reverse symmetry, braking, shared sag and charge accounting.

This is a lumped-inertia approximation using the existing steady electrical and
gearbox models. It does not resolve inductance, switching or gearbox compliance.
Robot translation, wheel contact, traction and hardware sensors are still ahead.
