# Straight-line rover bench

```sh
python examples/esp32_diff_drive/rolling_check.py --seconds 1 --step 0.001
```

This is the first connection from the hardware motor model to rover translation.
It uses the reference project's total mass and wheel radius. Both channels must
have identical motors, commands and initial speeds. Distance starts at zero and
can be negative for reverse travel. Battery sag, charge and motor telemetry come
from the same shared-supply solver as the shaft bench.

The example assumes 0.001 kg m² rotating inertia per wheel, friction coefficient
0.8 and 80% of the vehicle weight on the driven wheels. These are editable inputs,
not measured properties. The JSON records them alongside the component parameters.
Use `--inertia`, `--friction` and `--driven-weight-fraction` to change them.

## Mechanical approximation

For each wheel, with motor output torque T, rotating inertia J, radius r, contact
force F, angular acceleration alpha and total vehicle mass m:

```text
T = J * alpha + F * r
2 * F - m * g * sin(grade) = m * r * alpha
effective inertia per shaft = J + m * r^2 / 2
gravity load torque per shaft = m * g * sin(grade) * r / 2
body acceleration = r * alpha
```

J includes wheel and drivetrain rotation referred to the output axis. Vehicle
mass includes the wheels' translating mass; the added J accounts for rotation.
The model reuses the shaft integrator with this effective inertia. It advances
speed, distance and battery charge with explicit Euler. Compare smaller time
steps when using a new setup; there is no automatic accuracy guarantee.

Normal force is half the specified driven normal load per wheel, including
`cos(grade)` and constant throughout the run. Required contact force must stay within `friction * normal force` at
every recorded sample, including the initial state. Exceeding that bound raises
an error and no complete trace is returned. Force is not clipped: clipping without
separate wheel-slip dynamics would violate the assumed rolling constraint.

The default grade is zero. Set `--grade-rad 0.08726646259971647` for a 5° uphill
lane in this example; positive rises along travel. See [incline use and limits](rover-inclines.md).
This model covers equal-wheel straight travel on a constant grade. It does not model
turning, wheel slip, caster drag, rolling resistance, changing slopes, load transfer,
collisions or suspension. Current warnings remain estimates, not protection
actions.

## Changing commands during a run

`RollingRuntime` wraps the same bench with incremental steps. Each step holds its
two equal commands constant, then retains wheel angles, distance, speed and battery
charge for the next step. A rejected step leaves that runtime state unchanged.
It requires identical motor channels and speeds, with two equal commands; it is
not a differential-drive turning model.

The [rover wall-brake fixture](rover-brake.md) uses this runtime to connect echo
feedback to changing driver commands and moving-body telemetry. Its explicit
front boundary ends the run at first wall contact. That boundary is a geometric
fixture; it adds no impact or contact-force response to this rolling model.
