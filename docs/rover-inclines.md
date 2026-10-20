# Trying the rover on an incline

The straight-line hardware model now accepts a constant grade. You can check
whether a command can start uphill, how the rover responds on different slopes,
and whether the declared traction is sufficient. Part data and contact properties
in the reference are still assumptions.

```sh
ixion rover-brake examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_uphill.yaml --output uphill.json
ixion rover-report uphill.json --output uphill.html
```

This example is a 5° uphill lane with encoder, sonar and I2C IMU interfaces.
New starter projects also contain `rover_uphill.yaml`.
For separate level, 5° and 15° cases:

```sh
ixion rover-suite examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_grade_suite.yaml --output grade-suite.json --junit grade-suite.xml
ixion rover-report grade-suite.json --output grade-suite.html
```

These cases use the same 0.75 m sonar trigger and require 0.4 m final clearance,
with fresh battery/controller/sensor state. The viewer shows each grade and its
individual results. To compare changed designs, run this same complete suite on
both; `rover-compare` still rejects different mission conditions.

## Forces and coordinates

`grade_rad` is in radians; positive rises along forward travel. The rover accepts
`0 <= grade_rad < pi/2`. The lower-level rolling bench also accepts negative
grades, with signed motion and existing regeneration restrictions. Its example
accepts `--grade-rad`.

For mass `m`, grade `theta`, wheel radius `r` and per-wheel rotating inertia `J`:

```text
gravity load along the lane = m * g * sin(theta)
normal load per driven wheel = m * g * cos(theta) * driven_weight_fraction / 2
effective shaft inertia = J + m * r^2 / 2
gravity torque per shaft = m * g * sin(theta) * r / 2
contact force per wheel = (m * acceleration + m * g * sin(theta)) / 2
```

The gravity torque is subtracted from motor torque. Contact demand outside
`friction_coefficient * normal_load` is rejected without clipping. This uses the
[inclined gravity decomposition](https://openstax.org/books/university-physics-volume-1/pages/5-6-common-forces)
and [no-slip rolling relation](https://openstax.org/books/university-physics-volume-1/pages/11-1-rolling-motion).

Travel, speed, wall distance, offsets and clearance are **along the lane**, not
horizontal coordinates. The wall is perpendicular to travel. Normal-load fraction
is prescribed: load transfer, tipping, suspension, caster drag, rolling resistance,
changing slopes, turning and wheel-slip dynamics aren't calculated.

## Firmware and stopping

IMU x follows travel and z follows the surface normal. Ideal specific force is
`(acceleration + g*sin(theta), 0, g*cos(theta))` before register quantization.
Constant grade has zero angular velocity. See [Analog Devices' gravity projection
notes](https://www.analog.com/en/resources/app-notes/an-1057.html).
Firmware receives decoded observations, not the configured true grade. The example
forward-specific-force brake policy doesn't remove gravity, so tilt can change
its decision at the same kinematic acceleration.

Insufficient initial commanded torque reports a rollback limitation. Later
negative speed also aborts the forward fixture; review the time step if braking
crossed the small positive stop threshold. The estimate ends at that threshold.
It doesn't establish that the rover can stay parked on a slope or restart safely.

With default power conditions, level runs retain schema 7 and omit zero-valued
grade fields from their saved configuration/body. Uphill runs use schema 8 with
matching scenario/body grade. Nondefault [power conditions](rover-power.md) use
schema 9 for either grade, or schema 10 with a [declared voltage limit](rover-voltage-limits.md).
Build-reference packets containing a graded suite
use manifest version 2 with default power; nondefault-power suites use version 3.
Declared voltage limits or the new operating bounds use version 4. Default-power
level-only packets keep version 1. Older level suites and build packets remain
supported. These are estimates pending physical validation and add no 3D mesh.
