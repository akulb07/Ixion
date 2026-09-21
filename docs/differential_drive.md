# Differential-drive kinematics and integration

## Purpose, assumptions, inputs and outputs

`DifferentialDrive` maps angular wheel rates to motion, maps a desired body twist
back to wheel rates, and advances a pose. Both wheels have equal positive radius
`r`, with axle separation `L > 0`. They roll without slip on a flat plane.
The base origin is the axle midpoint. Wheel rates `wl`, `wr` are in rad/s and
positive for forward rolling. This avoids confusing angular rates with linear
rim speeds; the specification's wheel-velocity symbols are interpreted as angular.

Outputs are forward body speed `v` in m/s, yaw rate `omega` in rad/s, and pose in
world coordinates. Lateral body velocity is constrained to zero. A generic
holonomic twist is not representable by this drive model.

## Forward and inverse derivation

At y=+L/2 (left wheel) and y=-L/2 (right wheel), rigid-body rotation adds forward
velocity `-omega*L/2` and `+omega*L/2`. Rolling constraints therefore give:

```text
r*wl = v - omega*L/2
r*wr = v + omega*L/2
```

Adding and subtracting yields:

```text
v     = r*(wr + wl)/2
omega = r*(wr - wl)/L

wl = (v - omega*L/2)/r
wr = (v + omega*L/2)/r
```

Projecting the forward velocity onto world axes gives:

```text
x_dot = v*cos(theta)
y_dot = v*sin(theta)
theta_dot = omega
```

Equal rates produce a straight line, opposite rates spin about the axle midpoint,
and `wr > wl` produces positive yaw. Negative rates permit reverse motion.

## Exact integration

For constant wheel rates over `dt`, let `a = omega*dt`. Integrating sine/cosine
directly for nonzero omega yields:

```text
dx = (v/omega) * [sin(theta+a) - sin(theta)]
dy = (v/omega) * [cos(theta) - cos(theta+a)]
```

Those differences are ill-conditioned near zero omega. Applying sum-to-product
identities produces the equivalent stable formula:

```text
sinc(z) = sin(z)/z, sinc(0)=1
d = v*dt*sinc(a/2)
x_new = x + d*cos(theta+a/2)
y_new = y + d*sin(theta+a/2)
theta_new = wrap_angle(theta+a)
```

For `|z| < 1e-4`, the implementation evaluates
`sinc(z) = 1 - z^2/6 + z^4/120`. The omitted `z^6/5040` term is below 2e-28 at
the threshold, well below float64 rounding. At zero yaw this becomes straight
motion continuously, with no artificial angular cutoff in the physical model.

## Euler integration

Forward Euler evaluates derivatives at the old heading:

```text
x_new = x + v*dt*cos(theta)
y_new = y + v*dt*sin(theta)
theta_new = wrap_angle(theta + omega*dt)
```

Its global trajectory error is first order in dt for smooth bounded motion.
Halving dt approximately halves positional error on the tested circular motion.
Exact integration is the default and the reference for piecewise-constant wheel
rates; it is not an exact solution for arbitrarily varying actuators within a step.
Euler exists for explicit numerical comparison. RK4 and semi-implicit methods
should be introduced only with continuous/dynamic models that need them.

## API

```python
from roboforge.geometry import Pose2
from roboforge.robotics import DifferentialDrive, WheelSpeeds, BodyTwist2

drive = DifferentialDrive(wheel_radius=0.05, wheel_separation=0.30)
wheels = drive.inverse(BodyTwist2(0.5, 1))  # left=7, right=13 rad/s
twist = drive.forward(wheels)
next_pose = drive.integrate(Pose2(), wheels, dt=0.01, method="exact")
derivative = drive.derivative(Pose2(), wheels)  # x_dot, y_dot, theta_dot
```

The derivative is a tuple, not a Pose2, because rates must never be angle-wrapped.
Zero dt is a no-op, negative/nonfinite dt is invalid. The integrator name is
validated even for zero duration. There are no hidden limits or saturation.

## Independent reference cases

For r=.05 m, L=.30 m:

| Wheel rates (left, right) rad/s | Duration s | Expected pose from origin |
|---|---:|---|
| (10,10) | 10 | (5,0,0) |
| (-3,3) | pi/2 | (0,0,pi/2) |
| (2,4) | 3*pi/2 | (.45,.45,pi/2) |
| (4,2) | 3*pi/2 | (.45,-.45,-pi/2) |
| (0,0) | 10 | (0,0,0) |

The revised simulation demo starts at (1,1,0). Wheel rates (1,1) give v=.05 m/s,
so 10 s gives (1.5,1,0). Rates (-1,1) give omega=1/3 rad/s, so the next 10 s
leave position unchanged and give heading `wrap(10/3) = -2.949851973846253` rad.

## Tests and limitations

Tests use the above analytical answers, an independent 2x2 linear solve for
inverse kinematics, small-angle limits, reversed travel, full-circle closure,
repeatability, step-partition invariance, and Euler convergence. Numerical
reference tolerances are typically 1e-12, with 1e-11 for the 2,000-step circle.

This is kinematics, not dynamics: no inertia, friction, wheel deformation, slip,
actuator response, acceleration limits, encoder model or odometry estimator.
Do not interpret mathematically integrated ground truth as a sensor reading.

