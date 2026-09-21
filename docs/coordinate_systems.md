# Planar geometry and coordinate conventions

## Model and assumptions

RoboForge uses a right-handed planar world: +x points right in the top-down view,
+y points up, and +z points out of the plane. Robot base +x is forward, +y is left.
Positive yaw is counterclockwise from world +x. Lengths are metres, time seconds,
and angles radians. Mass/force, when implemented, will use kilograms/Newtons.

The base origin is the midpoint of the wheel axle. Static nominal mounts are:
left wheel `(0, +L/2)`, right wheel `(0, -L/2)`; LiDAR and IMU offsets will be
declared relative to base. The math demo/tests demonstrate these frame names;
there are no sensor models or general-purpose frame graph in this milestone.

## Vectors and poses

`Vector2(x,y)` supports addition/subtraction, scalar operations, dot/cross product,
norm, normalization and rotation. Its caller supplies units and frame. A vector
alone cannot check that two operands share those meanings. Normalizing zero and
dividing by zero raise clear errors; `hypot` avoids unnecessary squaring overflow.

`Pose2(x,y,theta)` is an immutable value in a caller-known reference frame. It is
not a transform until source/target frames are supplied. A canonical heading lies
in `[-pi,pi)`, so +pi maps to -pi. This is an orientation, not an accumulated
revolution counter; do not subtract headings without wrapping their difference.

`wrap_angle` uses remainder by 2pi and remaps the +pi endpoint. Avoiding an
initial addition of pi preserves very small angles that a naive formula loses.

## SE(2) derivation

Rotation maps the source basis vectors to `(cos(theta),sin(theta))` and
`(-sin(theta),cos(theta))`, so

```text
R(theta) = [ cos(theta)  -sin(theta) ]
           [ sin(theta)   cos(theta) ]

T_target_source = [ R  t ]
                  [ 0  1 ]

p_target = R * p_source + t
```

The translation is the source origin expressed in the target frame. Homogeneous
column vectors use `[x,y,1]` for points and `[vx,vy,0]` for free vectors.
`apply_point` includes translation; `apply_vector` only rotates. `apply_pose`
transforms position and adds/wraps heading.

For `T_a_b @ T_b_c`, substitution gives `R_a_c = R_a_b R_b_c` and
`t_a_c = R_a_b t_b_c + t_a_b`. The right-hand transform acts first. Mismatched
intermediate frame names fail immediately. The inverse solves for the original
point: `R_inverse = R.T` and `t_inverse = -R.T t`.

## Example and API

```python
from math import pi
from roboforge.geometry import Pose2, Transform2, Vector2

world_base = Transform2.from_pose(Pose2(2, 3, pi / 2), target_frame="world", source_frame="base")
base_lidar = Transform2("base", "lidar", Vector2(0.2, 0))
world_point = (world_base @ base_lidar).apply_point(Vector2(1, 0))
# world_point ~= Vector2(2, 4.2)
```

`matrix` returns a fresh NumPy float64 matrix. `from_matrix` validates shape,
finiteness, homogeneous bottom row, orthonormality and determinant +1, rejecting
shears, scale and reflection. Its absolute tolerance defaults to 1e-10 and must
be in `(0,1e-6]`. Accepted roundoff is canonicalized into a rotation angle; this
is not a general matrix repair procedure. Changing a returned array never mutates
the original immutable value.

## Numerical considerations and limitations

All scalars must be finite real numbers; booleans/strings are not numerical inputs.
Results that overflow representable float64 values fail rather than silently
propagating infinity. Large positions can lose small increments to rounding, and
very large angles lose precision in trigonometric argument reduction. This
foundation is intended for ordinary robot-scale coordinates and time intervals.
No automatic unit conversion, quaternion/SE(3) support, covariance transformation,
timestamped frame tree, or frame tagging on every vector is provided yet.

## Tests

Known 90-degree rotations, point/vector differences, matrix composition,
pose composition, inverse round trips, determinant checks, invalid matrix inputs,
and 100 seeded frame chains test these formulas independently. Composition is
also checked against NumPy homogeneous matrix multiplication. See
`tests/unit/test_geometry.py` and `tests/integration/test_math_pipeline.py`.

