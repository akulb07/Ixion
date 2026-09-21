# Testing and numerical validation

## Strategy

Run the complete suite with `python -m pytest`. It collects `unittest.TestCase`
classes; `python scripts/test.py` is an alternate runner. Install `[dev,plot]` to
exercise the plotting test too. There are no simulator, sensor, planner or
localizer mocks pretending to validate future navigation behavior.

| Test group | Independent evidence |
|---|---|
| Unit | Vector identities, known rotations, homogeneous matrices, shape/sign checks, strict validation |
| Integration | Commands -> kinematics -> pose -> frames; configuration -> simulator -> exports |
| Regression | 10-second straight travel, full-circle closure, partition invariance, repeatability |
| Numerical | Euler step equation, first-order convergence, exact integration across step sizes |

Milestone 2 also tests contact geometry, the signed-distance Lipschitz property,
thin-wall/full-loop sweeps, tangent and near-miss paths, query-budget failures,
mass/inertia descriptions, configurable frames, event serialization, fractional
clock stops, and sparse trajectory reconstruction. Spatial sweep tolerance is
not a universal time-of-impact error bound; see the collision model.

Seeded invariant tests use local `np.random.default_rng` instances (seeds 42 and
231). They exercise 100 transform chains and 100 inverse solves, and never use
global randomness. These are repeatable property-style checks, not exhaustive
formal verification. Future algorithms should add domain-specific property tests
and Hypothesis only when it adds value.

## Numerical criteria

Analytical geometry tests typically use absolute tolerances from 1e-15 to 1e-12.
Repeated integration uses 1e-12 for the foundation demo and 1e-11 for a full
2,000-step circle. Critical trajectory assertions set relative tolerance to zero.
Euler convergence checks that halving dt reduces error by a factor in [1.99,2.01]
for a known circular trajectory over one second.

Do not regenerate reference expectations from the implementation under test.
Straight motion, spin, and circular arcs have manually derived answers. Inverse
kinematics is checked against an independent NumPy linear solve. Composition is
checked against homogeneous matrix multiplication. Mathematical correctness does
not validate omitted physical effects such as slip or contact response.

## Reproducibility bounds

Same-build repeated simulation compares the complete immutable result exactly.
Cross-platform/cross-version comparisons use the documented tolerances; bitwise
portability of floating-point libraries is not promised. Exported JSON/CSV values
are inspectable. PNG byte equality is not a reproducibility criterion because
plot libraries and fonts may change.

The development dependency ranges describe supported installation intent, not
proof of every version combination. The validation report records the one tested
environment. A dependency snapshot accompanies the deliverable for that runtime.

## Gate for later phases

Release 0.8.0 additionally validates sensor timing/noise, ray intersections,
analytical actuator response and positional convergence, PID saturation recovery,
encoder-only gain correction, odometry arcs/dropouts, A*/Dijkstra cost agreement,
seeded sampling reproducibility and RRT* budget-extension behavior, and bounded
Pure Pursuit convergence. Demonstrations export actual measured data.

Add no algorithm merely because its demo looks correct. Explain its model first,
test analytical cases and failure modes, connect it to the existing interfaces,
run the suite, demonstrate a reproducible example, and document limitations.
For future EKF, additionally check analytic Jacobians against finite differences,
covariance symmetry/PSD, angle residual wrapping and observability assumptions.
For collisions, test tangency, initial overlap, boundaries and swept motion.
