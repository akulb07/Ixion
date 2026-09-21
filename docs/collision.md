# Circular-footprint collision model

## Purpose, inputs and outputs

`CollisionWorld` holds an immutable `Environment` description and supports static
signed clearance and continuous swept motion. The footprint is a disk centred on
the base origin. Obstacles are axis-aligned rectangles or circles; world bounds
are four inward-facing half-planes. Distances are metres. The user-declared
footprint must cover all relevant hardware. Orientation does not change a disk.

`query(Vector2, radius)` returns signed `clearance`, nonnegative `minimum_distance`,
`collision`, the most limiting `Contact`, and all touching/penetrating contacts.
Each contact includes an object ID, surface witness points on robot/obstacle,
a unit normal toward free space, and penetration depth. IDs are
`boundary:left/right/bottom/top` or stable zero-based indices such as `obstacle:0`.

Positive clearance means separated, zero touching, negative overlapping. Static
queries apply no hidden tolerance. During overlap, "nearest" means the most
violated signed constraint. Individual penetration depths are not a global
translation that resolves several simultaneous overlaps.

## Static equations

For footprint radius r, circular obstacle centre o and radius R:

```text
d = ||p-o|| - R - r
n = (p-o)/||p-o||
obstacle witness = o + R*n
robot witness = p - r*n
```

At coincident centres the normal is not unique; choose +x deterministically.

For a rectangle, clamp p to its closed AABB to obtain q. Outside the rectangle,
`d=||p-q||-r`, `n=(p-q)/||p-q||`. This correctly produces rounded inflated corners;
an expanded AABB alone would falsely block some near misses. Inside/on the
rectangle, choose the nearest face with gap g and outward normal n: `d=-g-r`.
Ties use left, right, bottom, top in that order.

Boundary clearances are `x-r`, `width-x-r`, `y-r`, `height-y-r`. A footprint is
free only if every constraint is positive. Overall signed clearance is their
minimum. Tie order is boundaries as listed, then obstacle configuration order.

## Swept-path model

`KinematicMotion(start,drive,wheels,dt,method)` defines p(f), f in [0,1], through
the existing tested integrator. Exact integration produces lines/arcs; Euler
produces its straight position update at the initial heading. Centre path length
is `S=abs(v)*dt` for either method. Stationary centres, including pure spin, need
only a static check. Full loops and reverse travel are supported.

Endpoint tests miss thin obstacles/full loops. Replacing an arc with a chord can
miss an obstacle on the arc or falsely hit one inside it. We evaluate the actual
configured motion instead.

Signed distance to these primitives is 1-Lipschitz, as is the minimum of their
constraints. For interval [a,b], midpoint m:

```text
||p(f)-p(m)|| <= S*(b-a)/2
d(p(f)) >= d(p(m)) - S*(b-a)/2
```

If the lower bound is positive after a roundoff guard, the entire interval is
clear. Otherwise subdivide, processing the left half first. Earlier intervals
must be certified before returning a candidate. This is adaptive subdivision,
not fixed spatial sampling.

When an unresolved interval's arc length is at most `spatial_tolerance` (default
1e-6 m), return its left endpoint as `safe_fraction` and a midpoint report:

- `clear`: every interval certified; fraction=1.
- `initial_contact`: start touches/overlaps; fraction=0.
- `contact`: candidate midpoint has nonpositive clearance.
- `conservative_contact`: midpoint is clear, but the small interval cannot be
  certified free at the requested resolution.

The last case can conservatively stop a near miss within approximately the
spatial tolerance. It is not a confirmed physical collision. The unresolved time
interval need not bracket a real contact; there may be a near miss. Crossing-time
accuracy depends on normal velocity. Tangency uncertainty can scale with
sqrt(spatial tolerance), so the distance tolerance is not a universal time bound.

Earlier certified intervals make the returned left endpoint safe for an initially
free robot. Initial overlaps are reported, not corrected. A guard of 64 ulps at
the coordinate/travel scale accommodates roundoff; tolerances below four guards
are rejected. This is conservative floating-point engineering, not formal
interval arithmetic over arbitrary scales or libm implementations.

`max_queries` bounds work per sweep. Exhaustion or unresolvable subdivision
raises an error, never a false clear result. Cost depends on travel, clearance,
geometry and tolerance; long grazing paths can be expensive.

## Simulation policy and diagnostics

```yaml
simulation:
  dt: 0.01
  integrator: exact
  collision:
    mode: stop
    spatial_tolerance: 0.000001
    max_queries: 100000
```

Default `disabled` preserves earlier configurations. `stop` terminates at the
safe fraction using a clock-derived partial-step timestamp. Terminal wheel/body
velocities are zero; requested commands and executed motion segments remain
recorded. This is geometric stopping, not impact forces, sliding, bounce, slip
or restart behavior. Run status `collision` includes labelled conservative stops.

Events retain attempted tick, stop time, unresolved interval, midpoint time/pose,
contact report, requested wheel rates, reason and query count. Witness points
describe the candidate midpoint, not the slightly earlier stop. `collisions.json`
serializes them. CLI exit 3 denotes a recorded stop, exit 2 invalid input/numerical
failure. `validate` rejects initial footprint contact; `simulate` records it for
inspection rather than losing the failure state.

## Analytical demonstrations

`python examples/milestone_2/run.py` runs two declarative cases:

1. Thin wall: x starts at 1, v=4 m/s, r=.2, wall at x=5. Contact occurs at
   `(5-.2-1)/4=.95 s`. A dt=2 step would end at x=9, so both endpoints are clear.
2. Full loop: v=1, omega=1, start=(5,5,0), circular obstacle at (6,6), combined
   radius .3. The trajectory circle has centre (5,6), radius 1. Chord distance to
   the obstacle centre is `2*sin((pi/2-theta)/2)`. Thus first contact is
   `theta=t=pi/2-2*asin(.15)`. A full 2pi step returns to the initial clear point.

Both stop before contact without penetration. Tests additionally cover reverse
motion, all boundaries, inside/corner cases, initial/multiple overlaps, tangency,
near misses, Euler, pure spin, budgets, deterministic repeats and export/reload.

## Limitations

Only static circles/axis-aligned rectangles and circular robot footprints are
supported. There are no moving obstacles, polygons, spatial acceleration,
physical response, sensors or navigation. Uncertainty remains explicit. This
simplified model does not establish safety guarantees for real hardware.
