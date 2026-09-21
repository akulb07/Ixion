# Known-map path planning

All planners accept a static Environment, start/goal points in metres, and a
positive circular footprint radius. A shared PlanningWorld checks full line
segments with the conservative swept-footprint engine. Tangency is blocked;
near-contact uncertainty may reject a valid narrow passage. Footprint inflation
is geometric and applies to rectangles, circles and world boundaries. Budgets
bound search work; collision queries retain their separate existing safety bound.

`grid_plan` implements Dijkstra and A* over eight-connected cell centers. Edge
cost is Euclidean length. A* uses Euclidean distance to goal, an admissible,
consistent heuristic. Start/goal retain their exact coordinates and connect to
nearby centers in their local 3x3 cell neighborhood. Short start-goal connectors
are also considered. Every edge, including diagonals and endpoint connectors,
is swept. No corner-cutting shortcut is allowed. Optimality is on this finite
graph only. Coarse grids can miss continuous paths; no_path means this graph was
exhausted. Partial cells at the far world edges are omitted. Nodes are generated
lazily and expansion limits avoid allocating the entire world upfront.

`sampling_plan` implements RRT and a fixed-neighborhood-radius RRT* variant.
PCG64 uses the explicit seed; ties and tree traversal have deterministic order.
Each sample extends the nearest node by at most step_size. RRT returns the first
goal connection. RRT* chooses a cheaper collision-free parent in rewire_radius,
rewires neighbors, propagates changed costs to descendants, and uses the full
iteration budget. The best feasible goal connection is selected using updated
costs. This finite implementation makes no empirical or theoretical optimality
claim beyond the reported measured path. Nearest-neighbor search is linear and
intended for small laboratory maps, not large-scale production planning.

PlanResult contains success/invalid_start/invalid_goal/no_path/budget_exceeded,
path, length, expanded count, collision-check count, algorithm and optional seed.
Sampling failure is budget_exceeded, never proof that no path exists. Paths are
geometric polylines; steering, speed profiles, tracking and moving obstacles are
separate layers. A disk-shaped differential-drive robot can rotate in place at
the vertices, but no travel-time or dynamic feasibility is established here.
