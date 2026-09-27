# Planning workspace — milestone 18

Open the **Path planning** tab in Experiment setup. Choose A*, Dijkstra, RRT or
RRT*, enter the goal coordinates, extra clearance and search budget, then select
**Find path**. The start is the draft robot's initial position. The grid planners
also expose grid resolution. Sampling uses the draft seed; change it in the
Simulation tab or advanced configuration before planning.

A purple dashed line shows a successful planned path. The orange goal marker and
circle show its position and planning footprint. Planning radius is the robot's
footprint radius plus extra clearance. This is a known, static world calculation;
it does not consume simulated sensor observations or execute a navigation run.
Switching to Simulation returns to the prescribed wheel-command workflow.

The result reports status, path length, waypoint count, node expansions and
collision checks. Failed searches remain visible and exportable. Budget exhaustion
does not prove infeasibility. A grid `no_path` result concerns that discretization,
not every possible continuous route. RRT* uses the existing fixed rewire radius;
this interface does not claim continuous optimality or provide a convergence proof.

Changing a goal, planner, budget, clearance, grid resolution, draft or setup mode
clears the old result. In-flight browser requests are aborted on such changes;
an already running server search finishes within its work budget. The service
allows one active planning request per app; another receives 429.

## Reproduction

**Export planning result** downloads JSON containing the complete resolved request,
software version, SHA-256 of the canonical request, planning radius, and result.
This includes unsuccessful outcomes. Planning results are not added to simulation
history or persisted by the server; export before leaving the page.

The endpoint is `POST /api/plans`. Submit the exported `request` object unchanged
to reproduce the search with the same software version. The input contains:

- `environment`, `start`, `goal`, `footprint_radius`, and `clearance` (meters).
- `algorithm`, `seed`, and `budget` (expansions for grids, iterations for sampling).
- `resolution` for grids; `step_size`, `goal_bias`, and `rewire_radius` for sampling.

The interface uses sampling defaults of 0.5 m step size, 0.1 goal bias and 1 m
rewire radius. The API exposes all three. Non-applicable parameters remain in the
resolved export but do not affect the selected algorithm.

## Bounds

Requests permit at most 1,000 expansions/iterations, 100 obstacles, 40,000 grid
cells, and `budget * (obstacle_count + 4) <= 100000`. World dimensions must lie
between 0.01 and 1,000 m; endpoint coordinates must lie inside the world bounds.
The footprint radius is 0.01–1,000 m and extra clearance is 0–100 m. Seeds must be
nonnegative JavaScript-safe integers. The shared 1 MB body limit and local-origin
checks apply. These bounds control work, not wall-clock completion time.

Malformed or excessive requests return 422. Geometrically blocked starts/goals
return successful HTTP responses with `invalid_start` or `invalid_goal` results,
so failures remain reproducible. One-point paths are valid when start equals goal.
