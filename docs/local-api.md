# Local run API — milestone 16

Install the optional service and start it from a terminal:

```sh
pip install -e ".[dev,plot,api]"
roboforge serve --output results/service --port 8765
```

Interactive endpoint documentation is at http://127.0.0.1:8765/docs.
The CLI binds only to loopback. This is a local, single-user service with no
authentication; it is not a deployment configuration for a shared server.

## Workflow

1. Read `GET /api/presets` for sensor and timed wheel-slip examples, or
   `GET /api/config/schema` for the strict configuration schema.
2. Send a configuration object to `POST /api/config/validate` to resolve defaults
   and inspect the estimated workload.
3. Send the same object to `POST /api/runs`. The 202 response includes the run ID.
4. Poll `GET /api/runs/{id}`. Status progresses from queued to running, then
   completed, collision, failed, or cancelled. Metrics appear after export.
5. Inspect `GET /api/runs/{id}/frame?time=0.5` and
   `GET /api/runs/{id}/trajectory?max_points=1000` after completion.
6. Retrieve individual files at `GET /api/runs/{id}/artifacts/trajectory.csv`.

`GET /api/runs?offset=0&limit=50` lists newest runs first. Configuration is at
`GET /api/runs/{id}/config`; health/version is at `GET /api/health`.
`POST /api/runs/{id}/cancel` requests cooperative cancellation. Requests against
finished runs are idempotent. Cancellation is checked between simulation steps;
an in-progress collision query or sensor scan finishes first. Cancelled runs
retain configuration and status but do not expose a partial replay.

## Persistence and replay

One worker executes runs; at most eight active or queued jobs are accepted.
Additional submissions receive 429. An OS-held lock prevents a second service
from opening the same results directory. Shutdown requests cancellation and
waits for the worker. On startup, unfinished jobs become interrupted rather than
being silently rerun. Unreadable job records are listed as recovery warnings.

Successful runs contain configuration, metadata, trajectory, motion segments,
sensor/controller/actuator/fault logs, metrics, job status, and a SHA-256 manifest.
The service publishes a completed status after export and manifest creation.
Replay validates the manifest on first load, with two replay logs cached in
memory. Treat exported run directories as immutable while the service is active.

Frames interpolate recorded motion without executing the simulator. They contain
ground-truth state, the latest delivered reading for each sensor, the latest
controller and actuator samples, and faults scheduled at that time. Measurement
capture and delivery times are retained. Ground truth is explicitly a separate
field from sensor readings. Trajectory responses preserve both endpoints and
declare whether intermediate states were sampled; the CSV retains every state.

## Bounds and current scope

Each submission is limited to 1 MB, 50,000 steps, 300 simulated seconds, 32
sensors, 64 faults, 100,000 estimated readings, and one million LiDAR rays.
Additional world and collision-query budgets apply. These are workload bounds,
not wall-clock deadlines. Invalid configurations and excessive workloads receive
422 before a job is created. Cross-origin browser writes and unexpected Host
headers are rejected; these checks do not replace authentication.

The API supports the existing prescribed-wheel-command simulation with optional
PID, actuator models, sensors, and faults. Navigation, SLAM, experiment sweeps,
and planner benchmarks remain available through their Python/CLI workflows;
they are not yet service job types. The [React workspace](workspace.md), added in
milestone 17, is served at `/`. Swagger documentation may require internet access for its
browser assets; the JSON API and OpenAPI schema work locally.
