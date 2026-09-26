# Interactive workspace — milestone 17

Install `roboforge[api]`, run `roboforge serve --output results/service`, and open
http://127.0.0.1:8765/. The React/TypeScript interface and its assets ship inside
the wheel. Node is required only to change the frontend; normal use needs no
frontend build, CDN, external fonts, or internet connection.

## Working with an experiment

Choose the sensor or wheel-slip laboratory. Edit the seed, timestep, wheel rates
and step count, then select **Run experiment**. Validation errors appear before
submission. The service runs simulations in its bounded background queue. Use
**Cancel run** while a selected job is queued or running.

The center world view shows the configured world before a run and the recorded
world after selecting a saved run. Replay becomes available after exports finish.
Play, pause, scrub the timeline, change playback speed, or return to the start.
Replay advances only when the requested frame arrives; speeds are target rates
and slower machines can play more slowly. It never recomputes the simulation.

Truth position, heading and speed appear separately from sensor readings. Sensor
cards retain capture and delivery timestamps. LiDAR overlays use the robot pose
at capture time plus the sensor's mount transform, including built-in wheel
frames. At most 360 rays from the first LiDAR sensor are drawn; all configured
sensor cards remain visible. The truth trail and shaft-speed chart use at most
3,000 saved states; the complete trajectory is available as a CSV download.

The fault inspector shows scheduled intervals and which faults are active at the
selected replay frame. A wheel-slip fault reduces ground motion without changing
the shaft-speed meaning of the wheel chart.

Run history reloads from disk. Selecting a run does not alter the draft on the
left. Use **Use setup** to clone the selected run's configuration into the draft.
**Preview draft world** returns to the draft without submitting it. History shows
the most recent 20 runs and can expand to 100; the API supports offset pagination
for older results.

Expand **Advanced configuration** to edit the complete JSON, including world
obstacles, sensors, PID, actuator parameters and faults. Validate to apply JSON
edits. Invalid JSON leaves the last applied world intact. Import accepts a JSON
configuration up to 1 MB; Export JSON saves the current editor text, including any
unapplied edits. Saved-run setup, trajectory and integrity manifest downloads are
available in the history panel.

## Development and checks

From `frontend`, use Node 22+ and pnpm 11+:

```sh
pnpm install --frozen-lockfile
pnpm build
pnpm test
```

Generated files live in `src/roboforge/web/assets` and are committed so the Python
release remains self-contained. Rebuild before packaging a wheel. The build
includes third-party MIT license notices. On Windows environments that deny
esbuild's ancestor-directory enumeration, set `ROBOFORGE_NODE_RESOLVE=1` to use
the build script's Node-based file resolver. It reads the same project/dependency
files and runs the same compiler. TypeScript checking still runs before bundling.

Backend integration tests verify packaged assets and capture-time poses.
Frontend numerical tests verify sensor transforms, ray rendering bounds and
terminal replay time. Browser validation covers simulation, replay, repeated
selection, cloning, invalid JSON, and desktop/mobile layout.

## Current scope

This workspace operates prescribed-wheel-command runs with optional PID, sensors,
actuators, collision stopping and timed faults. It does not yet provide navigation
scenario execution, SLAM displays, batch comparisons or a graphical world editor.
The existing Python/CLI research workflows remain available. This is a local
single-user application; see [service boundaries](local-api.md) before deployment.
