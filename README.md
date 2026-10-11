# Ixion

This started because we broke a robot and wanted to test the next version before
spending more money on parts. Simulating movement was useful, but it didn't tell
us whether the motors, driver and battery would actually work together.

That's what I'm trying to build with Ixion: pick the parts, connect them, run the
code and see how the robot behaves before building it for real.

It's still a work in progress. The 2D simulator and experiment workspace work.
The hardware side can describe components and check some wiring and electrical
constraints, but it can't run the full firmware-to-motor loop yet. The
[plan and architecture notes](docs/hardware-architecture.md) explain where this is going.

This used to be called RoboForge, so that name still shows up in the Python code.
Install this checkout with `python -m pip install -e .`
to get the `ixion` command. The `roboforge` command, Python imports and existing
run formats remain compatible. The repository URL has not changed.

```sh
ixion inspect-project examples/esp32_diff_drive/robot.yaml
```

This checks references, required connections, source conflicts and I2C topology.
It also checks [declared electrical limits and MCU assignments](docs/electrical-checks.md).
The example still has missing part ratings, so expect warnings. Its numbers are
starting assumptions, not a tested shopping list.

There's also a first [DC motor model](docs/motor-model.md). You can give it a
voltage and shaft speed and look at the estimated current, torque and losses:

```sh
ixion motor-point examples/esp32_diff_drive/robot.yaml --component left_motor --voltage 6 --rpm 150
```

It doesn't move the robot yet. There's now a separate [battery model](docs/battery-model.md)
for voltage sag and charge use too:

```sh
ixion battery-step examples/esp32_diff_drive/robot.yaml --component battery --current 2 --seconds 60
```

The first [motor-driver model](docs/motor-driver.md) connects driver commands to
the motor estimate. Try `python examples/esp32_diff_drive/driver_check.py` to
compare driving, braking and coasting. A [shared battery/motor estimate](docs/powertrain.md)
now accounts for both motors loading the same supply:

```sh
python examples/esp32_diff_drive/power_check.py
```

There's also a small rotating-shaft bench now:

```sh
python examples/esp32_diff_drive/shaft_check.py --seconds 1 --step 0.01
```

It lets the motors speed up while current and battery charge change. The inertia
is an assumption you can change with `--inertia`; it isn't a moving rover yet.
There's a [straight-line rover bench](docs/rolling-model.md) too:

```sh
python examples/esp32_diff_drive/rolling_check.py --seconds 1 --step 0.001
```

It connects both motors to the rover mass and estimates distance and speed on
level ground. Traction, weight distribution and rotating inertia are assumptions
you can change. Turning, wheel slip and the firmware loop still need work.

Add `--encoder-cycles 20` to get ideal A/B encoder pulses for each wheel as well.
That assumes 20 cycles per wheel revolution; the [encoder notes](docs/quadrature.md)
explain the count convention. The example also counts those pulses through logical
GPIO callbacks using the MCU pins in the wiring, so the output includes counts
read from the A/B signals and the resolved pin map. A live
rover firmware control loop is still ahead. There is now a small
[encoder-controlled shaft braking example](docs/shaft-runtime.md):

```sh
python examples/esp32_diff_drive/encoder_brake.py
```

It runs a Python controller that brakes after reading enough encoder pulses,
using the project's MCU input pins and digital/PWM wiring to the driver.
This is an independent motor bench, with assumed inertia and ideal signals.

For a reusable version that reads encoder mounting and resolution from the project:

```sh
ixion encoder-feedback examples/esp32_diff_drive/robot.yaml --inertia 0.01 --target-count 2400 --output feedback.json
```

The [feedback command notes](docs/feedback-command.md) explain the saved report,
count units and limits. Use a fresh output filename for each run.

`--control-every 10` lets you see what happens when the controller checks less
often. Reports include response delay, estimated speed and energy use.

The [ultrasonic examples](docs/ultrasonic-model.md) now turn known target distances
into echo pulses, measure them through a simulation clock, and brake a shaft when
a scripted target gets close. The full moving-rover version still needs work.
The reusable ultrasonic runner can also compare braking on a missing echo with
continuing after a timeout, and records which condition caused braking.

**Version: 0.29.0.** Experiments and planner benchmarks now export standalone HTML
reports and trial CSV files. Browser sweeps have the same downloads. See
[the release roadmap](docs/roadmap.md) for the historical simulator checkpoints
and new hardware phases.

## Getting it to run

The existing [acceptance checks](docs/regression-checks.md) work with saved
RoboForge runs in the comparison workspace or through `roboforge check`.
Use [portable check bundles](docs/portable-checks.md) to send a saved check and its
run artifacts to another machine with `roboforge bundle` and `roboforge bundle-check`.
Both acceptance commands support [JUnit reporting for CI](docs/ci-checks.md)
through `--junit`.
The optional [MCAP inspector](docs/recordings.md) inventories real recording topics,
message types and container timing without decoding or changing the original log.
The [ROS 2 odometry extractor](docs/recorded-odometry.md) converts a selected CDR
odometry topic to JSON estimates with original timestamps, frames and covariances.

Python 3.12+. From this folder:

```sh
python -m venv .venv
# Windows: .venv\Scripts\Activate.ps1
python -m pip install -e ".[dev,plot]"
ixion validate configs/foundation.yaml
python -m pytest
```

For the browser workspace, install the optional extra and start the local server:

```sh
python -m pip install -e ".[api]"
ixion serve --output results/service
```

Open [http://127.0.0.1:8765/](http://127.0.0.1:8765/). The page is in the wheel,
so you do not have to install Node to use it. The `/docs` link is the API docs.

## What is in here

There is exact and Euler differential-drive motion, a few kinds of collision
checks, wheel actuators and PID, noisy/delayed encoders, IMU and LiDAR, wheel
odometry, a small EKF, mapping and SLAM pieces, and several route planners.
The simulator can use prescribed wheel commands or follow a planned route using
encoder odometry. The planning tab has a **Run this path** button now. Saved runs
can be compared from the two-column icon in the activity bar: pick a baseline,
check the metrics and changed settings, and export the comparison.

Some examples worth looking at:

```sh
python examples/milestone_8/run.py   # a little encoder-based navigation run
python examples/milestone_12/run.py  # loop closure / pose graph
python examples/milestone_14/run.py  # faults and encoder bias
python examples/milestone_15/run.py  # planner benchmarks
```

There are more in `examples/` and config files in `configs/`. A few old validation
notes live beside newer ones in `docs/`; they are kept for the checkpoint history.

## Runs

```sh
ixion simulate configs/sensors.yaml --output results/sensors --plot
ixion experiment experiment.yaml --output results/runs
ixion replay results/runs/<experiment-id>/<trial-id> --time 1.25
ixion benchmark --output results/benchmarks --seed 42 --iterations 500
```

Runs keep their configs, measurements and exports. Replay reads the saved result
instead of running the simulator again. Example configs and the CLI are probably
the easiest place to start; the full [workspace notes](docs/workspace.md) are a bit
more detailed than this page.

## What's next

- The [model notes](docs/) go into equations, frames, sensors and limitations.
- Milestone history is in [progress.md](docs/progress.md). It has not been
  tidied into a product roadmap.
- The [planning workspace](docs/planning-workspace.md) has A*, Dijkstra, RRT and
  RRT*. The UI uses a VS Code-style layout with setup on the left, the world in
  the middle, and state/history on the right.
- [Run comparisons](docs/comparing-runs.md) show saved outcomes, including failed
  trials. [Parameter sweeps](docs/sweeps.md) run a small batch across settings
  and seeds, then show group statistics and links to every recorded trial.
- SLAM views, hardware and dynamic obstacles still need work. Force/contact
  physics is not simulated.

This is a local, single-user service with no authentication. Don't put it on a
shared server as-is. See [service notes](docs/local-api.md).
