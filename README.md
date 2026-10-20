# Ixion

This started because we broke a robot and wanted to test the next version before
spending more money on parts. Simulating movement was useful, but it didn't tell
us whether the motors, driver and battery would actually work together.

That's what I'm trying to build with Ixion: pick the parts, connect them, run the
code and see how the robot behaves before building it for real.

It's still a work in progress. The 2D simulator and experiment workspace work.
The [hardware workspace](docs/hardware-workspace.md) now opens the reference rover
in the browser, with part and wiring edits, sizing estimates, design checks and
saved experiments. You can replay a run in 3D alongside its telemetry, add a
payload and compare the same tests, then export both runs in one offline report.
The wheel size and spacing use the project inputs; the board shapes and layout
are still placeholders.
The hardware side can check some wiring and electrical constraints, and a small
host Python controller can now brake a moving rover from simulated echo pulses.
It's a straight-line fixture with assumed parts and geometry; the rest of the
hardware workflow is still being built. The
[plan and architecture notes](docs/hardware-architecture.md) explain where this is going.

This used to be called RoboForge, so that name still shows up in the Python code.
Install this checkout with `python -m pip install -e .`
to get the `ixion` command. The `roboforge` command, Python imports and existing
run formats remain compatible. The repository URL has not changed.

For a [starter rover](docs/starter-rover.md) with its first four checks and an
offline viewer:

```sh
ixion new-project my-rover --demo
```

Open `my-rover/results/rover-suite.html`. The templates ship with the installed
package, so this works outside the source checkout too. They're still assumed
parts and geometry, with no claim that the physical robot has been validated.

The [first rover experiment](docs/first-rover-experiment.md) walks through adding
a payload, comparing the same cases and taking the saved design to the bench.

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

That command checks one operating point. There's a separate [battery model](docs/battery-model.md)
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
you can change. Turning and wheel slip still need work.

The same hardware loop can now run on a [constant uphill grade](docs/rover-inclines.md).
There's a supplied level/5°/15° suite with motion, current, traction and tilted
IMU readings. It checks the declared cases, including clearance; keeping the
rover parked on a slope is still separate work.

For a quicker [motor and battery sizing check](docs/rover-sizing.md), there's now
`ixion rover-size`. Give it a speed, acceleration, slope and battery charge to
estimate required PWM, current and traction margin. It keeps failed demands and
missing ratings visible, so a passing calculation isn't a build-ready verdict.

The moving rover also accepts [initial battery charge and a pack-side auxiliary
load](docs/rover-power.md). The starter has a full/half-charge suite, with voltage,
charge and extra-load traces. The load keeps drawing current when the motors brake.

There are [declared voltage-limit cases](docs/rover-voltage-limits.md) too. They
keep the last motion state when supply voltage reaches a chosen boundary, and
the suite can check final speed, energy and limit time. A boundary isn't a BMS
simulation or proof that the rover stopped.

You can also [keep the rover moving after that event](docs/rover-power-response.md)
with a latched coast or brake command. The starter has both cases, so it's easy
to compare the extra travel and see when a run ends still moving. This keeps the
supplies modeled; it doesn't simulate disconnecting the battery.

There's now a [sampled voltage-feedback demo](docs/rover-voltage-feedback.md) too.
It compares fresh, slow, stale and clipped readings, so you can see how sensing
and controller timing change braking. It's an ideal monitor with explicit
conversion time and quantization; actual MCU ADC wiring is still ahead.

You can also try [gain, offset and repeatable noise](docs/rover-voltage-errors.md)
to see a high reading miss a low-voltage condition or a low reading brake early.
The new starter suite keeps the model voltage and controller reading together.

Add `--encoder-cycles 20` to get ideal A/B encoder pulses for each wheel as well.
That assumes 20 cycles per wheel revolution; the [encoder notes](docs/quadrature.md)
explain the count convention. The example also counts those pulses through logical
GPIO callbacks using the MCU pins in the wiring, so the output includes counts
read from the A/B signals and the resolved pin map. There is also a small
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
a scripted target gets close.
The reusable ultrasonic runner can also compare braking on a missing echo with
continuing after a timeout, and records which condition caused braking.

You can now run these from a scenario file:

```sh
ixion ultrasonic-feedback examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/missing_echo.yaml --output missing-echo-report.json
```

There's also an `obstacle.yaml` scenario in the same folder. Edit a copy to try
different target distances or timeout policies; the report keeps the run inputs.

The new [rover wall-brake fixture](docs/rover-brake.md) connects both motors, rover
travel, encoder signals and echo feedback in one run:

```sh
ixion rover-brake examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_wall.yaml --output rover-report.json
```

It drives toward a fixed wall and tries to brake before the rover's front reaches
it. Try `rover_late_brake.yaml` to see a threshold that leaves too little room.
The report keeps distance, speed, clearance, encoder counts and power estimates.
It now sends trigger pulses and reads echoes through the project's MCU pins.
The selected echo interface is an ideal logical model; ESP32 binaries, analog
sensor interfaces and real-world validation are still ahead.

You can also [swap the rover controller](docs/rover-firmware.md) from Python.
There's an example that brakes from encoder counts instead of the wall distance:

```sh
python examples/esp32_diff_drive/rover_encoder_firmware.py --target-count 6000 --output encoder-rover.json
```

It uses the same moving rover and sensors. There's also a controller that ramps
both motors from reduced power before letting sonar request braking:

```sh
python examples/esp32_diff_drive/rover_pwm_firmware.py --initial-duty 0.25 --output pwm-rover.json
```

The first command now determines the initial current and traction estimate too.
Only equal forward PWM and a latched brake are supported; steering, reversing
and a general firmware runtime are still ahead.

There's now a first [encoder speed controller](docs/encoder-speed.md) too:

```sh
python examples/esp32_diff_drive/rover_speed_firmware.py --target-speed 0.2 --output speed-rover.json
```

It estimates wheel speed from the encoder counts and adjusts PWM toward a ramped
target. Sonar still handles braking. The saved report includes target versus
measured speed and power requests. The gains are example settings, not tuned
values for a real rover.

You can try breaking the feedback too:

```sh
python examples/esp32_diff_drive/rover_speed_firmware.py --encoder-fault both-freeze --output frozen-counts.json
```

This freezes the count readings at 0.8 seconds while the simulated wheels keep
moving. The controller notices the lack of progress and brakes. There are also
one-wheel freeze and count-jump cases. The [fault notes](docs/encoder-faults.md)
explain what gets changed and what the checks can miss.

To repeat the normal run and all three fault cases together:

```sh
ixion rover-suite examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_feedback_suite.yaml --output rover-suite.json
```

The [suite](docs/rover-suite.md) checks the expected brake reason, clearance and
fault response time, then saves every run and check result in one file.

To look through that file in a browser:

```sh
ixion rover-report rover-suite.json --output rover-suite.html
```

The [offline viewer](docs/rover-viewer.md) puts speed, counter readings, PWM and
current on a shared timeline. Click a fault or brake event to inspect what was
recorded at that time. It reads the saved report without rerunning the rover.

There's also a first [design comparison](docs/rover-comparison.md). To see what
an extra kilogram does to the assumed reference rover:

```sh
python examples/esp32_diff_drive/compare_payload.py --extra-mass-kg 1 --output results/payload-check
```

It runs the same cases on both designs and saves the changed inputs, metric
differences and an HTML viewer. The numbers are estimates from this fixture,
so a passing comparison still needs to be checked against a physical robot.

When you want the design details in one place for the bench:

```sh
ixion build-reference examples/esp32_diff_drive/robot.yaml --output build-reference.zip
```

The [build reference](docs/build-reference.md) includes parts, pins, declared nets,
parameter sources and a blank measurement checklist. Missing part details stay
visible. It doesn't generate board firmware or a verified wiring guide.

There is now a first [bench-trace comparison](docs/bench-traces.md) too. It takes
a CSV recording and explicit timing alignment, then compares speed, voltage,
current or travel with a saved run. Missing values stay missing. A synthetic
example lets you try the import before collecting physical measurements:

```sh
python examples/esp32_diff_drive/synthetic_bench_check.py --output results/synthetic-bench
```

You can now save [error and coverage limits](docs/bench-acceptance.md) and check
that evidence locally or in CI with `ixion rover-bench-check`. Missing data stays
inconclusive, and a policy expecting measured data rejects a synthetic trace.
The included synthetic policy only demonstrates the workflow.

To share a recording and its checks, [bench bundles](docs/bench-bundles.md) keep
the original inputs in one ZIP. The recipient can recompute the results offline
with `ixion verify-bench-bundle` and export a fresh viewer.

There's a first [I2C IMU fixture](docs/imu-i2c.md) now too:

```sh
python examples/esp32_diff_drive/imu_check.py
```

It wakes the MPU6050 model through the project's bus, then reads acceleration,
rotation and temperature bytes from known inputs. It records the bus timing and
decoded values. This checks a small register subset. To read the IMU while the
rover moves and brakes, use the combined scenario:

```sh
ixion rover-brake examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_wall_imu.yaml --output rover-imu-report.json
```

Its IMU readings now come from rover acceleration, with gravity included in the
assumed aligned sensor frame. I2C responses, encoder edges and sonar share a
timeline. The report keeps when a sample was captured and when the read arrived.

Controller code can now react to those completed IMU readings too:

```sh
python examples/esp32_diff_drive/rover_imu_firmware.py --max-acceleration 2 --output imu-controller.json
```

This example brakes when forward acceleration reaches the chosen limit and
ignores stale readings. Sonar still handles the wall and missing echoes as a
fallback. The limit is just an example setting; the parts and sensor model still
need checking against a real rover.

For [debug messages on the same timeline](docs/serial-console.md), try
`rover_wall_debug.yaml` instead. It records controller, sonar and selected IMU
messages with their simulated transmit times. A slow or full console can drop
messages without holding up the rover; unfinished output stays marked as pending.
Add `--log-decisions` to the IMU controller example to log its own explanations
too. These messages share the same console queue and keep the callback time, so
you can see the decision separately from the later motor command.

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
