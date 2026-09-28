# RoboForge (robotics lab project)

So this is a differential-drive robot sim and a place to try some robotics ideas.
It has grown a bit over time. I keep adding little labs when something works.
Some parts are in much better shape than others; the model notes explain the
assumptions when it matters.

**Version: 0.24.0.** Saved LiDAR runs now support occupancy maps built with encoder
estimates or labelled ground-truth poses. See [the release roadmap](docs/roadmap.md)
for the 24 completed checkpoints and 8 remaining milestones.

## Getting it to run

Python 3.12+. From this folder:

```sh
python -m venv .venv
# Windows: .venv\Scripts\Activate.ps1
python -m pip install -e ".[dev,plot]"
roboforge validate configs/foundation.yaml
python -m pytest
```

For the browser workspace, install the optional extra and start the local server:

```sh
python -m pip install -e ".[api]"
roboforge serve --output results/service
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
roboforge simulate configs/sensors.yaml --output results/sensors --plot
roboforge experiment experiment.yaml --output results/runs
roboforge replay results/runs/<experiment-id>/<trial-id> --time 1.25
roboforge benchmark --output results/benchmarks --seed 42 --iterations 500
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
