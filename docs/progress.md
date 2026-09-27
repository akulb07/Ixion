# progress notes / things that got added

I kept adding pieces as I got them working. Here's the rough order, plus the
test counts I had written down at each point.

- 01 — basic differential-drive simulator, simple configs. 73 tests / 110
  subtests at that point.
- 02 — robot descriptions, frames and swept collision checks. 117 / 153.
- 03 — seeded encoders, IMU and LiDAR with timestamps. 131 / 157.
- 04 — delayed, asymmetric wheel actuator response. 144 / 157.
- 05 — encoder feedback PID, anti-windup and derivative filtering. 155 / 157.
- 06 — encoder odometry and dropout handling. 165 / 157.
- 07 — Dijkstra, A*, RRT and fixed-radius RRT*. 179 / 157.
- 08 — encoder navigation example. 184 / 157.
- 09 — EKF with encoder/gyro fusion and landmark correction. 198 / 157.
- 10 — LiDAR occupancy mapping. 211 / 157.
- 11 — ICP and incremental scan-to-map SLAM. 217 / 157.
- 12 — loop closure and pose graph example. 224 / 157.
- 13 — seed/parameter experiments, retained failures and replay. 230 / 157.
- 14 — faults and encoder bias. 240 / 157.
- 15 — planner comparisons. 247 / 157.
- 16 — service/API foundations. 257 / 157.
- 17 — local browser workspace, replay and history. 258 / 157.
- 18 — path planning panel, bounded planning endpoint and a VS Code-style UI.
  273 / 157, plus the three frontend numerical checks. A* in the browser found
  a 7.686 m path with 24 waypoints. Planning is still a preview; it doesn't drive
  the robot along the route.

Next up: running navigation scenarios in the workspace, then the research
workflow UI. Test counts above are from each milestone, not current totals.
