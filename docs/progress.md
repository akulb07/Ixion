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
  a 7.686 m path with 24 waypoints. This checkpoint only previewed the route.
- 19 — encoder-guided route execution in the simulator, saved navigation samples
  and a separate estimated pose in replay. 282 tests / 157 subtests.
- 20 — compare saved runs with a baseline, configuration differences and JSON
  exports. Failed runs stay visible and unavailable measurements stay empty.
- 21 — bounded parameter/seed sweep jobs, previewed trial designs, group
  statistics, cancellation and saved reports in the browser workspace.

- 22 — individual measurement plots and seed-paired comparisons in the sweep
  workspace. Reports retain measurements, signed differences and exclusion reasons.

- 23 — offline encoder-odometry view, matched-time truth comparison, drift
  metrics and JSON export. Missing initial measurements are rejected explicitly.

- 24 — occupancy-map workspace with encoder or explicit truth poses, capture-time
  matching, skipped-scan counts and portable cell/probability exports.

- 25 — offline encoder/gyro EKF workspace, paired odometry errors, adjustable
  noise assumptions, gyro acceptance counts and model-based position uncertainty.

- 26 — saved-run scan matching / SLAM inspection, accepted map points, per-scan
  residuals and rejection reasons, paired error metrics and JSON export. Also
  fixed Windows export failures caused by replacing an already published setup.

- 27 — editable wheel PID settings, feedback laboratory, per-wheel response
  and contribution plots, full-data tracking metrics and saved analysis export.
  Browser bundles now use content versions to avoid stale UI after updates.

Current release plan: **27 of 32 milestones complete**; see [roadmap.md](roadmap.md).
Next up: fault editing and reproducible failure experiments. Arms and grippers remain a later manipulation phase;
the mobile-robot experiment workflow comes first. Test counts above are from
each milestone, not current totals.
