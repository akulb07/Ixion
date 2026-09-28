# Mapping a saved run

Open a recorded run and scroll below odometry to **Mapping · Occupancy grid**.
Choose a LiDAR, pose source and cell size. The JSON export contains cell states,
probabilities, observed flags, grid configuration, run ID and scan counts.

Encoder mode uses saved encoder reconstruction with the configured initial pose
and wheel dimensions. Scans need a matching capture-time estimate (within 1 ns
for floating-point timestamps). Other scans are skipped and counted, with no
interpolation or extrapolation. Use compatible encoder/LiDAR rates. Only
readings delivered by the run end are used. This is offline analysis: a later
delivery can supply a matching pose if both readings arrived before run end.

Ground-truth reference mode explicitly uses saved true capture-time poses.
It helps isolate mapping error from localization drift. Neither mode is SLAM.

Both modes reuse OccupancyGrid. Rays add free-space log-odds evidence and hit
endpoints add occupied evidence; occupied evidence wins overlap within a scan.
Evidence is clamped, and missing rays contribute nothing. Mount translations
and rotations are applied. Grey means unknown/uncertain, pale means free, and
copper means occupied. Row zero is at the bottom of the displayed map.

The grid starts at (0, 0), covers the world and rounds up to whole cells. Limits:
10,000 cells, 50,000 input rays and 5,000,000 conservative ray/cell traversal
operations. Over-budget requests fail before mapping. Use a coarser grid or a
shorter/lower-rate run when needed.

`GET /api/runs/{id}/map?sensor=lidar&pose_source=encoder&encoder=encoders&resolution=0.1`
returns the report without resimulation or editing saved artifacts. Set
`pose_source=truth` for reference mode. If no estimated poses match, the result
is an unknown map with all skipped scans counted.
