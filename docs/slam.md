# Incremental scan matching and SLAM front end

This checkpoint implements the first SLAM stage: local scan-to-map matching and
incremental occupancy reconstruction. SlamBackend is a structural update protocol;
IncrementalSlam implements it without a simulator/world dependency. Inputs are
LiDAR and odometry priors at the same capture time. Initial map coordinates are
anchored to the first supplied prior; there is no absolute localization guarantee.

Valid LiDAR hits are transformed from the sensor mount into base coordinates.
Point-to-point ICP finds nearest target points within a distance gate, trims the
largest residuals, and solves a rigid alignment by centering plus SVD. A determinant
check excludes reflections. Success requires a small transform increment; residual
RMSE, pair count and iteration history remain inspectable. Too few correspondences,
collinear/collapsed geometry and iteration exhaustion have explicit failure results.
Nearest-neighbor search is dense and bounded at 2,000 points per cloud.

The frontend propagates the last corrected pose using relative odometry, matches
against accumulated accepted endpoints, and bounds RMSE and correction magnitude.
Rejected scans propagate odometry but do not update occupancy or the point map.
Accepted scans update the log-odds grid. A voxel filter and deterministic subsampling
bound map points; the occupancy map has its separate cell budget. This is a local
method: it needs overlap and a close prior, can settle into incorrect local minima,
and does not establish statistically calibrated pose covariance.

No loop detector, global relocalization, pose graph optimization or automatic
historical map correction is claimed in this first stage. Those remain explicit
next SLAM stages. The demo evaluates drift against truth externally and exports
all matches, including failures; truth is never passed to the backend.
