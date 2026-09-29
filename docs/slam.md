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

The browser now has a SLAM inspection panel under the saved-run analysis views.
Run a sensor or wheel-slip experiment, then choose Analyze SLAM. The plot shows
the final accepted map points alongside truth, encoder odometry and the corrected
path. Selecting a scan shows its pair count, iteration residuals and rejection
reasons. Red dots mark rejected scans. The export includes the occupancy grid,
all processed scans, skipped capture times, settings and error metrics.

The offline endpoint is `GET /api/runs/{id}/slam`, with `sensor`, `encoder`,
`resolution` and `max_match_rmse` query parameters. Only measurements delivered
by run end are used, processed in capture order. Encoder priors must match scan
capture times within 1 ns; unmatched scans are listed as skipped. Errors for
SLAM and odometry use the same processed scan times, including rejected matches.
This reconstruction is offline, so it does not model online delivery-order latency.

The workspace uses 300 points per cloud, 25 ICP iterations, at most 200 scans,
10,000 grid cells, 50,000 rays, 5 million ray traversal steps and 100 million
pair-distance evaluations. Oversized requests return an explanation instead of
silently trimming the experiment. For larger runs, lower the LiDAR rate or use
a shorter experiment. Initialization anchors the map to the supplied encoder
prior; a successful local match is not proof of globally correct localization.
