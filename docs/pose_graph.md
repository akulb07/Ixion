# Pose graphs and geometric loop closure

PoseGraph stores SE(2) poses and relative-pose constraints. For source i, target j
and measured relative pose z, residual is the translation and wrapped heading of
z^-1 * (pose_i^-1 * pose_j). Analytic source/target Jacobians are verified against
central finite differences. Information matrices must be symmetric positive
definite. The first pose is fixed, removing all three global gauge freedoms; a
disconnected graph is rejected. The dense solver is capped at 200 nodes.

Damped Gauss-Newton minimizes Huber loss on each whitened three-dimensional edge
residual. Step acceptance requires non-increasing robust cost. The result reports
converged, stalled or iteration_limit, all accepted costs, and corrected pose
snapshots. Optimization does not silently replace the input graph's poses.

The bounded batch SLAM backend forms relative odometry factors between keyframes.
Loop candidates need temporal separation and estimated spatial proximity. ICP
must converge with sufficient overlap, low RMSE and a bounded correction. At most
one candidate per target is accepted. Accepted measurements become loop factors;
all proposals and failures are retained. After optimization, every stored scan is
reintegrated into a fresh occupancy map using its corrected historical pose.

Default information matrices represent stated tuning assumptions, not calibrated
ICP uncertainties. Geometric verification and Huber weighting do not eliminate
perceptual aliasing or guarantee rejection of a false loop. Repetitive geometry,
large prior drift, occlusion and dynamic objects remain difficult. There is no
descriptor retrieval, global relocalization, sparse optimizer, online marginalizer,
or hardware-time execution claim. This is a small static-world batch backend that
can replace or complement the incremental front end through measurement contracts.
