# LiDAR occupancy mapping

OccupancyGrid consumes an estimated base pose and a scan at the same capture time.
The caller supplies the static base-from-LiDAR mount (translation and yaw) and its
frame name. The mapper has no Environment, collision-world or robot-state access.
Online callers must buffer until scan and matching historical estimate are ready.
Duplicate/out-of-order scans and frame mismatches are rejected before mutation.

Cells are half-open regions starting at the configured origin, with array indices
[row=y, column=x]. Bounds are origin plus resolution times column/row counts.
The allocation budget is four million cells. Unknown log odds starts at zero.
An observed mask distinguishes untouched cells from cells with conflicting evidence.

Every beam is clipped to the map rectangle and partitioned at exact grid-line
crossings. Positive-length cell intersections get free evidence. A valid hit
endpoint within the map gets occupied evidence; an outside hit never marks the
clipped border as an obstacle. Max-range non-hits clear traversed cells only.
Invalid/dropout rays do nothing. This is an infinitesimally thin beam model:
corner touches alone add no cells, and beams on grid boundaries use half-open
membership. It is not a finite-width beam or supercover rasterizer.

Per scan, each cell gets at most one increment. Occupied evidence wins over free
evidence when beams overlap. Log odds adds log(p/(1-p)), default p_free=.3 and
p_occupied=.7, clipped to [-5,5]. States are -1 unknown/uncertain, 0 free, 100
occupied with probability thresholds .4 and .6. Observed but uncertain cells retain
their observed mask. Returned arrays are copies; snapshots use data-only NPZ.

Assumptions: static world, instantaneous scan, calibrated mount and ranges, and
accurate capture-time pose. Pose/range uncertainty is not spread over neighboring
cells. Repeated correlated scans can create overconfidence despite evidence caps.
No decay, dynamic obstacle tracking, map resizing or planning adapter is claimed.
