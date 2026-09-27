# notebook scraps

Notes from my planner-panel work, 26 September 2026.

## 2026-09-26-ish — planner panel

- A* and Dijkstra are grid searches; RRT and RRT* sample. Same names, different
  assumptions, remember to say which one in a result.
- Plan is a static geometry calculation. The drawn purple path is only a preview.
  It doesn't send wheel commands and doesn't mean the robot can follow it.
- Extra clearance gets added to the footprint radius.
- The search budget is a work cap, not a time limit. A budget ending isn't proof
  that there is no route.
- Repro export should keep the resolved request and software version together.

## stuff to come back to

- This isn't dynamic obstacle planning.
- Planner runs aren't in simulation history yet; export the result if you want
  to keep it.
- Full source regression passed for milestone 18: 273 tests and 157 subtests.
- A friendlier way to place start/goal points would be nice. For now it's
  coordinates in the form.

## general reminders

- The simulator uses prescribed wheel commands; it isn't a full autonomous
  navigation system yet.
- Sensor capture time needs to stay separate from delivery time in replay.
- If a graph looks odd, check units before changing the math.
