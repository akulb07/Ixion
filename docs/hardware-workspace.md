# Hardware experiments in the workspace

Start the local app with `ixion serve`, then open `http://127.0.0.1:8765/`.
The robot hardware view now opens first. The earlier simulation, planning,
comparison and sweep tools are still available in the activity bar.

Choose a starter experiment to load the reference rover with encoder feedback,
power conditions, voltage delays, voltage errors or a voltage-limit response.
Select a part to inspect its parameters, units and evidence labels. Editing a
parameter marks that value as assumed, with a workspace-edit reference. Assembly
mass, shared wheel radius and track width are editable too. The assembly supplies
the wheel radius used by the reference runtime and its preview.

The **Assembly** tab now opens a 3D reference view with orbit, zoom, top view and
camera reset. Click a visible part to select it and inspect its declared values,
or use **Edit selected part** to open the parameter editor. Wheel radius and axle
spacing come from the assembly. Wheel width, chassis and board shapes, and all
part placement are illustrative: the project has no CAD geometry or mounting
poses yet. This view doesn't add 3D contacts or general robot dynamics. Rendering
uses bundled Three.js and requires WebGL 2; the other tools remain usable when
that preview is unavailable. Radius and track values outside the preview's
0.00001–1,000 m rendering range produce a message instead of an unbounded scene;
those are software display bounds, not part ratings.

In **Wiring**, select an electrical net to see its connected pins and their
declared capabilities. Add or remove pins and nets, or change an MCU function.
Edits stay pending until **Apply wiring changes** validates the whole graph, so
you can move a pin between nets in one transaction. A structurally invalid graph
stays unapplied. Electrical diagnostics remain visible even when the graph is
structurally valid. Running, setup export and other design edits are blocked
while wiring is pending. **Discard wiring edits** restores the applied graph;
**Undo applied wiring** restores the previous graph if no later edit replaced it.
The diagram shows logical membership, not physical wire routing or circuit physics.

In **Sizing**, enter a required speed, acceleration, uphill grade, battery charge
and explicit load/mechanical assumptions. **Calculate demand** uses the existing
rover sizing equations and retains failed demands, required duty/current/torque,
traction margin and supply estimates. Unknown ratings remain unassessed. Editing
the design or conditions marks the old estimate stale until recalculated.

**Pin comparison baseline** keeps an estimate while you change a design and
calculate again. Deltas are shown only for identical operating conditions. The
baseline/candidate project snapshots and assumptions stay in exported estimate
JSON. The in-memory estimates and pinned baseline aren't restored after a browser
reload. The constant-current charge estimate holds one instantaneous current
constant; it doesn't predict a mission, startup trajectory or cutoff behavior.

Use **Check design** for structural and declared electrical diagnostics. Missing
ratings remain warnings. These checks don't certify an assembly or establish
physical accuracy. In **Experiment**, adjust each case's wall distance, brake
distance, duration or controller interval. **Setup JSON** contains the complete
project, sensor settings, controller choices and acceptance policy. Apply that
editor before running; invalid edits don't replace the current applied setup.

**Run hardware suite** submits an independent snapshot to the local service. The
results tab shows its queued/running/completed state, case outcomes and the full
recorded evidence viewer. A completed suite can contain failed acceptance checks
or model-error cases. This is distinct from a service job that couldn't finish.
JSON and JUnit downloads retain the saved result. **3D replay + telemetry** uses
the saved run's project, independently of your current editor draft. Choose a case,
play at 0.25–5× speed, step through samples, scrub or click a trace to seek. Motion
and telemetry hold the last recorded sample at or before the cursor; event jumps
retain their precise delivery time, including between physics boundaries. Playback
ends exactly at the final recorded time and never invents post-contact motion.
Observed counters and decoded edge counts stay separate, and driver modes explain
the PWM values. Voltage readings appear only after their recorded delivery.

The 3D camera follows the rover's recorded distance along the declared level or
uphill fixture. Wheel spin is derived from distance/radius under its no-slip
assumption, not from counter readings. Shapes and placement remain illustrative;
this replay adds no physics. Positions beyond ±1,000,000 m are outside its display
bounds. **Full evidence report** retains the earlier detailed viewer. Both views
show saved evidence rather than rerunning a simulation; the replay includes its
source report hash and case acceptance results.

In **Compare**, choose a baseline and candidate from completed hardware runs.
The same named cases, controller settings and acceptance policies must match;
changing them requires a separate comparison. Saved suite failures and model-error
cases stay selectable. The table shows both outcomes and candidate-minus-baseline
deltas, with unavailable evidence kept separate from zero. A candidate pass means
it meets the common acceptance policy, not that every metric improved.

Design changes, assembly masses, source report hashes and normalized condition
identity remain inspectable. Open either source replay without loading its inputs
into the editor. **Comparison JSON** embeds both complete original reports and
their byte hashes; **Offline comparison HTML** includes the paired evidence viewer
and works without the server. The HTML's source fingerprint identifies the
corresponding JSON export. Combined JSON exports are limited to 128 MiB. The UI
comparison is kept in memory; use an export to retain it across reloads. Source
reports remain unchanged.

Browser storage retains the applied draft on the same local origin. Export the
setup as JSON for a portable copy; import validates the project and suite before
applying it. Unapplied JSON text isn't autosaved. Saved run inputs and reports
live under the service directory's `hardware/` folder. **Load inputs** restores
the selected run's setup into the editor. Later draft edits don't change that
completed run. Browser draft storage and server run history are separate.

The service runs one hardware suite at a time, with at most four active/queued
jobs and 200 saved jobs. The first UI history list shows the latest 100; the API
also supports pagination. Existing per-suite physics and case budgets apply;
requests are limited to 1 MB and reports to 64 MiB. Completed reports have
integrity hashes. Recovery keeps completed results, reports unreadable entries
through health warnings, and marks interrupted jobs failed without silently
rerunning them. Closing normally waits for the bounded queue to finish.

This workspace runs the existing straight-line fixture with built-in host Python
controllers. It doesn't load arbitrary Python from the browser or execute MCU
binaries. A general CAD assembly editor, 3D mechanics, real ADC circuitry and physical
reference validation remain ahead.
