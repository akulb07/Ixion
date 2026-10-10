# Ixion roadmap

The priority is now virtual hardware prototyping. See the
[architecture assessment and Phase A–H plan](hardware-architecture.md).
Phase A has begun with a hardware graph, reference netlist and topology checks.
Declared electrical-limit and ground-reference checks work, and the first motor
and battery estimates run separately.
The motor/electronics/firmware closed loop is not implemented yet.

## What would make this worth using

The goal is to catch an expensive design mistake before buying parts. A long
feature list won't mean much if the reference robot doesn't behave convincingly.
These priorities guide the Phase A–H implementation plan; they are planned work,
not another count of completed milestones.

| Priority | Work | Evidence needed |
|---|---|---|
| 1. One complete robot | Finish the ESP32 → driver → motors → wheels → physics → sensors → firmware loop. Propagate gearing and battery changes through it. | A deterministic obstacle-response demo with current, motion and sensor telemetry. |
| 2. A real reference robot | Build the physical counterpart and compare current, speed, acceleration, stopping distance and runtime. | Publish the exact parts, test conditions, measurements, prediction errors and known gaps. Physical measurements are a release dependency, not something simulation tests can replace. |
| 3. Exact component variants | Record board/module variants, datasheet sources, supported behavior, missing ratings and user measurements. | Every part in the reference design has traceable inputs; assumptions remain visibly different from verified data. |
| 4. Useful diagnostics | Explain the affected connections, operating conditions, evidence and possible fixes. | Deliberately bad designs produce specific warnings without claiming that a predicted risk proves hardware failure. |
| 5. Easy design comparisons | Reuse experiments to compare motors, gearing, drivers and batteries under the same mission. Include speed, torque, current, runtime, mass and cost where data exists. | Saved baseline/candidate designs retain conditions and provenance; missing metrics stay unknown. Costs have a source and date. |
| 6. Hardware-like firmware interfaces | Expose encoder transitions, sensor registers and echo pulses through a documented peripheral subset. | Firmware reacts to simulated signals; supported host code and actual ESP32 binary compatibility are clearly distinguished. |
| 7. One debugging timeline | Link PWM, current, battery voltage, wheel speed, sensor readings and serial logs. Add pause, stepping, replay and fault injection. | Selecting an event reveals synchronized evidence; replay reproduces recorded data without silently rerunning the model. |
| 8. An easy first run | Ship a starter robot with minimal setup in the dark IDE workspace. Make units, errors and unsupported features visible. | A clean-install walkthrough reaches a successful reference run without undocumented steps. |
| 9. A path to the workbench | Export a parts list, pin table, wiring reference, firmware configuration and measurement checklist. Later, import measurements for calibration. | Exports match the saved design and flag unresolved choices. Calibration preserves the original model and shows its effect against measured data. |

## First hardware release gates

- The complete reference loop runs headlessly and has an integration test proving
  that firmware changes the motor command in response to a sensor signal.
- The reference parts and supported firmware/peripheral behavior are documented.
  Unknown ratings or unsupported behavior cannot turn into a readiness pass.
- Real-versus-simulated results are published. Define acceptable error bounds for
  each claimed use case before validation, and report failures as well as successes.
- The starter project, diagnostics, basic telemetry inspection and build exports
  work together. More advanced comparison and debugging tools can follow.
- A new user can install the release and repeat the demo from the instructions.

The release demo should be: open a working rover, swap in a weaker driver, see
the predicted issue and its assumptions, swap it back, then compare the simulation
with measurements from the physical rover. This is the target demo, not a claim
that it works today.

AI features, photorealistic graphics, cloud collaboration and a large parts catalog
stay deferred until this reference workflow is convincing. Calibration and richer
bench integration come after a measured baseline. The existing simulator labs
remain available while the hardware workflow becomes the main focus.

## Previous experiment-workbench direction (preserved, no longer the priority)

The previous direction was a testing and debugging workbench for mobile-robot teams.
The 29 completed checkpoints below are the simulator foundation, not a claim that
the expanded product is nearly ready for industry use.

The next product phases are:

1. Saved acceptance policies and regression checks. The first run-pair workflow
   and navigation goal checks are implemented; scenario suites still need work.
2. Reproducible experiment packages with source and dependency provenance.
   Portable recorded-check ZIPs and independent verification are implemented;
   worker-start source fingerprints, Git identity and core dependency versions are
   recorded for new service runs. Source archives, locked environments and simulation
   reruns remain to be added.
3. Real MCAP/ROS 2 recording import, topic mapping and clock/transform diagnostics.
   MCAP inventory and selected ROS 2 odometry extraction are implemented, preserving
   header timestamps, frames and covariance. IMU extraction is also implemented.
   LiDAR decoding, algorithm topic
   mapping and transform-aware replay are still pending.
4. Synchronized debugging timelines, events and linked comparisons.
5. External Python algorithms, process isolation and ROS 2 adapters.
6. Repeatable navigation scenario suites and simulator integration.
7. Automated CI execution, JUnit results and review artifacts.
   JUnit export is implemented for saved-run and portable-bundle acceptance checks.
   Automated scenario execution and CI-provider workflows remain separate work.
8. Real-versus-simulation validation and model calibration.
9. Installation, performance, recovery and team workflow hardening.

These phases need validation with engineers using their own data. The first target
is reproducing a known failure and comparing a fix faster than their current workflow.
Arms, advanced rendering and AI assistance follow this mobile-robot workflow.

## Original simulator release plan

The working plan is 32 implementation milestones for the first mobile-robot
release. 29 are finished. This is a planning count, not a percentage of all the
features in the original specification. The milestones are different sizes.

The original expanded specification has 19 broader phases. The progress log
breaks work into smaller checkpoints, so the two numbers aren't interchangeable.
Core implementations exist across phases 1–16, but that doesn't mean every
requested feature in those phases is finished. Phase 17 (advanced visualization)
is in progress. Phases 18 and 19 (AI assistant and hardware/digital-twin bridge)
are later work.

Completed checkpoints 1–29 are in [progress.md](progress.md).

Remaining work for the first mobile-robot release:

- 30 — robot/environment setup editing and complete workspace workflows.
- 31 — installation, packaging and compatibility checks.
- 32 — release validation, examples, documentation and remaining usability fixes.

Arms and grippers are a later manipulation phase: joint/link models, kinematics,
3D collision and contact, grasping, motion planning and a 3D workspace. AI help,
ROS adapters and real hardware also need separate plans. They aren't included in
the 32-milestone mobile-robot release count.
