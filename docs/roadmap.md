# Where this is going

The direction is now a testing and debugging workbench for mobile-robot teams.
The 29 completed checkpoints below are the simulator foundation, not a claim that
the expanded product is nearly ready for industry use.

The next product phases are:

1. Saved acceptance policies and regression checks. The first run-pair workflow
   is implemented; scenario suites and navigation outcome rules still need work.
2. Reproducible experiment packages with source and dependency provenance.
3. Real MCAP/ROS 2 recording import, topic mapping and clock/transform diagnostics.
4. Synchronized debugging timelines, events and linked comparisons.
5. External Python algorithms, process isolation and ROS 2 adapters.
6. Repeatable navigation scenario suites and simulator integration.
7. Automated CI execution, JUnit results and review artifacts.
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
