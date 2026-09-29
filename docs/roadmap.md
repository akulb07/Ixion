# Where this is going

The working plan is 32 implementation milestones for the first mobile-robot
release. 28 are finished. This is a planning count, not a percentage of all the
features in the original specification. The milestones are different sizes.

The original expanded specification has 19 broader phases. The progress log
breaks work into smaller checkpoints, so the two numbers aren't interchangeable.
Core implementations exist across phases 1–16, but that doesn't mean every
requested feature in those phases is finished. Phase 17 (advanced visualization)
is in progress. Phases 18 and 19 (AI assistant and hardware/digital-twin bridge)
are later work.

Completed checkpoints 1–28 are in [progress.md](progress.md).

Remaining work for the first mobile-robot release:

- 29 — benchmark suite and experiment report exports.
- 30 — robot/environment setup editing and complete workspace workflows.
- 31 — installation, packaging and compatibility checks.
- 32 — release validation, examples, documentation and remaining usability fixes.

Arms and grippers are a later manipulation phase: joint/link models, kinematics,
3D collision and contact, grasping, motion planning and a 3D workspace. AI help,
ROS adapters and real hardware also need separate plans. They aren't included in
the 32-milestone mobile-robot release count.
