# Ixion roadmap

The priority is now virtual hardware prototyping. See the
[architecture assessment and Phase A–H plan](hardware-architecture.md).
Phase A has begun with a hardware graph, reference netlist and topology checks.
Declared electrical-limit and ground-reference checks work, and the first motor
and battery estimates now have a shared-supply operating-point solver through
the first TB6612 channel model. Shared battery discharge and independent shaft
acceleration now run over simulation time. A straight-line rolling bench connects
motor torque to vehicle mass with a traction validity check. Turning, wheel slip
and general chassis dynamics remain pending.
Ideal encoder A/B edges now replay through logical GPIO inputs and callbacks in
simulation time. Direct encoder nets now resolve to MCU input pins, with reads and
interrupt callbacks checked against pin assignments. An incremental shaft runtime
now accepts changing commands, with a host Python encoder-triggered brake example.
The example now reads project-wired MCU inputs and writes digital/PWM outputs
through direct TB6612 control nets. Motor output nets now determine the connected
driver channel and lead polarity. A complete rover firmware loop and automatic
execution of the whole electronics graph remain pending.
Three further checkpoints are complete: rotor encoder mounting/resolution lookup,
a reusable wired shaft feedback runner, and an `encoder-feedback` CLI with saved
reports. These extend the partial hardware phases; they do not complete Phases B–E.
Controller cadence and count-based speed/response/energy metrics are implemented.
An ideal ultrasonic timing model and bounded digital signal clock now support a
scripted echo-triggered shaft brake fixture. Sensor voltage interfaces, general
scene-based ranging and a complete hardware runtime are still pending.
The ultrasonic shaft fixture is now a reusable runner with configurable target
schedules, explicit missing-echo behavior and selectable timeout braking policy.
The `ultrasonic-feedback` command now loads bounded YAML/JSON scenarios and saves
reports, with supplied obstacle and missing-echo cases and explicit outcome codes.
A further partial Phase E checkpoint now connects two equal motors, battery sag,
straight-line rover motion, project-wired encoder signals and ideal ultrasonic
feedback in a [fixed-wall braking fixture](rover-brake.md). The `rover-brake`
command saves telemetry and distinguishes a speed-threshold stop before the wall
from first contact or an unfinished response. The supplied early/late thresholds
make a useful comparison. Sonar trigger/echo signals now follow the reference
MCU nets through an explicitly selected ideal logical interface, and reports save
the actual GPIO edges. Pin assignments and supported paths are checked before
execution. Analog voltage conversion, arbitrary scenes, a general firmware
scheduler and physical validation remain pending. This does not complete Phase E
or the reference hardware release.

The first [MPU6050/I2C fixture](imu-i2c.md) now supports identity, wake/sleep/reset,
range selection and coherent motion-register reads from supplied physical inputs.
It resolves the declared bus pins and address, tracks sequential transfer timing,
and rejects unsupported registers and overlapping requests. This is a partial
Phase C checkpoint. The combined rover scenario now publishes its body
acceleration into the IMU at physics endpoints and delivers I2C responses on the
same timeline as encoder and sonar GPIO. Pending transfers remain pending at run
end, and nonblocking polls count busy skips. This adds a bounded reference
coordinator for partial Phases D/E. A bounded [logical serial console](serial-console.md)
now records controller and sensor events with request/transmit/completion times.
Nonblocking whole-write rejection exposes buffer drops without delaying motion;
unfinished output remains pending at run end. Internal sensor sampling/filtering,
physical UART/USB behavior, serial input and a general user-firmware scheduler
remain pending.

The rover now has a [host controller interface](rover-firmware.md) for replacing
the brake policy from Python. A separate encoder-count example runs against the
same motion and sensor loop. Range observations and decoded counts are immutable,
the first brake request latches, and output changes wait for a control tick.
Sonar timeouts now arrive on the shared clock at their exact deadlines. This is
another partial Phase D/E checkpoint. Equal forward PWM requests now support
variable motor power before a latched brake; arbitrary firmware, CPU scheduling
and steering remain pending.

The controller interface now also supports optional IMU callbacks at I2C completion.
The [acceleration-limit example](rover-firmware.md#using-the-imu-in-a-controller)
uses decoded forward specific force, rejects stale frames and keeps sonar fallback.
Pending reads stay pending at run end. This extends the partial Phase D/E work;
it doesn't add CPU timing or hardware-binary execution.

Controller callbacks can now return bounded debug messages alongside a brake
request. These use the shared serial queue, with callback timestamps, severity,
text and accepted/rejected counts. The IMU example can log its own decisions with
`--log-decisions`. Full or disabled consoles don't suppress braking. This adds
controller-authored output to the partial Phase D runtime; serial input and a
general scheduler are still pending.

A [ramped-start controller](rover-firmware.md#changing-motor-power) now changes
both motor PWM outputs on control ticks. The first command establishes initial
current and traction, and braking cancels pending duty updates. Reports preserve
applied duty, request time and pending requests. The discharge-only power model
still rejects reductions that require regeneration; restart, reverse and turning
remain outside this fixture.

A [reference encoder speed loop](encoder-speed.md) now regulates equal forward
PWM from delivered count-window estimates. Its PI/feedforward settings, bounds,
rate limit and anti-windup are explicit configuration. The demo records target,
measured speed and power requests, and retains sonar braking. This is further
partial Phase D/E work; physical tuning, broader mechanics, fault handling and
general firmware scheduling remain ahead.

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
