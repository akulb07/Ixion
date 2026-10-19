# Hardware prototyping direction

Ixion should help test a robot before buying its parts. The next release goal
is one ESP32 differential-drive assembly whose firmware, electronics, motors,
physics and sensors form a closed loop. A schema check alone is not that release.

## Existing code assessment

The package is a Python `src` distribution with Pydantic/PyYAML boundaries,
NumPy numerical routines, an optional FastAPI service and a React/TypeScript
workspace. `RunConfig` describes a bounded run, not a hardware assembly.
The legacy `Simulator` creates a fresh clock, controller, actuator, fault and sensor state.
It advances prescribed wheel rates through custom planar kinematics and optionally
stops at swept circular-footprint collisions. There is no PyBullet or MuJoCo
backend in this repository. In that legacy path, mass properties are descriptive;
they do not determine motor loading, and actuator response cannot predict current
or torque. The newer `hardware/` path below supplies separate motor, powertrain
and straight-line rolling estimates.

Sensors have separate capture/delivery times and seeded noise streams. Encoders
emit counts; the IMU is planar; neither exposes MCU pins or registers. PID and
navigation consume delivered readings. Saved artifacts, replay, provenance,
comparison and regression checks already provide useful experiment boundaries,
but their configuration and telemetry contracts assume the existing simulator.

| Decision | Modules | Treatment |
|---|---|---|
| KEEP | `core`, `geometry`, `robotics/differential_drive`, `control`, sensor noise, collision/raycast math | Preserve numerical tests and reuse where their assumptions apply. |
| KEEP | `experiments`, `io`, `replay`, `reports`, `comparison`, `regression`, `bundles`, `provenance`, `ci_reports` | Preserve current formats; add hardware run adapters later. |
| ADAPT | `config`, `robot`, `simulation`, `actuators`, `sensors` | Add a hardware project boundary alongside the existing run path. Separate physical state from peripheral signals. |
| ADAPT | `service`, `api`, `batch_service`, frontend | Eventually accept hardware projects and typed telemetry without equations in the UI. |
| FREEZE | planning/benchmarks, tracking/navigation labs, localization, odometry, mapping, pose graphs, SLAM and their analysis panels | Keep runnable demos and tests; no new algorithms to drive the architecture. |
| FREEZE | MCAP inspection and recorded sensor extraction | Preserve useful import work; defer more import features until the hardware loop works. |
| REPLACE | No existing module yet | Add an electromechanical actuator path rather than changing the meaning of old wheel-rate commands. |

## Smallest boundary change

Add `hardware/` with a versioned, serializable `RobotProject`. Components compose
identity, pins, explicit-unit parameters and mechanical links; they do not inherit
large electrical/mechanical class trees. References are typed component/pin pairs.
Each net is a canonical electrical connection with two or more endpoints. A pin
belongs to at most one net. Multiple wire segments should be merged into a net
before serialization, so connectivity does not depend on drawing order.

Use explicit SI field suffixes for fixed geometry and `{value, unit}` parameters
for catalog data. This first format accepts a small documented unit vocabulary;
it does not parse arbitrary expressions or silently convert units. Specialized
motor/battery parameter schemas will enforce dimensions when those models land.
Model parameters retain assumed/datasheet/measured provenance and a reference.

Schema integrity errors reject loading. Design diagnostics report problems in a
well-formed graph separately. A check with no detected graph errors means only
that the implemented rules found none. Declared electrical limits and reference
connectivity now have checks, but a clean graph does not establish dynamic supply
behavior, thermal safety, current protection or mechanical performance.

Component definitions are inline and self-contained for now. No executable plugin
loading, online registry or firmware execution occurs while loading a project.
The reference assembly will become the first local catalog once its parts have
verified model contracts. A backend contract will be introduced with the first
torque-driven consumer, not filled with unimplemented CAD/physics operations.

## Reference assembly contract

The reference project names `esp32`, `driver`, `left_motor`, `right_motor`,
`left_encoder`, `right_encoder`, `imu`, `ultrasonic`, `battery`, `regulator_5v`,
`echo_level_shifter`, `left_wheel`, `right_wheel`, `chassis` and `caster`.
It uses a 2.3 kg total assembly, 0.0325 m wheel radius and 0.18 m track.
Motor data must distinguish gearbox output RPM/torque from rotor values. Initial
6 V, 300 RPM, 1.8 A stall, 0.25 N m output stall torque and 100:1 gearing are
illustrative assumptions, not a verified N20 SKU. Encoder counts are specified
per rotor revolution with a separate edge multiplier.

The intended nets connect driver inputs to GPIO25/26/27 and GPIO18/19/23,
standby to GPIO13, I2C to GPIO21/22, encoders to GPIO32/33/34/35, and ultrasonic
trigger/echo to GPIO16/17 through an explicit echo level interface. All grounds
share one net. Battery feeds VM and a 5 V regulator; the board's 3.3 V output
feeds logic and the IMU. The 2S battery must be evaluated across its charge range,
not just nominal voltage. Direct battery drive of assumed 6 V motors is a design
risk to evaluate, not a wiring endorsement. Board/breakout variants, pull-ups,
decoupling, protection and regulator/level-interface selection remain required.

ESP32 input-only pins are represented as such, per the
[Espressif GPIO reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html).
The example is an abstract netlist, not a build-ready schematic.

## Implementation checkpoints

The [product priorities and release gates](roadmap.md#what-would-make-this-worth-using)
sit alongside these technical checkpoints. In particular, completing the software
loop does not replace validation against a physical reference robot.

1. **A1:** Project schema, explicit units, pins/nets, mechanical links, I2C graph,
   structural diagnostics, reference assembly and CLI inspection. Round-trip and
   bad-reference tests; keep the old simulator suite passing.
2. **A2:** Verified part definitions and electrical rules: supply/ground paths,
   pin mode assignments, voltage ranges, logic levels and bus pull-ups. Add a
   minimal backend protocol together with a headless consumer.
   Declared voltage-envelope, digital-level and MCU-assignment checks are now
   implemented ([usage and limits](electrical-checks.md)), together with explicit
   ground-reference connectivity checks. Verified part ratings, ground impedance,
   source activation and pull-up analysis remain pending.
3. **B:** DC motor equations, driver truth table/loss model, battery sag/capacity,
   wheel torque coupling and quadrature events. Analytical zero/free/stall/reverse
   tests plus current/power accounting before claiming useful estimates.
   The first [quasi-static DC gearmotor model](motor-model.md) and `motor-point`
   CLI are implemented, along with a separate [discharge-only battery model](battery-model.md)
   for OCV, sag, charge consumption and current diagnostics. A first [TB6612 channel model](motor-driver.md)
   now drives the motor estimate with PWM, brake and high-impedance modes. A
   [shared-supply solver](powertrain.md) couples battery sag to one or two channels.
   A bounded fixed-step discharge trace now advances SOC and recomputes electrical
   loads. An independent shaft bench now integrates speed and angular displacement
   from motor torque, explicit inertia and signed load torque while advancing battery
   charge. A [straight-line rolling bench](rolling-model.md) adds vehicle mass and
   rejects excessive contact-force demand. Turning, slip dynamics, general chassis
   dynamics remain pending. Ideal quadrature edge traces now come from sampled
   shaft angles, with explicit resolution and interpolated event times. Runtime
   GPIO replay now delivers those edges to logical pin reads and callbacks.
   Direct encoder wiring now maps to MCU input pins with assignment checks.
   Incremental shaft steps now accept changing commands; a host Python example
   brakes from decoded encoder counts through project-wired MCU reads and TB6612
   digital/PWM control outputs. Motor P/N nets now resolve channel and lead polarity.
   An incremental rolling runtime now accepts equal changing commands while
   preserving distance, wheel angle and battery state. General rover control,
   turning and slip remain pending; Phase B is still partial.
   A [rover sizing solve](rover-sizing.md) now inverts the same motor/rolling
   equations for prescribed speed, acceleration, uphill grade and SOC, including
   shared battery sag, duty bounds, traction and current-reference screening.
   Unknown ratings remain unassessed; this is one operating point rather than
   a complete trajectory or physical capability claim.
4. **C:** GPIO/PWM/I2C timing, MPU6050 register subset and HC-SR04 trigger/echo.
   Test against deterministic physical inputs and document unsupported registers.
   Ideal [ultrasonic timing](ultrasonic-model.md) now converts supplied distances to
   echo pulses, with short-trigger, cycle and timeout handling. Digital pulse capture
   works on a simulation clock. The rover fixture now routes trigger and echo
   through MCU nets, with a direct echo path or one explicitly selected ideal
   non-inverting interface. Analog conversion and general acoustics remain pending.
   A first [MPU6050 register subset and wired I2C adapter](imu-i2c.md) now supports
   identity, power/range configuration and coherent motion-byte reads from supplied
   physical samples. Sequential transactions have explicit start/completion times.
   The optional rover IMU now receives aligned body specific force at physics
   endpoints. Its reads complete on the shared GPIO timeline. Sensor sampling,
   filtering and arbitrary sensor mounting remain pending.
5. **D:** Host firmware runtime with simulation-time scheduling and serial output.
   No claim of ESP32 binary compatibility.
   A bounded digital event clock now supports callbacks and pulse capture. The
   shaft runner supports slower control ticks and count-based speed observations;
   the rover coordinator now schedules I2C completion callbacks alongside GPIO,
   including chained initialization and nonblocking IMU polls. A general user
   firmware scheduler and CPU timing remain pending. A bounded
   [logical serial console](serial-console.md) now schedules UTF-8/8N1 message
   completions on the same clock, with explicit queue drops and pending output.
   Physical UART/USB and serial input are not executed.
   A [host rover controller interface](rover-firmware.md) now separates brake
   policy from the fixture. It passes immutable range observations and decoded
   counts to trusted Python callbacks, then latches the first brake request until
   a control tick applies it. An encoder-based replacement example shares the
   same mechanics and peripherals. Optional `on_imu` callbacks now receive
   completed decoded frames with immutable vectors and sample/delivery times.
   An acceleration-limit example ignores stale frames and retains sonar fallback;
   callbacks don't depend on serial logging. Callback responses can now submit
   bounded debug messages alongside a brake request. Messages share the serial
   queue; rejection doesn't cancel braking, and reports count queued/rejected
   controller messages. Equal forward PWM requests now apply on control ticks,
   with pending-request metadata, held outputs and latched-brake priority. The
   first tick sets initial power before electrical and traction estimates are
   calculated. A ramped-start example keeps the sonar brake policy; power changes
   that need a regenerative sink still fail explicitly. Arbitrary actuator commands, controller-issued
   peripheral requests, process isolation and CPU scheduling are still pending.
   A [reference encoder speed controller](encoder-speed.md) now estimates wheel
   speed from count windows and adjusts equal forward PWM with illustrative
   PI/feedforward gains, duty bounds, slew limits and conditional anti-windup.
   Sonar braking freezes its integral and stops further drive requests. A saved
   controller trace distinguishes target, observed speed and requested duty.
   Optional [encoder feedback checks](encoder-faults.md) now latch a brake from
   reverse counts, excessive estimated speed, persistent no-progress or wheel
   disagreement. Counter-read freeze/offset injection leaves GPIO decoding and
   physical motion intact; saved reports keep both count layers and trip evidence.
   These checks don't diagnose the physical cause or provide recovery.
   Calibration, stability across parts and real-board execution remain pending.
6. **E:** Headless obstacle-response demo: firmware changes PWM after observing an
   echo, with encoder and power telemetry and repeatable integration tests.
   A scripted target fixture now triggers shaft braking through measured echo width.
   Its reusable runner records missing echoes and can brake on timeout; target
   schedules and controller policies are explicit inputs.
   A further [fixed-wall rover fixture](rover-brake.md) now joins two equal motors,
   straight-line travel, mounted encoder edges and ideal echo feedback. A host
   Python controller brakes on a measured distance or timeout. The report includes
   clearance, speed, encoder counts, power and controller timing, with separate
   stopped-before-wall and first-contact outcomes. Sonar GPIO now follows the
   declared wiring, with an ideal logical echo interface and recorded pin edges.
   Analog interfaces and the whole electronics graph are not executed. This is a partial Phase E
   checkpoint; the general runtime and physical reference validation remain open.
7. **F/G:** Engineering design estimates, readiness categories, component swaps,
   CLI templates and portable reports with explicit assumptions.
8. **H:** Extend the existing dark IDE workspace after the loop is validated.

MuJoCo is the first backend to evaluate: its joint transmissions, applied forces
and stepping API fit torque-driven experiments ([official computation docs](https://mujoco.readthedocs.io/en/stable/computation/)).
This is a candidate, not an integrated or benchmarked dependency. The evaluation
must test wheel contact, slip, slope behavior, fixed-step determinism and Windows
headless installation. Electrical behavior stays in Ixion. No second physics
engine is needed until there is a concrete compatibility requirement.
