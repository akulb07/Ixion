# Hardware prototyping direction

Ixion should help test a robot before buying its parts. The next release goal
is one ESP32 differential-drive assembly whose firmware, electronics, motors,
physics and sensors form a closed loop. A schema check alone is not that release.

## Existing code assessment

The package is a Python `src` distribution with Pydantic/PyYAML boundaries,
NumPy numerical routines, an optional FastAPI service and a React/TypeScript
workspace. `RunConfig` describes a bounded run, not a hardware assembly.
`Simulator` creates a fresh clock, controller, actuator, fault and sensor state.
It advances prescribed wheel rates through custom planar kinematics and optionally
stops at swept circular-footprint collisions. There is no PyBullet or MuJoCo
backend in this repository. Mass properties are descriptive; they do not determine
motor loading. The current actuator response cannot predict current or torque.

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
that the implemented rules found none: electrical operating ranges, supply paths,
thermal/current limits and mechanical performance remain unassessed.

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
4. **C:** GPIO/PWM/I2C timing, MPU6050 register subset and HC-SR04 trigger/echo.
   Test against deterministic physical inputs and document unsupported registers.
5. **D:** Host firmware runtime with simulation-time scheduling and serial output.
   No claim of ESP32 binary compatibility.
6. **E:** Headless obstacle-response demo: firmware changes PWM after observing an
   echo, with encoder and power telemetry and repeatable integration tests.
7. **F/G:** Engineering design estimates, readiness categories, component swaps,
   CLI templates and portable reports with explicit assumptions.
8. **H:** Extend the existing dark IDE workspace after the loop is validated.

MuJoCo is the first backend to evaluate: its joint transmissions, applied forces
and stepping API fit torque-driven experiments ([official computation docs](https://mujoco.readthedocs.io/en/stable/computation/)).
This is a candidate, not an integrated or benchmarked dependency. The evaluation
must test wheel contact, slip, slope behavior, fixed-step determinism and Windows
headless installation. Electrical behavior stays in Ixion. No second physics
engine is needed until there is a concrete compatibility requirement.
