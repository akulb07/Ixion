# Encoder-controlled shaft braking

```sh
python examples/esp32_diff_drive/encoder_brake.py
```

This is a first host Python feedback experiment. One reference motor accelerates
an assumed 0.01 kg m² shaft load. Ideal A/B encoder signals are decoded through
project-wired GPIO32/33 reads. At 40 observed counts, the controller latches the brake command for
the remaining run. It reads counts, not the motor model's true angle or speed.
The example assumes 20 encoder cycles per output revolution and a 5 ms control
interval, for two seconds of simulation time.

The shaft keeps moving after braking starts. Forty counts is the trigger, not a
guaranteed stopping position. No position controller or stopping-distance planner
is implemented. The JSON includes the trigger time, observed counts, applied mode,
speed, battery state, assumed parameters and original component evidence.

## Incremental runtime

`ShaftRuntime.step(duration_s, inputs)` accepts one `DriverInputs` value per motor.
Each command is held for the entire interval. The returned `ShaftStep` includes
start and end telemetry and applied commands. Omitting inputs retains the previous
commands. Time, displacement, speed and battery charge carry forward. A failed
step commits none of those updates, so callers can inspect the error and retry.
The runtime stores only current state; the caller controls recording and run length.

The existing explicit-Euler shaft bench supplies the actual physics step. Its
assumptions, convergence requirements and unsupported regenerative states still
apply. Reducing PWM while spinning may demand regeneration and raise an error;
the runtime does not invent a battery sink or silently clip current.

In the example, mechanics advances first, then the interval's interpolated encoder
edges are delivered in time order. Their counts inform the next command. A
threshold reached inside an interval therefore affects the following interval.
GPIO replay uses local interval timestamps; recorded samples and brake time use
absolute simulation time. Callbacks have zero simulated execution cost.

This is a host-side feedback loop for an independent shaft. It does not run
compiled ESP32 firmware, model CPU instruction timing, drive the full project net
graph, or close a rover navigation loop. A shared event scheduler and hardware-style
sensor buses remain future work.

## MCU outputs to the driver

`TB6612BoardOutputs(project, "esp32", "driver")` resolves seven TB6612 control
inputs through direct project nets. The reference maps AIN1/AIN2/PWMA to
GPIO25/26/27, BIN1/BIN2/PWMB to GPIO18/19/23, and shared STBY to GPIO13.
`write(digital={...}, pwm={...})` updates logical output latches. Digital values
must be booleans and PWM values must be finite duty fractions from zero to one.
The whole batch is validated before any latch changes.

`inputs("A")` or `inputs("B")` returns the resulting driver command. Both channels
share STBY. Initial latches are explicitly low with zero PWM; this is a model
default, not an ESP32 boot-state claim. The adapter enforces DIGITAL_OUT and PWM
assignments and rejects project wiring errors. It exposes unresolved warnings,
which the example includes in its JSON along with input and output pin maps.

The example enables standby and forward drive through GPIO writes, then sets
GPIO26 high to brake when its encoder counter reaches the target. Driver commands
are sampled at the start of each physics interval. Unsupported truth-table states
still fail when the driver evaluates them. Intermediate writes are not electrical
switching transients; PWM remains an averaged duty value.

The example also uses `resolve_motor_connection(project, "left_motor")` to follow
the motor P/N nets to one complete TB6612 channel. It records the resolved driver,
channel and polarity in `motor_connection`. Swapping P/N reverses the applied
voltage convention and shaft direction while preserving brake/coast/standby.
The reference uses AO1-to-P and AO2-to-N, giving channel A with positive polarity.

Both motor terminals must connect directly to the same driver's AO1/AO2 or BO1/BO2
pair. Cross-channel wiring, branched nets and unsupported driver types are rejected.
This supports the project's passive P/N motor-terminal convention; it does not
infer terminal roles from arbitrary names or discover the mechanical frame of a
mounted motor. Shaft angle and torque remain in the motor's positive frame.

The controller still writes fixed reference GPIO pins. Moving the motor to channel
B without changing those writes will leave it controlled by B's existing levels;
the resolver does not silently rewrite the controller. A reversed shaft may never
reach this example's positive count target, which is an observable wiring issue.
Regulators, power-on sequencing, interface chips and fan-out are still unsupported.
This remains a host Python controller.
