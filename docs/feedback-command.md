# Run the wired shaft experiment

After installing the checkout, from the repository root:

```sh
ixion encoder-feedback examples/esp32_diff_drive/robot.yaml --inertia 0.01 --target-count 2400 --seconds 2 --step 0.005 --output feedback.json
```

This runs one shaft with the project's motor, driver wiring, battery and mounted
encoder. The host Python controller drives forward, reads quadrature transitions
through mapped MCU inputs, then latches braking after the positive count target.
The target is a brake trigger. Rotation continues while the shaft slows down.

Three pieces are now available:

1. `resolve_mounted_encoder` follows the motor's single outgoing `encoder_shaft`
   link. It requires an encoder with integer `cycles_per_rotor_revolution` and
   `edge_multiplier=4`. Rotor angle is output angle multiplied by motor gear ratio.
   The reference gives 12 cycles × 4 edges × 100 ratio = 4,800 counts per output
   revolution. These component values are still illustrative, not measured.
2. `run_encoder_feedback(project, FeedbackConfig(...))` runs the reusable experiment.
   It resolves motor lead polarity, driver control pins and encoder MCU pins. It
   holds commands per interval, decodes events, then makes the next control decision.
   The final interval is shortened when needed. Limits are 10,000 steps and 100,000
   edges per run; failures return no complete report.
3. `ixion encoder-feedback` saves the report as JSON. It includes the full project,
   assumed inertia, run settings, wiring maps, encoder mount, validation diagnostics,
   count/speed/angle/current/SOC samples and brake-command time. Existing files are
   refused. The output's parent directory must already exist.

Use `--motor right_motor` for the other reference channel. `--mcu` and `--battery`
select explicit components; the battery is not automatically discovered from power
nets. Inertia is required and refers to the output shaft. Encoders on output shafts
with differently named resolution parameters are not inferred. Positive mechanical
encoder orientation is assumed; mounting handedness is not represented yet.

Unlike the earlier fixed-pin example, this controller discovers its pin choices
from the project. Moving a complete channel to different MCU pins therefore moves
the controller's writes too. Reversing motor leads still reverses motion and may
prevent the positive target from being reached. The original `encoder_brake.py`
example remains useful for testing firmware that uses fixed pin names.

Status is `brake_triggered`, `target_not_reached`, or `target_reached_at_end` when
the target arrives too late for a following control interval. CLI exit codes are
0 for a triggered brake, 3 for a completed experiment without that trigger, and 2
for invalid inputs or runtime/file errors. Exit 0 does not mean the shaft stopped.

This remains an independent-shaft estimate. It does not simulate rover traction,
power-net activation, CPU instructions, interrupt latency or compiled firmware.
Logic supply remains an explicit 3.3 V assumption. Check smaller steps for numerical
convergence. The earlier `--encoder-cycles 20` example uses an override assumption;
this command uses the actual resolution declared in the project instead.
