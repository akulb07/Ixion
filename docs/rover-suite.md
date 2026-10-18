# Repeating the rover checks

Instead of running each feedback demo separately, there's now a small acceptance
suite. From the repository root:

```sh
ixion rover-suite examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_feedback_suite.yaml --output rover-suite.json
```

The supplied suite runs normal feedback, a left counter freeze, both counters
freezing and a left count jump. Each case starts from rest with a full battery and
fresh controller, clock and sensor state. It uses the same project for all cases.
The four reference cases pass their declared checks; this is simulation evidence,
not a claim that the reference parts have been physically validated.

## Declaring cases

The file is bounded YAML or JSON with `schema_version: 1`, a `name`, and `cases`.
Each named case contains a complete `scenario` in the normal
[rover format](rover-brake.md), followed by optional controller settings and an
`acceptance` policy. Keeping complete scenarios makes the saved input independent
of other scenario files. Duplicate keys, unknown fields and executable YAML tags
are rejected. YAML merge keys aren't supported.

`controller` is `encoder_speed` by default, or `sonar` for the existing constant
forward-power fixture. The speed controller accepts `target_speed_m_s`,
`target_ramp_s` and optional `encoder_health` thresholds. Set `encoder_health: null`
to compare without feedback checks. These speed-specific settings have no effect
on the sonar controller. Resolution and wheel radius always come from the project;
other PI settings retain the reference defaults and appear in each run's report.

Acceptance settings are:

| Field | Meaning |
|---|---|
| `expected_status` | Exact fixture outcome; defaults to `stopped_before_wall`. A deliberate contact experiment can explicitly expect `wall_contact`. |
| `expected_brake_reason` | Optional exact reason, such as `obstacle` or `encoder_no_progress`. |
| `minimum_front_clearance_m` | Optional lower bound on final front clearance. |
| `maximum_peak_pack_current_a` | Optional upper bound on the peak sampled pack current. This inherits the simulation's sampling and electrical assumptions. |
| `maximum_fault_detection_delay_s` | Optional upper bound from the first actual injected counter-fault activation to the speed controller's first feedback-check trip. Requires a fault in the case. |

Limits are inclusive. Missing requested measurements fail their checks. A sonar
stop cannot substitute for a feedback-check trip in the fault-delay check. For
different fault times on each wheel, delay is measured from the earliest actual
activation, not a guessed cause of the eventual trip. The individual report keeps
the full activation and fault evidence so that interpretation stays visible.

The suite supports 1–16 uniquely named cases, a total budget of 40,000 requested
physics steps and a 128 KiB input limit. Each case keeps its existing 10,000-step
limit. Cases execute sequentially. There is no general scheduler, arbitrary
controller loading, component swapping or automatic gain tuning here.

## Results

The JSON report has its own schema version 1, the complete project and suite
configuration, an overall status and counts of passed, failed and error cases.
Every successful simulation includes its full rover report, controller evidence,
and each check's actual value, expected value and result. A model error is saved
as an error case without a partial run report, and the next case still runs.
Passing all checks means only that the declared simulated criteria were met.

Exit code 0 means every case passed, 3 means a check failed or a case hit a model
error, and 2 means invalid input or a file error. Failed suites still save their
evidence. Existing output files are never replaced. Use a fresh filename each time.

## Running it in CI

Add `--junit rover-suite.xml` to the command to write CI test results as well as
the JSON evidence. Each rover case becomes one test. A failed acceptance check
is a test failure; a simulation/model error is a test error. The XML keeps every
check's actual and expected values. Simulation duration isn't reported as test
execution time. These results describe the declared simulated checks only.

JSON and XML paths must be distinct, new, and different from the input files.
Both are checked before simulation starts. Files are written sequentially, so an
I/O failure or a competing writer can leave only the first output; the command
reports a file error in that situation. Existing files are never overwritten.
