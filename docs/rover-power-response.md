# What happens after the voltage limit?

The rover can keep simulating after a declared pack-voltage boundary. Set
`voltage_limit_action` to `brake` or `coast` to latch that driver command for the
rest of the run. This lets a stopping-distance experiment keep the rover's
momentum instead of ending the trace at the voltage event.

```sh
ixion rover-suite examples/esp32_diff_drive/robot.yaml examples/esp32_diff_drive/rover_power_response_suite.yaml --output response-suite.json --junit response-suite.xml
ixion rover-report response-suite.json --output response-suite.html
```

The supplied suite uses the reference speed controller. Its normal case brakes
from sonar. Two more cases use the same deliberately tight 8.39 V sag-screening
trigger: one brakes and one coasts. That number makes the behavior easy to see
with the assumed reference parts; it isn't a recommended battery cutoff.

With these inputs, both reach the trigger at about 0.98 s. The brake reaches the
0.005 m/s speed threshold after about 4.5 cm and 0.96 s more travel. The coast
travels about 27.5 cm after the event and is still moving at about 0.107 m/s when
the three-second horizon ends. These are simulation estimates from assumed
inputs, not measured stopping distances.

## Choosing a response

Add the fields to a complete rover scenario:

```yaml
minimum_pack_voltage_v: 8.39
voltage_limit_action: brake
```

| Action | Behavior |
|---|---|
| `end` | Default. End at the [declared boundary](rover-voltage-limits.md), retaining speed. |
| `brake` | Hold both TB6612 channel inputs high, with PWM high: the existing short-brake model. |
| `coast` | Hold both channel inputs low, with PWM high: the existing high-impedance coast model. |

A coast or brake action requires `minimum_pack_voltage_v`. Both keep STBY high
and retain the declared pack, auxiliary load and driver logic supply. The
response is an ideal fixture override, with no response delay. It is **not a BMS
disconnect or a controller reacting to a simulated voltage-sensor reading**.
The electrical model must remain within its supported supply and discharge
ranges afterward. Depletion, unsupported regeneration, slip or numerical
reversal remain model errors.

The first boundary latches the response. Voltage rebound and later firmware
requests don't release it. Host callbacks and logical peripherals continue;
control decisions retain the requested command and `motor_override` separately.
A sonar brake request can therefore appear during an already latched coast,
without silently changing that coast into a brake.

## Motion and timing

A crossing within a physics step splits that step. Its above-threshold lower
bracket edge becomes a recorded sample, the override starts there, and a fresh
Euler interval uses the remainder up to the original grid endpoint. Control and
sensor polls keep their original cadence. The response can begin up to the
recorded bracket tolerance before the mathematical crossing; this isn't a
continuous-time physical switching model.

The complete feedback loop also includes quantized encoder counts and discrete
control ticks. In a supplied-case refinement check at 1 ms, 0.5 ms and 0.25 ms,
the tight demo trigger occurred at 0.98 s, 1.00 s and 0.98 s respectively. Extra
brake travel was about 4.53, 4.64 and 4.52 cm; all met the illustrative criteria.
These event shifts aren't monotonic convergence. A separate analytic test checks
the linear brake/coast decay after the response and its step-refinement error.
Inspect whole-loop event changes when selecting your own bounds.

At a command boundary the last sample still describes the previous interval.
The proposed command and prospective voltage are in `power_limit_event`; the
replacement inputs are in `power_response`. The viewer shows both events at
that time. No duplicate motion sample or invented elapsed time is added. An
initial limit at rest can reach the stopped-speed threshold at time zero.

The outcomes stay separate:

- `stopped_after_power_limit`: the configured forward-speed threshold was reached.
- `power_response_incomplete`: the run horizon ended while still above it.
- `wall_contact`: the wall was reached; approach speed remains recorded.

An earlier wall contact or ordinary sonar stop can still end the run before a
voltage event. No response is fabricated in that case. Reaching the small
positive speed threshold doesn't establish static holding, particularly uphill.
Coasting includes the fitted motor viscous loss and gravity, but there is no
rolling-resistance or static-friction model.

## Checking the response

```yaml
acceptance:
  expected_status: stopped_after_power_limit
  minimum_front_clearance_m: 0.7
  maximum_final_speed_m_s: 0.005
  maximum_power_limit_time_s: 1.1
  maximum_distance_after_power_limit_m: 0.05
  maximum_power_response_elapsed_s: 1.1
```

These are the supplied brake case's illustrative criteria. The elapsed bound
requires the stopped-after-limit outcome, so an unfinished short run can't pass
as a quick stop. The distance bound requires an explicit response outcome. The
voltage-time bound still checks the conservative upper edge of the crossing
bracket. Missing requested evidence fails a check; all bounds are inclusive.

Response-configured runs use rover schema 11, including when the limit is never
reached. `power_response` records time, action, distance, accumulated terminal
energy and the two applied driver inputs. The extra metrics are
`distance_after_power_limit_m`, `energy_after_power_limit_j` and
`power_response_elapsed_s`. They are null without a response. Auxiliary energy
continues after the limit; a short brake doesn't imply zero total pack current.

Each later sample also keeps `interval_start`: the time, speed, SOC, driver
modes and starting pack/auxiliary power for the interval that ended at that
sample. The initial sample has no elapsed interval. Starting electrical power
can differ from the preceding sample at the same instant because the command
changed. The viewer checks interval energy against starting power times elapsed
time, then checks the accumulated energy and sampled peak current. Its cursor
shows the interval's start time and power alongside the endpoint state.

The offline viewer shows response events, final travel/elapsed cards and the
recorded driver modes at the cursor. A sample at the response time can still
show the preceding mode; later samples show the latched mode. Comparisons retain
the new metrics and require matching action/criteria in both saved suites.
Build-reference manifest 5 preserves this policy; manifests 1–4 still verify.
Individual `rover-brake` runs return zero for a stopped response, 3 for an
unfinished response or wall contact, and 2 for invalid input/model errors.

Next gaps are physical shutdown circuitry, delay/hysteresis/restart, a real
voltage-sensing firmware path and measurements against an actual rover.
