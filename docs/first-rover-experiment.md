# A first useful rover experiment

Say the rover needs to carry another kilogram. Before buying different motors,
try the same mission with the extra mass and compare what changes. This example
uses assumed parts and a straight-line model, so the result is a design estimate
to check on the bench.

## Get a baseline

With Ixion installed from the checkout:

```sh
ixion new-project my-rover --demo
cd my-rover
```

Open `results/rover-suite.html`. The normal case and three counter-fault cases
have already run. Select a case, click an event and inspect the motion, observed
and raw encoder counts, PWM and pack current at the same time.

A passing fault case means the declared response checks passed. It doesn't mean
the fault never happened. The recorded outcome and the check results are both
visible.

## Try the extra payload

Copy `robot.yaml` to `robot-payload.yaml`. In that copy, change only
`assembly.total_mass_kg` from `2.3` to `3.3`. Keep the remaining assembly fields,
links and component definitions. This represents an assumed extra kilogram on
the chassis; it doesn't add measured payload properties or change the wheel inertia.

Run the same suite and compare it with the saved baseline:

```sh
ixion rover-suite robot-payload.yaml rover_feedback_suite.yaml --output payload-suite.json --junit payload-suite.xml
ixion rover-compare results/rover-suite.json payload-suite.json --output payload-comparison.json
ixion rover-report payload-comparison.json --output payload-comparison.html
```

Open `payload-comparison.html`. The design changes should identify the mass
change. Look at speed tracking, sampled current, stopping clearance, response
times and terminal energy. The deltas are candidate minus baseline, with units.
Select either design to inspect its own events and exact recorded samples.

Check the actual run durations before interpreting energy: a run that ends
earlier may consume less energy without being a better design. A pass says the
candidate meets the common suite limits; it doesn't rank every design tradeoff.
The suite deliberately refuses comparisons with different mission/controller
settings or acceptance policies.

## Keep the design details with the experiment

```sh
ixion build-reference robot-payload.yaml --suite rover_feedback_suite.yaml --case normal --output payload-build-reference.zip
ixion verify-build-reference payload-build-reference.zip
```

The packet includes the saved design, parts, parameter evidence, pins, logical
wiring and selected controller settings. Its blank checklist is a place to record
the test setup and measurements. Missing ratings stay visible. It isn't a verified
shopping list, physical wiring plan or ESP32 binary.

## Close the loop with a real recording

Once a physical rover exists, record speed, distance, voltage or current in the
[bench CSV format](bench-traces.md), with its instrument and operating conditions.
Select the matching saved suite case and set the clock offset and sample-age
allowance from the actual recording setup. Inspect rejected rows as well as the
paired errors. Don't change the model just to make those errors disappear.

Decide your [error and evidence limits](bench-acceptance.md) before the experiment.
Use a policy expecting measured data for a physical comparison. The included
synthetic policy only demonstrates the import and checking workflow.

For someone else to review it, a [bench bundle](bench-bundles.md) keeps the
original inputs, policy and timing together. They can recompute the acceptance
and generate their own report offline. Instrument authenticity, uncertainty and
the suitability of your limits still need an engineering review.

Turning, wheel slip, arbitrary firmware and a full 3D robot are further work.
This walkthrough exercises the current straight-line reference loop.
