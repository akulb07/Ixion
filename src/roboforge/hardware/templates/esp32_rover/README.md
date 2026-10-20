# My first Ixion rover

This is the small ESP32-style reference rover from Ixion. It's a starting point
for trying the software. The parts and their values still include assumptions;
this isn't a tested shopping list or a physical wiring guide.

From this folder, with Ixion installed:

```sh
ixion inspect-project robot.yaml
ixion rover-brake robot.yaml rover_wall.yaml --output wall-run.json
ixion rover-report wall-run.json --output wall-run.html
```

Open `wall-run.html` in a browser to inspect motion, encoder counts, PWM, pack
current and events. The report is local and doesn't need an internet connection.
Use `rover_uphill.yaml` with new output names to try the supplied 5° uphill lane.
Travel is along the lane; the speed-threshold stop doesn't establish hill holding.

For a quick speed/acceleration and battery-demand calculation:

```sh
ixion rover-size robot.yaml rover_sizing.yaml --output sizing.json
```

The input asks for 0.3 m/s and 0.2 m/s² on a 5° slope at half charge. It prints
required PWM, pack current and traction margin, and saves the full assumptions
and checks. Missing current ratings stay unassessed. This is one operating point,
so it doesn't prove the rover can reach it or complete a mission.

To repeat the moving wall approach at full charge, half charge and half charge
with a 0.25 A pack-side auxiliary load:

```sh
ixion rover-suite robot.yaml rover_power_suite.yaml --output power-suite.json --junit power-suite.xml
ixion rover-report power-suite.json --output power-suite.html
```

The power-aware cases show battery voltage and charge alongside the other traces.
The auxiliary allowance keeps drawing current during braking. It is a constant
pack-side input, not an automatically calculated electronics load or a BMS model.

For declared voltage boundaries and energy/final-speed checks:

```sh
ixion rover-suite robot.yaml rover_voltage_limit_suite.yaml --output voltage-suite.json --junit voltage-suite.xml
ixion rover-report voltage-suite.json --output voltage-suite.html
```

The voltage boundary ends the estimate and keeps the last speed; it doesn't
simulate a physical disconnection or the rover coasting to a stop.

To keep the motion going with a declared driver coast/brake response:

```sh
ixion rover-suite robot.yaml rover_power_response_suite.yaml --output response-suite.json --junit response-suite.xml
ixion rover-report response-suite.json --output response-suite.html
```

The tight 8.39 V trigger is only there to make the assumed reference demo easy
to see. It isn't a physical battery cutoff setting. The brake case reaches the
speed threshold; the coast case deliberately ends still moving. Pack and logic
supplies stay modeled, and later firmware requests can't release the override.

To compare braking from fresh, slow, stale and clipped voltage readings:

```sh
ixion rover-suite robot.yaml rover_voltage_feedback_suite.yaml --output feedback-suite.json --junit feedback-suite.xml
ixion rover-report feedback-suite.json --output feedback-suite.html
```

The monitor is an ideal sampled pack-voltage fixture with quantization and
conversion time. It doesn't simulate an ESP32 ADC or divider circuit. The 8 V
demo threshold is an assumption. Open the report to compare reading delivery,
control-tick delay and the resulting motion.

For a reading that is biased high or low, plus repeatable noise cases:

```sh
ixion rover-suite robot.yaml rover_voltage_error_suite.yaml --output voltage-errors.json --junit voltage-errors.xml
ixion rover-report voltage-errors.json --output voltage-errors.html
```

The gain, offset and noise values are assumptions for trying the software. The
report overlays model voltage and delivered readings, so it's possible to see a
high reading miss the chosen voltage threshold or a low reading brake early.
It doesn't calibrate a physical sensor or simulate the ADC wiring.

For the normal case and three encoder faults:

```sh
ixion rover-suite robot.yaml rover_feedback_suite.yaml --output suite.json --junit suite.xml
ixion rover-report suite.json --output suite.html
```

If this folder was created with `--demo`, the suite and its viewer already live
in `results/`. Choose new output names for another run. Try changing a copy of
`robot.yaml` and comparing both saved suites with `ixion rover-compare`.

To take the declared design details to the bench:

```sh
ixion build-reference robot.yaml --suite rover_feedback_suite.yaml --case normal --output build-reference.zip
ixion verify-build-reference build-reference.zip
```

The starter manifest records the files as created and the Ixion version. It's
an initial snapshot, so editing files makes those original hashes stale. It
doesn't certify your edited design or authenticate physical measurements.

Only straight motion, equal forward commands and a latched brake are supported
by this fixture. It runs a host Python controller rather than ESP32 machine code.
Turning, wheel slip, arbitrary firmware and measured physical accuracy are still
being worked on. Check the assumptions, exact parts and electrical interfaces
before using anything from this example on a real robot.

Template source: Ixion (`akulb07/ixion`), under the included MIT license.
