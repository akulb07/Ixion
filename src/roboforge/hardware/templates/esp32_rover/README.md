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
