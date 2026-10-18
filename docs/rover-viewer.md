# Looking through a saved rover run

The rover reports can be turned into a single HTML file. It works offline and
doesn't need the local service, Node, a chart library or a simulation rerun:

```sh
ixion rover-report rover-suite.json --output rover-suite.html
```

Open the HTML in a browser. This accepts the [acceptance suite](rover-suite.md)
report or an individual `rover-brake` / speed-demo JSON report. Use a new output
filename; the command never replaces an existing file or modifies the source.

The layout has a case list, result summary, shared time cursor, four plots and an
event list. The dark charcoal and purple styling follows the workspace. Select
a case to see its recorded outcome and acceptance checks, including failed checks
and model errors. A deliberately expected wall contact can pass its policy;
the physical fixture outcome remains visible separately.

The plots show physical wheel speed alongside the controller's measured speed
and target when available, firmware-observed versus raw decoded encoder counts,
driver PWM and battery current. Use the slider or click a plot to inspect a saved
sample. Readouts include travel, clearance, pack voltage and charge. The cursor
uses the latest saved physics sample at or before the selected time; it doesn't
interpolate a new physical state. A sample on a control boundary describes the
interval that just ended, before the next command takes effect.

Selecting a counter fault, brake, sonar, IMU or serial event moves the same cursor
and shows the saved event details. Serial completion time stays separate from
the original event/request time in its payload. Pending serial messages appear
at request time with their future completion time in the details; they aren't
presented as delivered. Filter the list to inspect one event type.

Plots use up to 1,500 points per series and join recorded points. Readouts retain
every saved physics sample. Use the original JSON for exact numerical analysis;
plot decimation can hide a brief peak. A driver PWM of one during a latched brake
is the short-brake command, not forward drive. The source JSON SHA-256 identifies
the imported bytes; it doesn't certify the report or validate physical accuracy.

The viewer supports rover schema 7, speed-control schema 2 and suite schema 1.
Inputs are limited to 64 MiB, 16 cases, 10,001 motion samples per case and 20,000
events per case. It checks the fields it displays and rejects nonfinite numbers,
duplicate JSON keys, unordered sample times, unsupported versions and count values
that JavaScript can't represent exactly. This is a projection of the report, not
a validator for every original field or a complete archive of its contents.

The HTML has no external resources. Report strings become text and escaped JSON;
an inline script hash in its content-security policy restricts executable code.
The original project/scenario JSON remains the reproducible input, and physical
validation, general replay controls and integration into the main IDE remain ahead.

For the small JavaScript interaction tests during development:

```sh
node --test tests/js/rover-report.test.cjs
```

These exercise timeline selection and case/event controls with a minimal DOM
fixture. They don't replace a real browser layout and accessibility review.
