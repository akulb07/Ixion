# Taking the saved design to the bench

The build-reference export puts the design's parts, pin assignments and declared
connections in one ZIP file. It doesn't run physics or compile board firmware.

```sh
ixion build-reference examples/esp32_diff_drive/robot.yaml --output build-reference.zip
ixion verify-build-reference build-reference.zip
```

To include the configuration of one experiment:

```sh
ixion build-reference examples/esp32_diff_drive/robot.yaml --suite examples/esp32_diff_drive/rover_feedback_suite.yaml --case normal --output rover-build-reference.zip
```

Choose a new output name. A multi-case suite needs an explicit case name; a
single-case suite can select its only case automatically. The chosen controller
must be configurable from this project. The exporter doesn't execute it.

The packet contains:

- `project.json`: complete saved design, including original parameter references.
- `parts.csv`: one row per component instance with quantity one, part metadata and
  an explicit list of missing metadata. It doesn't group unknown parts together.
- `parameters.csv`: declared values, units and assumed/datasheet/measured labels.
- `pins.csv`: every declared MCU pin, selected function, capabilities, net and
  connected endpoints. Unassigned functions and unconnected pins stay blank.
- `wiring.md`: logical nets and declared I2C addresses. This isn't a physical
  wiring, PCB, breadboard or soldering guide.
- `diagnostics.json`: the current graph and declared electrical-limit checks,
  including their explanations and suggestions.
- `bench-checklist.csv`: blank fields for measurements, uncertainty, instrument
  and conditions. It contains no measured data or automatic calibration.
- `README.md` and `manifest.json`: scope, missing metadata, selected case,
  design fingerprint and a size/checksum for every other file.

When selected, `suite.json`, `case.json` and `firmware-config.json` preserve the
experiment and host Python configuration. There is no ESP32 binary, flashing
step or claim of board timing compatibility.

## What verification means

The verifier reads the archive without extracting files. It checks file names,
membership, sizes, checksums, supported schemas and consistency of the generated
tables/configuration with the saved project. Duplicate entries, extra paths,
encrypted entries and nonregular file entries are rejected. Exports use fixed ZIP
metadata, so the same supported inputs produce identical archive bytes in the
same software environment.

The archive limit is 8 MiB plus 64 KiB of ZIP overhead. Individual entries are
limited to 2 MiB, and total uncompressed content to 8 MiB. Current packets contain
9 files without a case, or 12 with a case. Verification regenerates references
using the installed exporter format; incompatible future format changes need a
new manifest version.

Packets with default power conditions and level-only scenarios retain manifest
version 1 and its scenario encoding. Default-power [graded suites](rover-inclines.md)
use version 2. Suites with any [nondefault power conditions](rover-power.md) use
version 3, including when the selected case itself uses defaults. The current
verifier also supports version 4 for [declared voltage limits and operating bounds](rover-voltage-limits.md).
Suites with a [post-limit coast/brake response](rover-power-response.md) use version 5.
Suites with a [sampled voltage monitor](rover-voltage-feedback.md) use version 6,
including its conversion settings and the selected voltage-guard policy.
Suites with [declared voltage-sensor errors](rover-voltage-errors.md) use version 7
and retain gain, offset, noise bound and seed.
Each version preserves its generated
configuration instead of allowing an older verifier to ignore new conditions.

Checksums detect changed contents but don't authenticate who made the packet.
Verification doesn't establish engineering readiness. Declared part metadata and
evidence labels are not independently verified. The reference still has assumed
parameters and incomplete exact-part metadata; export doesn't turn it into a
shopping list or approval to wire or buy hardware.

Keep the original packet and fill measurements into a separate copy of the
checklist. Changing an exported file intentionally invalidates the original
checksums. CSV text that could be interpreted as a spreadsheet formula is prefixed
with an apostrophe; original strings remain in `project.json`.

Export returns 0 when the implemented design checks contain no errors, 3 when
they report design errors (the packet still saves those diagnostics), and 2 for
invalid inputs or file/model errors. Verification returns 0 for a consistent
packet and 2 for invalid evidence. Its success code concerns artifact consistency,
even if the enclosed design has errors. Existing outputs are never replaced.
