# Starting a rover project

With Ixion installed, this creates the bundled rover and runs its first checks:

```sh
ixion new-project my-rover --demo
```

Open `my-rover/results/rover-suite.html` to inspect the normal run and three
encoder-fault cases. The same folder has the complete JSON and JUnit XML.
No Node installation, repository example folder or network request is needed at
runtime. The template and viewer are included in the Python wheel. Install Ixion
from the checkout as described in the README; this doesn't imply a published
package on a package index.

To create just the input files, leave off `--demo`. The only template is currently
`--template esp32-rover`. The destination must be a new directory. Existing
folders or files are rejected, including an empty folder. Inputs and optional demo
outputs are prepared before creating it; a filesystem write failure can still
leave a partial directory, which should be inspected before retrying elsewhere.

The starter contains:

- `robot.yaml`: the assumed 15-component hardware graph, part parameters,
  provenance labels, MCU pins, electrical nets and mechanical links.
- `rover_wall.yaml`: the first fixed-wall brake experiment.
- `rover_uphill.yaml`: a constant 5° uphill case with body-coupled IMU readings;
  see the [incline limits](rover-inclines.md).
- `rover_feedback_suite.yaml`: normal feedback, left/both counter freeze and
  a counter-jump case, with declared simulation checks.
- `README.md`, `.gitignore` and the original MIT license for the template.
- `starter-manifest.json`: Ixion version, template identity, diagnostic counts
  and the initial files' sizes and SHA-256 hashes. With `--demo` it also includes
  the saved simulation results' fingerprints and case summary.

The files are ordinary editable inputs. The manifest records their initial bytes;
editing a file intentionally makes its original fingerprint stale. This isn't a
lockfile or a certificate for your changes. Keep modified designs and experiments
in version control, and choose new output names for later runs.

For example, from the new folder:

```sh
ixion inspect-project robot.yaml
ixion rover-brake robot.yaml rover_wall.yaml --output wall-run.json
ixion rover-report wall-run.json --output wall-run.html
ixion build-reference robot.yaml --suite rover_feedback_suite.yaml --case normal --output build-reference.zip
```

A passing demo only means the simulated cases meet their declared checks. The
template still has missing part ratings and assumed mechanics. It supports equal
forward commands and a latched brake on a straight-line fixture, with host Python
firmware. It doesn't generate a 3D robot, execute an ESP32 binary, verify a physical
wiring plan or demonstrate real hardware accuracy.
