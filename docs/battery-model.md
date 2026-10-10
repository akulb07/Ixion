# Battery model

The battery model estimates voltage sag and charge use for a constant discharge
current. The separate [powertrain solver](powertrain.md) now calculates the coupled
motor/driver load at a fixed SOC and prescribed shaft speeds.

```sh
ixion battery-step examples/esp32_diff_drive/robot.yaml --component battery --current 2 --seconds 60 --soc 1
```

The example starts at an assumed 8.4 V open circuit. At 2 A through the assumed
0.15 ohm pack resistance, terminal voltage starts at 8.1 V. One minute consumes
0.0333 Ah from the assumed 2.2 Ah pack. Current ratings are missing, so the report
warns that continuous and burst capability are unknown.

## What is being calculated

```text
terminal voltage = open-circuit voltage(SOC) - current * internal resistance
charge used (Ah) = current (A) * time (s) / 3600
new SOC = old SOC - charge used / pack capacity
terminal power = terminal voltage * current
internal loss = current^2 * internal resistance
```

This is a simple ohmic equivalent circuit with coulomb counting; more complete
[equivalent-circuit battery models](https://www.mathworks.com/help/simscape-battery/ref/batteryequivalentcircuit.html)
also account for transient and temperature behavior. Ixion currently omits those.

The Python `BatteryModel` accepts a piecewise-linear OCV table covering SOC 0 to 1,
with strictly increasing SOC and nondecreasing voltage. Chemistry can be LiPo or
Li-ion, but is only descriptive: the supplied curve determines behavior. The CLI
component adapter uses explicitly supplied `empty_ocv` and `full_ocv` parameters
as a two-point curve. The reference values are illustrative assumptions, not a
measured curve or a recommended battery cutoff. Nominal voltage is not used as a
substitute for missing OCV data.

Capacity, resistance, current limits and voltages are all **whole-pack** values.
They are not multiplied by the component's series/parallel cell counts. The model
does not check individual cells or their balance. Input parameters and evidence
references are retained in the CLI report.

`discharge` returns a new immutable state and start/end operating points. Peak
current is retained across positive-duration steps. Steps that exceed available
charge or collapse terminal voltage below zero are rejected without changing the
old state. At exact charge exhaustion, the end point has zero current. That is an
ideal charge boundary, not a simulated BMS switch. Further positive discharge is
rejected. Negative current is unsupported rather than treated as battery charging.

The reported runtime is **charge-limited runtime at the current load**. It ignores
cutoff voltage, load changes, aging, temperature and rate-dependent capacity, so it
can overestimate usable runtime. At zero load it is absent, not infinity.
Current-limit diagnostics do not clip current or simulate protection. Burst-duration
limits are not modeled. A successful CLI exit means the estimate was produced;
it does not mean the requested load is within the battery's ratings.
