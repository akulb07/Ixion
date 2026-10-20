import { useEffect, useState } from "react";
import { post, request } from "./api";

type Config = Record<string, string | number>;
type Check = {
  code: string;
  status: string;
  actual: number | null;
  minimum: number | null;
  maximum: number | null;
  unit: string;
  explanation: string;
};
type Report = {
  status: string;
  config: Config;
  project: {
    name: string;
    snapshot: { assembly: { total_mass_kg: number } };
    sha256: string;
  };
  result: {
    required_pwm_duty: number | null;
    required_motor_current_per_channel_a: number;
    required_output_torque_per_wheel_nm: number;
    traction_margin_per_wheel_n: number;
    checks: Check[];
    point: {
      battery: {
        terminal_voltage_v: number;
        open_circuit_voltage_v: number;
        current_a: number;
        power_w: number;
        charge_limited_runtime_s: number | null;
      };
    } | null;
  };
  limitations: string[];
};
const fmt = (value: number | null | undefined, unit = "") =>
  value == null
    ? "Unavailable"
    : `${value !== 0 && (Math.abs(value) < 0.0001 || Math.abs(value) >= 1e7) ? value.toExponential(4) : value.toLocaleString("en", { maximumSignificantDigits: 5 })}${unit}`;
const pretty = (value: unknown) => JSON.stringify(value, null, 2);
function save(value: unknown, name: string) {
  const url = URL.createObjectURL(
    new Blob([pretty(value)], { type: "application/json" }),
  );
  const link = document.createElement("a");
  link.href = url;
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function EngineeringPanel({
  project,
  visible,
  disabled,
}: {
  project: unknown;
  visible: boolean;
  disabled: boolean;
}) {
  const [config, setConfig] = useState<Config | null>(null);
  const [result, setResult] = useState<Report | null>(null);
  const [reference, setReference] = useState<Report | null>(null);
  const [resultKey, setResultKey] = useState("");
  const [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  useEffect(() => {
    if (visible && !config)
      void request<Config>("/api/hardware/sizing/defaults")
        .then(setConfig)
        .catch((error) => setError(String(error)));
  }, [visible, config]);
  const key = pretty({ project, config }),
    stale = !!result && key !== resultKey;
  const comparable =
    !!reference &&
    !!result &&
    pretty(reference.config) === pretty(result.config);
  const calculate = async () => {
    setBusy(true);
    setError("");
    const input = { project, config };
    try {
      const report = await post<Report>("/api/hardware/sizing", input);
      setResult(report);
      setResultKey(pretty(input));
    } catch (error) {
      setError(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  };
  const values = (report: Report): [string, number | null, string][] => [
    [
      "Required PWM",
      report.result.required_pwm_duty == null
        ? null
        : report.result.required_pwm_duty * 100,
      "%",
    ],
    [
      "Motor current · each",
      report.result.required_motor_current_per_channel_a,
      " A",
    ],
    [
      "Wheel torque · each",
      report.result.required_output_torque_per_wheel_nm,
      " N·m",
    ],
    ["Traction margin · each", report.result.traction_margin_per_wheel_n, " N"],
    ["Pack current", report.result.point?.battery.current_a ?? null, " A"],
    [
      "Pack voltage",
      report.result.point?.battery.terminal_voltage_v ?? null,
      " V",
    ],
  ];
  const fields = [
    ["speed_m_s", "Required speed · m/s"],
    ["acceleration_m_s2", "Acceleration · m/s²"],
    ["grade_rad", "Uphill grade · degrees"],
    ["soc", "Battery charge · fraction 0–1"],
    ["max_pwm_duty", "Maximum duty · fraction 0–1"],
    ["auxiliary_current_a", "Auxiliary pack load · A"],
    ["friction_coefficient", "Assumed friction coefficient"],
    ["driven_weight_fraction", "Weight on driven wheels · fraction"],
    ["rotating_inertia_per_wheel_kg_m2", "Rotating inertia per wheel · kg·m²"],
    ["vcc_v", "Driver logic supply · V"],
  ];
  return (
    <section className="engineering-panel hardware-content" hidden={!visible}>
      <h2>Motor and battery sizing</h2>
      <p className="hint">
        Check one speed, acceleration and slope against the current design. This
        estimates an operating point; startup, mission runtime and physical
        readiness require separate evidence.
      </p>
      {error && (
        <p className="message error" role="alert">
          {error}
        </p>
      )}
      {disabled && (
        <p className="hint">
          Finish the current setup action or apply/discard JSON edits before
          calculating.
        </p>
      )}
      {config && (
        <>
          <details open>
            <summary>
              Operating conditions · {fmt(Number(config.speed_m_s), " m/s")} ·{" "}
              {fmt((Number(config.grade_rad) * 180) / Math.PI, "° uphill")} ·{" "}
              {fmt(Number(config.soc) * 100, "% charge")}
            </summary>
            <fieldset disabled={busy || disabled}>
              <legend>Operating conditions and assumptions</legend>
              <div className="hardware-parameters">
                {fields.map(([field, label]) => (
                  <label key={field}>
                    {label}
                    <input
                      type="number"
                      step="any"
                      value={
                        field === "grade_rad"
                          ? (Number(config[field]) * 180) / Math.PI
                          : Number(config[field])
                      }
                      onChange={(event) => {
                        const number = event.target.valueAsNumber;
                        if (Number.isFinite(number))
                          setConfig({
                            ...config,
                            [field]:
                              field === "grade_rad"
                                ? (number * Math.PI) / 180
                                : number,
                          });
                      }}
                    />
                  </label>
                ))}
              </div>
            </fieldset>
          </details>
          <button
            className="primary"
            disabled={busy || disabled}
            onClick={() => void calculate()}
          >
            {busy ? "Calculating…" : "Calculate demand"}
          </button>
        </>
      )}
      {stale && (
        <p className="message notice">
          The design or conditions changed. Calculate again to update this
          estimate.
        </p>
      )}
      {result && (
        <>
          <h2>
            {result.status.replaceAll("_", " ")} ·{" "}
            {
              result.result.checks.filter(
                (check) => check.status === "not_assessed",
              ).length
            }{" "}
            checks unassessed
          </h2>
          <div className="engineering-cards">
            {values(result).map(([label, value, unit]) => (
              <div className="engineering-card" key={label}>
                <small>{label}</small>
                <strong>{fmt(value, unit)}</strong>
              </div>
            ))}
          </div>
          <div className="hardware-actions">
            <button
              disabled={stale}
              onClick={() => save(result, "ixion-sizing.json")}
            >
              Export estimate
            </button>
            <button disabled={stale} onClick={() => setReference(result)}>
              Pin comparison baseline
            </button>
            {reference && (
              <button onClick={() => setReference(null)}>Clear baseline</button>
            )}
          </div>
          {reference && (
            <div className="engineering-comparison">
              <h3>Design comparison at this operating point</h3>
              {!comparable ? (
                <p className="hint">
                  The baseline and candidate use different conditions. Use
                  identical conditions to compare the two designs.
                </p>
              ) : (
                <>
                  <p className="hint">
                    Baseline: {reference.project.name} (
                    {fmt(
                      reference.project.snapshot.assembly.total_mass_kg,
                      " kg",
                    )}
                    ) · candidate: {result.project.name} (
                    {fmt(result.project.snapshot.assembly.total_mass_kg, " kg")}
                    ). Deltas are candidate minus baseline.
                  </p>
                  <table>
                    <thead>
                      <tr>
                        <th>Metric</th>
                        <th>Baseline</th>
                        <th>Candidate</th>
                        <th>Delta</th>
                      </tr>
                    </thead>
                    <tbody>
                      {values(result).map(([label, value, unit], index) => {
                        const before = values(reference)[index][1];
                        return (
                          <tr key={label}>
                            <td>{label}</td>
                            <td>{fmt(before, unit)}</td>
                            <td>{fmt(value, unit)}</td>
                            <td>
                              {fmt(
                                value == null || before == null
                                  ? null
                                  : value - before,
                                label === "Required PWM" ? " pp" : unit,
                              )}
                            </td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                  <button
                    disabled={stale}
                    onClick={() =>
                      save(
                        { baseline: reference, candidate: result },
                        "ixion-sizing-comparison.json",
                      )
                    }
                  >
                    Export estimate pair
                  </button>
                </>
              )}
            </div>
          )}
          <h3>Declared checks</h3>
          <div className="engineering-table">
            <table>
              <thead>
                <tr>
                  <th>Check</th>
                  <th>Status</th>
                  <th>Value</th>
                  <th>Limits</th>
                  <th>Meaning</th>
                </tr>
              </thead>
              <tbody>
                {result.result.checks.map((check) => (
                  <tr key={check.code}>
                    <td>{check.code.replaceAll("_", " ")}</td>
                    <td>{check.status.replaceAll("_", " ")}</td>
                    <td>
                      {fmt(check.actual, check.unit ? ` ${check.unit}` : "")}
                    </td>
                    <td>
                      {check.minimum == null && check.maximum == null
                        ? "Not declared"
                        : `${check.minimum == null ? "" : check.minimum === Number.MIN_VALUE ? "> 0 " : `≥ ${fmt(check.minimum)} `}${check.maximum == null ? "" : `≤ ${fmt(check.maximum)}`}`}
                    </td>
                    <td>{check.explanation}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
          {result.result.point && (
            <p className="hint">
              Supply estimate:{" "}
              {fmt(
                result.result.point.battery.open_circuit_voltage_v,
                " V open circuit",
              )}{" "}
              →{" "}
              {fmt(
                result.result.point.battery.terminal_voltage_v,
                " V at the declared load",
              )}
              . {fmt(result.result.point.battery.power_w, " W pack output")}.
              Constant-current charge estimate:{" "}
              {fmt(
                result.result.point.battery.charge_limited_runtime_s == null
                  ? null
                  : result.result.point.battery.charge_limited_runtime_s / 60,
                " min",
              )}
              ; this holds the instantaneous current constant and does not
              predict mission duration.
            </p>
          )}
          <details>
            <summary>Model limits and retained assumptions</summary>
            <ul>
              {result.limitations.map((line) => (
                <li key={line}>{line}</li>
              ))}
            </ul>
            <pre>{pretty(result.config)}</pre>
          </details>
        </>
      )}
    </section>
  );
}
