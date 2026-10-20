import { useEffect, useRef, useState } from "react";
import { request } from "./api";
import { quantity } from "./ReplayPanel";

export type HardwareJob = {
  id: string;
  name: string;
  project_name: string;
  created_at: string;
  status: string;
  outcome: string | null;
  error: string | null;
  summary: {
    total: number;
    passed: number;
    failed: number;
    error: number;
  } | null;
};
type Metric = {
  unit: string;
  baseline: number | null;
  candidate: number | null;
  delta: number | null;
  availability: string;
};
type Comparison = {
  status: string;
  conditions_sha256: string;
  sources: Record<
    "baseline" | "candidate",
    { sha256: string; project_sha256: string; hash_scope: string }
  >;
  run_metadata: Record<
    "baseline" | "candidate",
    HardwareJob & { mass_kg: number }
  >;
  cases: {
    name: string;
    baseline_status: string;
    candidate_status: string;
    baseline_outcome: string | null;
    candidate_outcome: string | null;
    metrics: Record<string, Metric>;
  }[];
  design_changes: {
    total: number;
    shown: number;
    truncated: boolean;
    changes: {
      path: string;
      baseline: unknown;
      candidate: unknown;
      baseline_present: boolean;
      candidate_present: boolean;
    }[];
  };
  limitations: string[];
};
const readable = (value: string) => value.replaceAll("_", " ");
const metricName = (value: string) =>
  readable(value.replace(/_(m_s|s|m|a|j)$/, ""));
const valueText = (value: unknown, present = true) =>
  present ? JSON.stringify(value) : "Absent";

export function HardwareComparison({
  jobs,
  visible,
  onOpenRun,
}: {
  jobs: HardwareJob[];
  visible: boolean;
  onOpenRun: (id: string) => void;
}) {
  const [baseline, setBaseline] = useState(""),
    [candidate, setCandidate] = useState("");
  const [result, setResult] = useState<Comparison | null>(null);
  const [caseIndex, setCaseIndex] = useState(0),
    [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  const pending = useRef<AbortController | null>(null);
  useEffect(() => {
    pending.current?.abort();
    setResult(null);
    setError("");
    setBusy(false);
    setCaseIndex(0);
  }, [baseline, candidate]);
  useEffect(() => () => pending.current?.abort(), []);
  const completed = jobs.filter((job) => job.status === "completed");
  const query = `baseline=${encodeURIComponent(baseline)}&candidate=${encodeURIComponent(candidate)}`;
  const compare = async () => {
    pending.current?.abort();
    const controller = new AbortController();
    pending.current = controller;
    setBusy(true);
    setError("");
    setResult(null);
    setCaseIndex(0);
    try {
      const data = await request<Comparison>(
        `/api/hardware/comparisons?${query}`,
        { signal: controller.signal },
      );
      if (!controller.signal.aborted) setResult(data);
    } catch (error) {
      if (!controller.signal.aborted)
        setError(error instanceof Error ? error.message : String(error));
    } finally {
      if (!controller.signal.aborted) setBusy(false);
    }
  };
  const row = result?.cases[caseIndex];
  return (
    <section className="hardware-comparison hardware-content" hidden={!visible}>
      <h2>Compare saved robot designs</h2>
      <p className="hint">
        Select two completed hardware suites. Cases, controller settings and
        acceptance policies must match. This reads saved evidence without
        running another simulation.
      </p>
      <div className="hardware-summary">
        {(
          [
            ["baseline", baseline, setBaseline],
            ["candidate", candidate, setCandidate],
          ] as const
        ).map(([label, value, setter]) => (
          <label key={label}>
            {label === "baseline"
              ? "Baseline hardware run"
              : "Candidate hardware run"}
            <select value={value} onChange={(e) => setter(e.target.value)}>
              <option value="">Choose saved run…</option>
              {completed.map((job) => (
                <option value={job.id} key={job.id}>
                  {job.project_name} / {job.name} · {job.created_at} ·{" "}
                  {job.id.slice(-8)} · {job.outcome}
                </option>
              ))}
            </select>
          </label>
        ))}
      </div>
      {completed.length < 2 && (
        <p className="hint">
          Run a baseline, change the design and run the same suite again.
          Completed suites with failed checks or model errors are included.
        </p>
      )}
      {baseline && baseline === candidate && (
        <p className="message notice">
          Both selections are the same run. Available deltas will be zero.
        </p>
      )}
      <div className="hardware-actions">
        <button
          disabled={!baseline || !candidate || busy}
          onClick={() => void compare()}
        >
          {busy ? "Comparing saved evidence…" : "Compare saved designs"}
        </button>
        {result && (
          <>
            <a href={`/api/hardware/comparisons?${query}&format=json`}>
              Comparison JSON ↓
            </a>
            <a href={`/api/hardware/comparisons?${query}&format=html`}>
              Offline comparison HTML ↓
            </a>
          </>
        )}
      </div>
      {error && (
        <p className="message error" role="alert">
          {error}
        </p>
      )}
      {result && (
        <>
          <p
            className={`message ${result.status === "passed" ? "notice" : "error"}`}
            role="status"
          >
            Candidate suite: {result.status}. This is acceptance against the
            common policy, not an overall improvement score.
          </p>
          <div className="comparison-sources">
            {(["baseline", "candidate"] as const).map((label) => (
              <article key={label}>
                <h3>
                  {label === "baseline" ? "Baseline" : "Candidate"} ·{" "}
                  {result.run_metadata[label].mass_kg} kg
                </h3>
                <strong>{result.run_metadata[label].project_name}</strong>
                <p>
                  {result.run_metadata[label].name} ·{" "}
                  {result.run_metadata[label].outcome}
                </p>
                <p className="hint">{result.run_metadata[label].created_at}</p>
                <button
                  onClick={() =>
                    onOpenRun(label === "baseline" ? baseline : candidate)
                  }
                >
                  Open {label} replay
                </button>
                <details>
                  <summary>Saved evidence identity</summary>
                  <p>
                    Report SHA-256 ({result.sources[label].hash_scope}):{" "}
                    {result.sources[label].sha256}
                  </p>
                  <p>Design SHA-256: {result.sources[label].project_sha256}</p>
                </details>
              </article>
            ))}
          </div>
          <h3>Case outcomes</h3>
          <div className="comparison-table">
            <table>
              <thead>
                <tr>
                  <th>Case</th>
                  <th>Baseline</th>
                  <th>Candidate</th>
                </tr>
              </thead>
              <tbody>
                {result.cases.map((test) => (
                  <tr key={test.name}>
                    <th>{test.name}</th>
                    <td>
                      {test.baseline_status} ·{" "}
                      {test.baseline_outcome ?? "No motion evidence"}
                    </td>
                    <td>
                      {test.candidate_status} ·{" "}
                      {test.candidate_outcome ?? "No motion evidence"}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
          <label className="comparison-case">
            Compare case
            <select
              value={caseIndex}
              onChange={(e) => setCaseIndex(Number(e.target.value))}
            >
              {result.cases.map((test, i) => (
                <option value={i} key={test.name}>
                  {test.name}
                </option>
              ))}
            </select>
          </label>
          <p className="hint">
            Delta = candidate − baseline, in each row's units. Energy totals
            cover each run's actual duration; compare outcomes and recorded
            duration before interpreting them. Unavailable values are never
            treated as zero.
          </p>
          <div className="comparison-table">
            <table aria-label="Saved design metric differences">
              <thead>
                <tr>
                  <th>Metric</th>
                  <th>Baseline</th>
                  <th>Candidate</th>
                  <th>Delta</th>
                  <th>Evidence</th>
                </tr>
              </thead>
              <tbody>
                {row &&
                  Object.entries(row.metrics).map(([name, metric]) => (
                    <tr key={name}>
                      <th>{metricName(name)}</th>
                      <td>
                        {quantity(
                          metric.baseline,
                          metric.unit === "1" ? "fraction" : metric.unit,
                        )}
                      </td>
                      <td>
                        {quantity(
                          metric.candidate,
                          metric.unit === "1" ? "fraction" : metric.unit,
                        )}
                      </td>
                      <td>
                        {quantity(
                          metric.delta,
                          metric.unit === "1" ? "fraction" : metric.unit,
                        )}
                      </td>
                      <td>{readable(metric.availability)}</td>
                    </tr>
                  ))}
              </tbody>
            </table>
          </div>
          <details className="comparison-changes" open>
            <summary>
              Declared design changes · {result.design_changes.total}
            </summary>
            {result.design_changes.total === 0 && (
              <p>No declared design changes.</p>
            )}
            {result.design_changes.truncated && (
              <p className="message notice">
                Showing the first {result.design_changes.shown} changes of{" "}
                {result.design_changes.total}.
              </p>
            )}
            <div className="comparison-table">
              <table>
                <thead>
                  <tr>
                    <th>Design field</th>
                    <th>Baseline</th>
                    <th>Candidate</th>
                  </tr>
                </thead>
                <tbody>
                  {result.design_changes.changes.map((change) => (
                    <tr key={change.path}>
                      <th>{change.path}</th>
                      <td>
                        {valueText(change.baseline, change.baseline_present)}
                      </td>
                      <td>
                        {valueText(change.candidate, change.candidate_present)}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </details>
          <details>
            <summary>Conditions identity and interpretation</summary>
            <p>
              Normalized case conditions SHA-256: {result.conditions_sha256}
            </p>
            <ul>
              {result.limitations.map((note) => (
                <li key={note}>{note}</li>
              ))}
            </ul>
            <p>
              Exports embed both complete source reports. The JSON can be read
              by <code>ixion rover-view</code>; the HTML is self-contained.
            </p>
          </details>
        </>
      )}
    </section>
  );
}
