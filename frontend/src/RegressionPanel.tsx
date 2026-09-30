import { useEffect, useRef, useState } from "react";
import { request } from "./api";

type CheckReport = {
  status: string;
  checks: {
    name: string;
    status: string;
    reason: string;
    measured?: number | null;
  }[];
};
const initial = JSON.stringify(
  {
    name: "Mobile robot acceptance",
    allowed_config_changes: [],
    allow_software_change: false,
    require_goal_reached: false,
    rules: [{ name: "No collisions", metric: "collision_count", maximum: 0 }],
  },
  null,
  2,
);

function save(name: string, text: string) {
  const url = URL.createObjectURL(
    new Blob([text], { type: "application/json" }),
  );
  const link = document.createElement("a");
  link.href = url;
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function RegressionPanel({
  baseline,
  candidates,
}: {
  baseline: string;
  candidates: string[];
}) {
  const [candidate, setCandidate] = useState(candidates[0]);
  const [policy, setPolicy] = useState(initial);
  const [report, setReport] = useState<CheckReport | null>(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const pending = useRef<AbortController | null>(null);
  useEffect(() => () => pending.current?.abort(), []);
  function reset() {
    pending.current?.abort();
    setReport(null);
    setError("");
    setBusy(false);
  }
  async function check() {
    reset();
    const controller = new AbortController();
    pending.current = controller;
    setBusy(true);
    try {
      const data = await request<CheckReport>("/api/regressions", {
        method: "POST",
        signal: controller.signal,
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          baseline_id: baseline,
          candidate_id: candidate,
          policy: JSON.parse(policy),
        }),
      });
      if (!controller.signal.aborted) setReport(data);
    } catch (e) {
      if (!controller.signal.aborted) setError(String(e));
    } finally {
      if (!controller.signal.aborted) setBusy(false);
    }
  }
  return (
    <section aria-label="Regression checks">
      <h2>Acceptance checks</h2>
      <p className="hint">
        The selected baseline stays fixed. Missing evidence is inconclusive.
        Delta rules use candidate minus baseline in the metric’s units. Setup
        differences must be named in allowed_config_changes using the paths in
        the comparison below.
      </p>
      <label>
        Candidate run
        <select
          value={candidate}
          onChange={(e) => {
            reset();
            setCandidate(e.target.value);
          }}
        >
          {candidates.map((id) => (
            <option key={id} value={id}>
              {id}
            </option>
          ))}
        </select>
      </label>
      <button
        onClick={() => {
          reset();
          setPolicy(
            JSON.stringify(
              {
                name: "Navigation goal acceptance",
                require_goal_reached: true,
                max_goal_error_m: 0.1,
                allowed_config_changes: [],
                allow_software_change: false,
                rules: [
                  {
                    name: "No collisions",
                    metric: "collision_count",
                    maximum: 0,
                  },
                ],
              },
              null,
              2,
            ),
          );
        }}
      >
        Use navigation acceptance policy
      </button>
      <label>
        Acceptance policy JSON
        <textarea
          aria-label="Acceptance policy JSON"
          rows={14}
          style={{ width: "100%", boxSizing: "border-box" }}
          value={policy}
          onChange={(e) => {
            reset();
            setPolicy(e.target.value);
          }}
        />
      </label>
      <div className="comparison-actions">
        <button disabled={busy} onClick={check}>
          {busy ? "Checking…" : "Run acceptance checks"}
        </button>
        <button
          onClick={() => {
            try {
              save(
                "acceptance-policy.json",
                JSON.stringify(JSON.parse(policy), null, 2),
              );
            } catch (e) {
              setError(String(e));
            }
          }}
        >
          Save policy ↓
        </button>
        {report && (
          <button
            onClick={() =>
              save("regression-report.json", JSON.stringify(report, null, 2))
            }
          >
            Check report ↓
          </button>
        )}
      </div>
      <p className="hint">
        Paste a saved policy here to reuse it. Bounds are inclusive; use
        minimum, maximum or both. mode can be absolute (default) or delta.
        Execution completion is always checked.
      </p>
      {error && <p role="alert">{error}</p>}
      {report && (
        <>
          <p role="status">Acceptance result: {report.status.toUpperCase()}</p>
          <table className="comparison-table">
            <thead>
              <tr>
                <th>Check</th>
                <th>Result</th>
                <th>Measured</th>
                <th>Reason</th>
              </tr>
            </thead>
            <tbody>
              {report.checks.map((item, i) => (
                <tr key={i}>
                  <td>{item.name}</td>
                  <td>{item.status}</td>
                  <td>{item.measured ?? "—"}</td>
                  <td>{item.reason}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </>
      )}
    </section>
  );
}
