import { useEffect, useRef, useState } from "react";
import { active, ready, request } from "./api";
import type { Comparison, Job } from "./types";
import { RegressionPanel } from "./RegressionPanel";

const format = (value: number | null) =>
  value === null
    ? "—"
    : new Intl.NumberFormat("en", { maximumSignificantDigits: 6 }).format(
        value,
      );

function ConfigValue({ present, value }: { present: boolean; value: unknown }) {
  if (!present) return <span className="muted">Absent</span>;
  const text = JSON.stringify(value);
  return text.length < 100 ? (
    <code>{text}</code>
  ) : (
    <details>
      <summary>
        View value ({Array.isArray(value) ? `${value.length} items` : "object"})
      </summary>
      <pre>{JSON.stringify(value, null, 2)}</pre>
    </details>
  );
}

export function ComparisonPanel({
  visible,
  onReplay,
}: {
  visible: boolean;
  onReplay: (id: string) => void;
}) {
  const [jobs, setJobs] = useState<Job[]>([]);
  const [total, setTotal] = useState(0);
  const [offset, setOffset] = useState(0);
  const [refresh, setRefresh] = useState(0);
  const [picked, setPicked] = useState<Job[]>([]);
  const [result, setResult] = useState<Comparison | null>(null);
  const [error, setError] = useState("");
  const [listError, setListError] = useState("");
  const [loading, setLoading] = useState(false);
  const [busy, setBusy] = useState(false);
  const pending = useRef<AbortController | null>(null);
  useEffect(() => () => pending.current?.abort(), []);
  useEffect(() => {
    if (!visible) return;
    const controller = new AbortController();
    setLoading(true);
    setListError("");
    request<{ items: Job[]; total: number }>(
      `/api/runs?offset=${offset}&limit=20`,
      { signal: controller.signal },
    )
      .then((data) => {
        if (!controller.signal.aborted) {
          setJobs(data.items);
          setTotal(data.total);
        }
      })
      .catch((e) => {
        if (!controller.signal.aborted)
          setListError(e instanceof Error ? e.message : String(e));
      })
      .finally(() => {
        if (!controller.signal.aborted) setLoading(false);
      });
    return () => controller.abort();
  }, [visible, offset, refresh]);

  const changeSelection = (next: Job[]) => {
    pending.current?.abort();
    setBusy(false);
    setPicked(next);
    setResult(null);
    setError("");
  };
  const compare = async () => {
    pending.current?.abort();
    const controller = new AbortController();
    pending.current = controller;
    setBusy(true);
    setError("");
    setResult(null);
    try {
      const data = await request<Comparison>("/api/comparisons", {
        method: "POST",
        signal: controller.signal,
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ run_ids: picked.map((job) => job.id) }),
      });
      if (!controller.signal.aborted) setResult(data);
    } catch (e) {
      if (!controller.signal.aborted)
        setError(e instanceof Error ? e.message : String(e));
    } finally {
      if (!controller.signal.aborted) setBusy(false);
    }
  };
  const download = () => {
    const url = URL.createObjectURL(
      new Blob([JSON.stringify(result, null, 2)], { type: "application/json" }),
    );
    const link = document.createElement("a");
    link.href = url;
    link.download = "roboforge-comparison.json";
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  return (
    <section
      className="comparison-workspace"
      hidden={!visible}
      aria-label="Run comparison"
    >
      <aside className="panel comparison-picker">
        <div className="panel-title">
          <h2>Saved runs</h2>
          <span className="tag">{picked.length} / 4 selected</span>
        </div>
        <p className="hint">
          Select two to four finished runs. Failed and cancelled runs stay
          available for comparison.
        </p>
        <div className="comparison-paging">
          <button onClick={() => setRefresh((v) => v + 1)} disabled={loading}>
            Refresh runs
          </button>
          <span className="small">
            {total
              ? `${offset + 1}–${Math.min(offset + 20, total)} of ${total}`
              : "No runs yet"}
          </span>
        </div>
        {listError && (
          <p role="alert" className="hint amber">
            {listError}
          </p>
        )}
        <fieldset disabled={loading} className="comparison-choices">
          <legend className="small">
            {loading ? "Loading runs…" : "Available runs"}
          </legend>
          {jobs.map((job) => (
            <label key={job.id} className="comparison-choice">
              <input
                type="checkbox"
                aria-label={`Compare ${job.name} ${job.id.slice(-6)}`}
                checked={picked.some((item) => item.id === job.id)}
                disabled={
                  active(job.status) ||
                  (picked.length === 4 &&
                    !picked.some((item) => item.id === job.id))
                }
                onChange={(event) =>
                  changeSelection(
                    event.target.checked
                      ? [...picked, job]
                      : picked.filter((item) => item.id !== job.id),
                  )
                }
              />
              <span>
                <strong>{job.name}</strong>
                <small>
                  seed {job.seed} · {job.status.replaceAll("_", " ")} ·{" "}
                  {job.id.slice(-6)}
                </small>
              </span>
            </label>
          ))}
        </fieldset>
        <div className="comparison-paging">
          <button
            disabled={loading || offset === 0}
            onClick={() => setOffset(Math.max(0, offset - 20))}
          >
            Newer
          </button>
          <button
            disabled={loading || offset + 20 >= total}
            onClick={() => setOffset(offset + 20)}
          >
            Older
          </button>
        </div>
      </aside>
      <div className="panel comparison-editor">
        <div className="panel-title editor-title">
          <h2>comparison.json</h2>
          <span className="tag">SAVED RESULTS</span>
        </div>
        <div className="comparison-body">
          <h2>Compare runs</h2>
          <p className="hint">
            Choose a baseline to inspect measurements and configuration changes.
            Differences alone do not establish which setting caused an outcome.
          </p>
          {picked.length > 0 && (
            <div className="comparison-selection">
              <label>
                Baseline
                <select
                  value={picked[0].id}
                  onChange={(e) => {
                    const first = picked.find(
                      (job) => job.id === e.target.value,
                    )!;
                    changeSelection([
                      first,
                      ...picked.filter((job) => job.id !== first.id),
                    ]);
                  }}
                >
                  {picked.map((job) => (
                    <option key={job.id} value={job.id}>
                      {job.name} · {job.id.slice(-6)}
                    </option>
                  ))}
                </select>
              </label>
              <div className="comparison-selected" aria-label="Selected runs">
                {picked.map((job) => (
                  <button
                    key={job.id}
                    aria-label={`Remove ${job.name} ${job.id.slice(-6)}`}
                    onClick={() =>
                      changeSelection(
                        picked.filter((item) => item.id !== job.id),
                      )
                    }
                  >
                    {job.name} · {job.id.slice(-6)} ×
                  </button>
                ))}
              </div>
            </div>
          )}
          <div className="comparison-actions">
            <button
              className="primary"
              disabled={picked.length < 2 || busy}
              onClick={compare}
            >
              {busy ? "Comparing…" : "Compare selected runs"}
            </button>
            {result && <button onClick={download}>Export comparison ↓</button>}
          </div>
          {error && (
            <p role="alert" className="hint amber">
              {error}
            </p>
          )}
          {result && (
            <>
              <RegressionPanel
                key={result.runs.map((run) => run.id).join(":")}
                baseline={result.baseline_id}
                candidates={result.runs.slice(1).map((run) => run.id)}
              />
              <p role="status" className="hint">
                {result.same_setup
                  ? "Same configuration apart from names."
                  : "Configurations differ; inspect the changed fields below."}{" "}
                {!result.same_software &&
                  "These runs used different software versions."}
                {result.same_source === false &&
                  " Python source fingerprints differ."}
                {result.same_dependencies === false &&
                  " Core dependency versions differ."}
                {(result.same_source == null ||
                  result.same_dependencies == null) &&
                  " Original source or dependency provenance is unavailable for some runs."}
              </p>
              <div className="comparison-table-scroll">
                <table className="comparison-table">
                  <caption>
                    Recorded metrics · Δ = run minus baseline, in the metric's
                    units. — means unavailable. No ranking is implied.
                  </caption>
                  <thead>
                    <tr>
                      <th scope="col">Measurement</th>
                      {result.runs.map((run, index) => (
                        <th scope="col" key={run.id}>
                          <strong>{run.name}</strong>
                          <small>
                            {index === 0 ? "BASELINE · " : ""}
                            {run.id.slice(-6)}
                          </small>
                          <small>
                            {run.status.replaceAll("_", " ")} · seed {run.seed}
                          </small>
                          {run.navigation_outcome && (
                            <small>
                              Navigation:{" "}
                              {run.navigation_outcome.replaceAll("_", " ")}
                            </small>
                          )}
                          <small>v{run.software_version}</small>
                          {run.provenance ? (
                            <details>
                              <summary>Execution provenance</summary>
                              <small>
                                Captured: {run.provenance.captured_utc}
                              </small>
                              <small>
                                Python {run.provenance.python.version} ·{" "}
                                {run.provenance.python.implementation}
                              </small>
                              <small>
                                Source:{" "}
                                {run.provenance.source.sha256 ?? "unavailable"}
                              </small>
                              <small>
                                Git:{" "}
                                {run.provenance.source.git_revision ??
                                  "unavailable"}
                              </small>
                              <small>
                                Working tree:{" "}
                                {run.provenance.source.git_dirty == null
                                  ? "unknown"
                                  : run.provenance.source.git_dirty
                                    ? "modified"
                                    : "clean"}
                              </small>
                              {Object.entries(run.provenance.dependencies).map(
                                ([name, version]) => (
                                  <small key={name}>
                                    {name}: {version}
                                  </small>
                                ),
                              )}
                              <small>
                                Source hash covers files on disk at capture
                                time, not loaded bytecode.
                              </small>
                            </details>
                          ) : (
                            <small>Execution provenance unavailable</small>
                          )}
                          {run.error && (
                            <p className="comparison-error">{run.error}</p>
                          )}
                          {ready(run.status) && (
                            <button onClick={() => onReplay(run.id)}>
                              Open replay
                            </button>
                          )}
                        </th>
                      ))}
                    </tr>
                  </thead>
                  <tbody>
                    {result.metrics.map((metric) => (
                      <tr key={metric.name}>
                        <th scope="row">{metric.name.replaceAll("_", " ")}</th>
                        {metric.values.map((value, i) => (
                          <td key={result.runs[i].id}>
                            <span>{format(value)}</span>
                            {i > 0 && (
                              <small>
                                Δ{" "}
                                {metric.deltas[i] !== null &&
                                metric.deltas[i]! > 0
                                  ? "+"
                                  : ""}
                                {format(metric.deltas[i])}
                              </small>
                            )}
                          </td>
                        ))}
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
              {result.metrics.length === 0 && (
                <p className="hint">
                  No measurements were saved for these runs.
                </p>
              )}
              <h2>
                Configuration differences ({result.config_differences.length})
              </h2>
              {result.config_differences.length ? (
                <div className="comparison-table-scroll">
                  <table className="comparison-table config-differences">
                    <caption>
                      Exact saved input values. Arrays expand in place; the
                      export contains complete configurations and run
                      identities.
                    </caption>
                    <thead>
                      <tr>
                        <th scope="col">Field</th>
                        {result.runs.map((run) => (
                          <th scope="col" key={run.id}>
                            {run.name}
                            <small>{run.id.slice(-6)}</small>
                          </th>
                        ))}
                      </tr>
                    </thead>
                    <tbody>
                      {result.config_differences.map((row) => (
                        <tr key={row.path}>
                          <th scope="row">
                            <code>{row.path}</code>
                          </th>
                          {row.values.map((value, i) => (
                            <td key={result.runs[i].id}>
                              <ConfigValue {...value} />
                            </td>
                          ))}
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              ) : (
                <p className="hint">The saved configurations match exactly.</p>
              )}
              <p className="hint">
                Snapshot from {new Date(result.created_utc).toLocaleString()}.
                Export to keep it; comparison snapshots are not stored
                separately on the server.
              </p>
            </>
          )}
        </div>
      </div>
    </section>
  );
}
