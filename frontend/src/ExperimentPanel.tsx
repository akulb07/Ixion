import { useEffect, useRef, useState } from "react";
import { active, post, ready, request } from "./api";
import type { Batch, BatchPreview, BatchSummary, Config } from "./types";
import { SweepAnalysis } from "./SweepAnalysis";

type Design = {
  name: string;
  base: Config;
  seeds: number[];
  axes: { path: string; values: number[] }[];
};
const errorText = (error: unknown) =>
  error instanceof Error ? error.message : String(error);
const number = (value: number | null) =>
  value === null
    ? "—"
    : value.toLocaleString("en", { maximumSignificantDigits: 5 });
function numbers(text: string, label: string) {
  const values = text.split(",");
  if (values.some((value) => !value.trim() || !Number.isFinite(Number(value))))
    throw new Error(`${label}: enter finite numbers separated by commas.`);
  return values.map(Number);
}

export function ExperimentPanel({
  visible,
  config,
  disabled,
  onReplay,
}: {
  visible: boolean;
  config: Config | null;
  disabled: boolean;
  onReplay: (id: string) => void;
}) {
  const [name, setName] = useState("parameter_sweep");
  const [seeds, setSeeds] = useState("42, 43");
  const [axes, setAxes] = useState([
    { path: "robot.wheel_radius", values: "0.04, 0.05, 0.06" },
  ]);
  const [preview, setPreview] = useState<{
    design: Design;
    data: BatchPreview;
  } | null>(null);
  const [checking, setChecking] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [listError, setListError] = useState("");
  const [jobs, setJobs] = useState<BatchSummary[]>([]);
  const [offset, setOffset] = useState(0);
  const [total, setTotal] = useState(0);
  const [refresh, setRefresh] = useState(0);
  const [selected, setSelected] = useState<string | null>(null);
  const [batch, setBatch] = useState<Batch | null>(null);
  const [metric, setMetric] = useState("path_length_m");
  const pending = useRef<AbortController | null>(null);
  const invalidate = () => {
    pending.current?.abort();
    setChecking(false);
    setPreview(null);
    setError("");
  };
  useEffect(() => {
    pending.current?.abort();
    setChecking(false);
    setPreview(null);
    return () => pending.current?.abort();
  }, [config, disabled]);

  useEffect(() => {
    if (!visible) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      try {
        const data = await request<{ items: BatchSummary[]; total: number }>(
          `/api/experiments?offset=${offset}&limit=20`,
          { signal: controller.signal },
        );
        if (!controller.signal.aborted) {
          setJobs(data.items);
          setTotal(data.total);
          setListError("");
        }
      } catch (e) {
        if (!controller.signal.aborted) setListError(errorText(e));
      }
      if (!controller.signal.aborted) timer = setTimeout(poll, 1500);
    };
    void poll();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, [visible, offset, refresh]);

  useEffect(() => {
    if (!selected || !visible) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    const poll = async () => {
      try {
        const data = await request<Batch>(`/api/experiments/${selected}`, {
          signal: controller.signal,
        });
        if (!controller.signal.aborted) {
          setBatch(data);
          if (active(data.status)) timer = setTimeout(poll, 500);
        }
      } catch (e) {
        if (!controller.signal.aborted) setError(errorText(e));
      }
    };
    void poll();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, [selected, visible, refresh]);

  const check = async () => {
    if (!config) return;
    invalidate();
    setChecking(true);
    const controller = new AbortController();
    pending.current = controller;
    try {
      const design = {
        name,
        base: config,
        seeds: numbers(seeds, "Seeds"),
        axes: axes.map((axis) => ({
          path: axis.path.trim(),
          values: numbers(axis.values, "Values"),
        })),
      };
      const data = await request<BatchPreview>("/api/experiments/preview", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        signal: controller.signal,
        body: JSON.stringify(design),
      });
      if (!controller.signal.aborted) setPreview({ design, data });
    } catch (e) {
      if (!controller.signal.aborted) setError(errorText(e));
    } finally {
      if (!controller.signal.aborted) setChecking(false);
    }
  };
  const start = async () => {
    if (!preview) return;
    setBusy(true);
    setError("");
    try {
      const result = await post<Batch>("/api/experiments", preview.design);
      setSelected(result.id);
      setBatch(result);
      setRefresh((v) => v + 1);
    } catch (e) {
      setError(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  const cancel = async () => {
    if (!batch) return;
    setBusy(true);
    setError("");
    try {
      await post<Batch>(`/api/experiments/${batch.id}/cancel`);
      setRefresh((v) => v + 1);
    } catch (e) {
      setError(errorText(e));
    } finally {
      setBusy(false);
    }
  };
  const metricNames = [
    ...new Set(
      Object.values(batch?.groups ?? {}).flatMap((group) =>
        Object.keys(group.metrics),
      ),
    ),
  ].sort();
  const selectedMetric = metricNames.includes(metric)
    ? metric
    : (metricNames[0] ?? "");
  return (
    <section
      className="comparison-workspace"
      hidden={!visible}
      aria-label="Parameter sweeps"
    >
      <aside className="panel comparison-picker">
        <div className="panel-title">
          <h2>Experiments</h2>
          <span className="tag">{total} saved</span>
        </div>
        <div className="comparison-paging">
          <button
            onClick={() => {
              setSelected(null);
              setBatch(null);
              setError("");
            }}
          >
            New experiment
          </button>
        </div>
        {listError && (
          <p role="alert" className="hint amber">
            {listError}
          </p>
        )}
        {jobs.map((item) => (
          <button
            key={item.id}
            className={`history-item ${selected === item.id ? "selected" : ""}`}
            onClick={() => {
              if (selected === item.id) return;
              setSelected(item.id);
              setBatch(null);
              setError("");
            }}
          >
            <span>
              <strong>{item.name}</strong>
              <small>
                {item.expected_trials} trials · {item.id.slice(-6)}
              </small>
            </span>
            <em>{item.status}</em>
          </button>
        ))}
        {!jobs.length && (
          <p className="hint">Saved batches will appear here.</p>
        )}
        <div className="comparison-paging">
          <button
            disabled={offset === 0}
            onClick={() => setOffset(Math.max(0, offset - 20))}
          >
            Newer
          </button>
          <button
            disabled={offset + 20 >= total}
            onClick={() => setOffset(offset + 20)}
          >
            Older
          </button>
        </div>
      </aside>
      <div className="panel comparison-editor">
        <div className="panel-title editor-title">
          <h2>
            {selected ? "experiment-results.json" : "experiment-setup.json"}
          </h2>
          <span className="tag">PARAMETER & SEED SWEEPS</span>
        </div>
        <div className="comparison-body">
          {!selected && (
            <>
              <h2>Run a parameter sweep</h2>
              <p className="hint">
                Base setup: <strong>{config?.name ?? "Loading…"}</strong>. Every
                value combination runs with the same seed list. Up to 32 trials,
                8 seeds and 2 parameter axes.
              </p>
              {disabled && (
                <p className="hint amber">
                  Apply your draft JSON edits in Simulation before previewing
                  this design.
                </p>
              )}
              <fieldset
                className="sweep-form"
                disabled={disabled || !config || busy || checking}
              >
                <label>
                  Experiment name
                  <input
                    value={name}
                    onChange={(e) => {
                      invalidate();
                      setName(e.target.value);
                    }}
                  />
                </label>
                <label>
                  Seeds, comma-separated
                  <input
                    value={seeds}
                    onChange={(e) => {
                      invalidate();
                      setSeeds(e.target.value);
                    }}
                  />
                </label>
                {axes.map((axis, index) => (
                  <div className="sweep-axis" key={index}>
                    <label>
                      Parameter {index + 1}
                      <input
                        aria-label={`Sweep parameter ${index + 1}`}
                        value={axis.path}
                        onChange={(e) => {
                          invalidate();
                          setAxes(
                            axes.map((a, i) =>
                              i === index ? { ...a, path: e.target.value } : a,
                            ),
                          );
                        }}
                      />
                    </label>
                    <label>
                      Values, comma-separated
                      <input
                        aria-label={`Sweep values ${index + 1}`}
                        value={axis.values}
                        onChange={(e) => {
                          invalidate();
                          setAxes(
                            axes.map((a, i) =>
                              i === index
                                ? { ...a, values: e.target.value }
                                : a,
                            ),
                          );
                        }}
                      />
                    </label>
                    <button
                      aria-label={`Remove parameter ${index + 1}`}
                      onClick={() => {
                        invalidate();
                        setAxes(axes.filter((_, i) => i !== index));
                      }}
                    >
                      Remove
                    </button>
                  </div>
                ))}
                <div className="comparison-actions">
                  <button
                    disabled={axes.length === 2}
                    onClick={() => {
                      invalidate();
                      setAxes([
                        ...axes,
                        { path: "simulation.dt", values: "0.01, 0.02" },
                      ]);
                    }}
                  >
                    Add parameter
                  </button>
                  <button className="primary" onClick={check}>
                    Preview trials
                  </button>
                </div>
              </fieldset>
              <p className="hint">
                Remove both parameter axes for seed-only trials. Parameters use
                config paths such as robot.wheel_radius or sensors.0.latency.
                Values use the parameter's SI units.
              </p>
              {checking && (
                <p role="status">Checking trial configurations and workload…</p>
              )}
              {preview && (
                <>
                  <p role="status" className="hint">
                    {preview.data.expected_trials} trials ·{" "}
                    {preview.data.resources.steps.toLocaleString()} maximum
                    steps · {number(preview.data.resources.duration_s)}{" "}
                    simulated seconds in total.{" "}
                    {
                      preview.data.trials.filter((t) => t.status === "invalid")
                        .length
                    }{" "}
                    invalid trials will be retained without execution.
                  </p>
                  <button
                    className="primary"
                    disabled={busy || disabled}
                    onClick={start}
                  >
                    {busy ? "Submitting…" : "Run sweep"}
                  </button>
                  <div className="comparison-table-scroll">
                    <table className="comparison-table">
                      <caption>Resolved trial design</caption>
                      <thead>
                        <tr>
                          <th>Group</th>
                          <th>Parameters</th>
                          <th>Seed</th>
                          <th>Validation</th>
                        </tr>
                      </thead>
                      <tbody>
                        {preview.data.trials.map((trial) => (
                          <tr key={trial.index}>
                            <td>{trial.group}</td>
                            <td>{JSON.stringify(trial.parameters)}</td>
                            <td>{trial.seed}</td>
                            <td>{trial.error ?? "Ready"}</td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                </>
              )}
            </>
          )}
          {selected && !batch && <p role="status">Loading saved experiment…</p>}
          {batch && (
            <>
              <h2>{batch.name}</h2>
              <p role="status" className="hint">
                {batch.status.replaceAll("_", " ")} · {batch.finished_trials} /{" "}
                {batch.expected_trials} trials finished
              </p>
              <progress
                value={batch.finished_trials}
                max={batch.expected_trials}
                aria-label="Finished experiment trials"
              />
              <div className="comparison-actions">
                {active(batch.status) && (
                  <button
                    disabled={busy || batch.cancel_requested}
                    onClick={cancel}
                  >
                    {batch.cancel_requested ? "Cancelling…" : "Cancel sweep"}
                  </button>
                )}
                <a href={`/api/experiments/${batch.id}/report`}>Report ↓</a>
                <a
                  href={`/api/experiments/${batch.id}/artifacts/experiment.json`}
                >
                  Design ↓
                </a>
                <a href={`/api/experiments/${batch.id}/artifacts/inputs.json`}>
                  Resolved inputs ↓
                </a>
              </div>
              {batch.error && (
                <p role="alert" className="hint amber">
                  {batch.error}
                </p>
              )}
              <h2>Group summary</h2>
              <p className="hint">
                Only finished trials count below. Failure means the trial did
                not complete normally, including collisions, invalid inputs,
                cancellations and exhausted budgets. Metrics include all
                available measurements; a finished batch can contain failed
                trials.
              </p>
              {metricNames.length > 0 && (
                <label className="sweep-metric">
                  Metric
                  <select
                    value={selectedMetric}
                    onChange={(e) => setMetric(e.target.value)}
                  >
                    {metricNames.map((name) => (
                      <option key={name} value={name}>
                        {name.replaceAll("_", " ")}
                      </option>
                    ))}
                  </select>
                </label>
              )}
              <div className="comparison-table-scroll">
                <table className="comparison-table">
                  <caption>
                    Mean and sample standard deviation use measured trials only.
                    Failure interval is Wilson 95%; it does not measure
                    systematic model error.
                  </caption>
                  <thead>
                    <tr>
                      <th>Parameters</th>
                      <th>Finished</th>
                      <th>Failed / rate</th>
                      <th>Failure interval</th>
                      <th>Measured</th>
                      <th>Mean</th>
                      <th>Std. dev.</th>
                    </tr>
                  </thead>
                  <tbody>
                    {Object.entries(batch.groups).map(([name, group]) => {
                      const stat = group.metrics[selectedMetric];
                      const parameters = batch.trials.find(
                        (t) => t.group === name,
                      )?.parameters;
                      return (
                        <tr key={name}>
                          <th scope="row">{JSON.stringify(parameters)}</th>
                          <td>{group.trials}</td>
                          <td>
                            {group.failed_trials} /{" "}
                            {number(group.failure_rate * 100)}%
                          </td>
                          <td>
                            {group.failure_rate_wilson95
                              .map((v) => number(v * 100))
                              .join("–")}
                            %
                          </td>
                          <td>{stat?.measured_trials ?? 0}</td>
                          <td>{number(stat?.mean ?? null)}</td>
                          <td>{number(stat?.sample_stddev ?? null)}</td>
                        </tr>
                      );
                    })}
                  </tbody>
                </table>
              </div>
              <SweepAnalysis
                key={batch.id}
                batch={batch}
                metric={selectedMetric}
                visible={visible}
              />
              <h2>Trials</h2>
              <div className="comparison-table-scroll">
                <table className="comparison-table">
                  <caption>
                    Each submitted trial also appears in Run history and Compare
                    runs.
                  </caption>
                  <thead>
                    <tr>
                      <th>Group / seed</th>
                      <th>Status</th>
                      <th>Run</th>
                    </tr>
                  </thead>
                  <tbody>
                    {batch.trials.map((trial) => (
                      <tr key={trial.index}>
                        <td>
                          {trial.group} · seed {trial.seed}
                        </td>
                        <td>
                          {trial.status.replaceAll("_", " ")}
                          {trial.error && (
                            <details>
                              <summary>Details</summary>
                              <pre>{trial.error}</pre>
                            </details>
                          )}
                        </td>
                        <td>
                          {trial.run_id ? (
                            <>
                              <code>{trial.run_id.slice(-6)}</code>
                              {ready(trial.status) && (
                                <button onClick={() => onReplay(trial.run_id!)}>
                                  Open replay
                                </button>
                              )}
                            </>
                          ) : (
                            "Not submitted"
                          )}
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </>
          )}
          {error && (
            <p role="alert" className="hint amber">
              {error}
            </p>
          )}
        </div>
      </div>
    </section>
  );
}
