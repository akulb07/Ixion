import { useEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { request, post, ready, active } from "./api";
import { World } from "./World";
import { nextTime } from "./math.mjs";
import type { Config, Frame, Job, Preset, Trajectory } from "./types";
import "./style.css";

const pretty = (value: unknown) => JSON.stringify(value, null, 2);
const number = (value: number | undefined | null, digits = 3) =>
  value == null || !Number.isFinite(value) ? "—" : value.toFixed(digits);
const message = (e: unknown) => (e instanceof Error ? e.message : String(e));

function Chart({
  trajectory,
  cursor,
}: {
  trajectory: Trajectory | null;
  cursor: number;
}) {
  if (!trajectory)
    return (
      <div className="empty-chart">
        Run an experiment to inspect recorded wheel motion.
      </div>
    );
  const states = trajectory.states,
    duration = states.at(-1)!.time;
  const values = states.flatMap((s) => [s.wheels.left, s.wheels.right]);
  const lo = Math.min(0, ...values),
    hi = Math.max(1, ...values),
    span = hi - lo || 1;
  const path = (side: "left" | "right") =>
    states
      .map(
        (s) =>
          `${12 + (676 * s.time) / (duration || 1)},${88 - (70 * (s.wheels[side] - lo)) / span}`,
      )
      .join(" ");
  return (
    <svg
      viewBox="0 0 720 110"
      className="chart"
      role="img"
      aria-label="Recorded left and right wheel speeds in radians per second"
    >
      {[18, 53, 88].map((y) => (
        <line
          key={y}
          x1="12"
          x2="688"
          y1={y}
          y2={y}
          stroke="#2a343b"
          strokeDasharray="3 5"
        />
      ))}
      <polyline
        points={path("left")}
        fill="none"
        stroke="#68ded2"
        strokeWidth="2"
      />
      <polyline
        points={path("right")}
        fill="none"
        stroke="#99a4ff"
        strokeWidth="2"
      />
      <line
        x1={12 + (676 * cursor) / (duration || 1)}
        x2={12 + (676 * cursor) / (duration || 1)}
        y1="10"
        y2="90"
        stroke="#e5e9e8"
        opacity=".65"
      />
      <text x="12" y="105">
        0 s
      </text>
      <text x="636" y="105">
        {number(duration, 2)} s
      </text>
      <text x="694" y="22">
        {number(hi, 1)}
      </text>
      <text x="694" y="88">
        {number(lo, 1)}
      </text>
    </svg>
  );
}

function App() {
  const [presets, setPresets] = useState<Preset[]>([]),
    [draft, setDraft] = useState<Config | null>(null);
  const [json, setJson] = useState(""),
    [jsonDirty, setJsonDirty] = useState(false);
  const [version, setVersion] = useState(""),
    [error, setError] = useState(""),
    [notice, setNotice] = useState("");
  const [jobs, setJobs] = useState<Job[]>([]),
    [total, setTotal] = useState(0),
    [limit, setLimit] = useState(20);
  const [selected, setSelected] = useState<string | null>(null),
    [job, setJob] = useState<Job | null>(null);
  const [runConfig, setRunConfig] = useState<Config | null>(null),
    [trajectory, setTrajectory] = useState<Trajectory | null>(null);
  const [frame, setFrame] = useState<Frame | null>(null),
    [cursor, setCursor] = useState(0),
    [playing, setPlaying] = useState(false),
    [speed, setSpeed] = useState(1);
  const [busy, setBusy] = useState(false),
    [showPath, setShowPath] = useState(true),
    [showScan, setShowScan] = useState(true);
  const [tab, setTab] = useState<"sensors" | "faults">("sensors");
  const importInput = useRef<HTMLInputElement>(null);
  const apply = (config: Config) => {
    setDraft(config);
    setJson(pretty(config));
    setJsonDirty(false);
  };
  const select = (id: string | null) => {
    if (id === selected) return;
    setSelected(id);
    setJob(null);
    setRunConfig(null);
    setFrame(null);
    setTrajectory(null);
    setCursor(0);
    setPlaying(false);
    setError("");
  };
  const duration = trajectory?.states.at(-1)?.time ?? 0;
  const displayConfig = selected ? runConfig : draft;

  useEffect(() => {
    let disposed = false;
    Promise.all([
      request<Preset[]>("/api/presets"),
      request<{ version: string; recovery_warnings: string[] }>("/api/health"),
    ])
      .then(([items, health]) => {
        if (disposed) return;
        setPresets(items);
        apply(items[0].config);
        setVersion(health.version);
        if (health.recovery_warnings.length)
          setNotice(
            `${health.recovery_warnings.length} saved run records could not be read.`,
          );
      })
      .catch((e) => {
        if (!disposed) setError(message(e));
      });
    return () => {
      disposed = true;
    };
  }, []);

  useEffect(() => {
    let disposed = false;
    let timer: ReturnType<typeof setTimeout>;
    const refresh = async () => {
      try {
        const list = await request<{ items: Job[]; total: number }>(
          `/api/runs?limit=${limit}`,
        );
        if (!disposed) {
          setJobs(list.items);
          setTotal(list.total);
        }
      } catch (e) {
        if (!disposed) setError(message(e));
      }
      if (!disposed) timer = setTimeout(refresh, 1500);
    };
    void refresh();
    return () => {
      disposed = true;
      clearTimeout(timer);
    };
  }, [limit]);

  useEffect(() => {
    if (!selected) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    const refresh = async () => {
      try {
        const result = await request<Job>(`/api/runs/${selected}`, {
          signal: controller.signal,
        });
        if (controller.signal.aborted) return;
        setJob(result);
        const cfg = await request<Config>(`/api/runs/${selected}/config`, {
          signal: controller.signal,
        });
        if (controller.signal.aborted) return;
        setRunConfig(cfg);
        if (ready(result.status)) {
          const data = await request<Trajectory>(
            `/api/runs/${selected}/trajectory?max_points=3000`,
            { signal: controller.signal },
          );
          if (!controller.signal.aborted) setTrajectory(data);
        } else if (active(result.status)) timer = setTimeout(refresh, 500);
      } catch (e) {
        if (!controller.signal.aborted) setError(message(e));
      }
    };
    void refresh();
    return () => {
      controller.abort();
      clearTimeout(timer);
    };
  }, [selected]);

  useEffect(() => {
    if (!selected || !trajectory) return;
    const controller = new AbortController();
    request<Frame>(`/api/runs/${selected}/frame?time=${cursor}`, {
      signal: controller.signal,
    })
      .then((data) => {
        if (!controller.signal.aborted) setFrame(data);
      })
      .catch((e) => {
        if (!controller.signal.aborted) {
          setError(message(e));
          setPlaying(false);
        }
      });
    return () => controller.abort();
  }, [selected, cursor, trajectory]);

  useEffect(() => {
    if (!playing || !frame || Math.abs(frame.state.time - cursor) > 1e-7)
      return;
    if (cursor >= duration) {
      setPlaying(false);
      return;
    }
    const timer = setTimeout(
      () => setCursor(nextTime(cursor, duration, speed)),
      100,
    );
    return () => clearTimeout(timer);
  }, [playing, frame, cursor, duration, speed]);

  const validate = async (): Promise<Config> => {
    const input = JSON.parse(json);
    const result = await post<{ config: Config }>(
      "/api/config/validate",
      input,
    );
    apply(result.config);
    return result.config;
  };
  const validateDraft = async () => {
    setBusy(true);
    setError("");
    try {
      await validate();
      setNotice("Configuration validated. Ready to run.");
    } catch (e) {
      setError(message(e));
    } finally {
      setBusy(false);
    }
  };
  const run = async () => {
    setBusy(true);
    setError("");
    setNotice("");
    try {
      const config = await validate();
      const result = await post<Job>("/api/runs", config);
      select(result.id);
      setJob(result);
    } catch (e) {
      setError(message(e));
    } finally {
      setBusy(false);
    }
  };
  const edit = (update: (copy: Config) => void) => {
    if (!draft) return;
    const copy = structuredClone(draft);
    update(copy);
    apply(copy);
    setNotice("");
  };
  const cancel = async () => {
    if (!selected) return;
    setBusy(true);
    try {
      const result = await post<Job>(`/api/runs/${selected}/cancel`);
      setJob((current) => (current?.id === result.id ? result : current));
    } catch (e) {
      setError(message(e));
    } finally {
      setBusy(false);
    }
  };
  const importFile = async (file: File | undefined) => {
    if (!file) return;
    if (file.size > 1_000_000) {
      setError("Configuration file exceeds 1 MB.");
      return;
    }
    setBusy(true);
    setError("");
    try {
      const result = await post<{ config: Config }>(
        "/api/config/validate",
        JSON.parse(await file.text()),
      );
      apply(result.config);
      select(null);
      setNotice("Configuration imported and validated.");
    } catch (e) {
      setError(message(e));
    } finally {
      setBusy(false);
      if (importInput.current) importInput.current.value = "";
    }
  };
  const exportConfig = () => {
    const blob = new Blob([json], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = "roboforge-config.json";
    anchor.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand">
          <span className="brand-mark">
            R<span>F</span>
          </span>
          <div>
            ROBOFORGE<small>ROBOTICS EXPERIMENT WORKSPACE</small>
          </div>
        </div>
        <div className="top-context">
          <span className="dot" /> Local workspace{" "}
          <span className="divider">/</span>{" "}
          <span className="muted">
            {version ? `v${version}` : "Connecting…"}
          </span>
        </div>
        <a className="text-link" href="/docs" target="_blank" rel="noreferrer">
          API reference ↗
        </a>
      </header>
      <div className="page-heading">
        <div>
          <div className="eyebrow">EXPERIMENT LAB / DIFFERENTIAL DRIVE</div>
          <h1>Understand every move.</h1>
          <p>Configure, simulate, and inspect the evidence.</p>
        </div>
        <div className="heading-actions">
          <button onClick={() => importInput.current?.click()} disabled={busy}>
            ↑ Import setup
          </button>
          <button className="primary" disabled={busy || !draft} onClick={run}>
            {busy ? "Working…" : "▶ Run experiment"}
          </button>
          <input
            ref={importInput}
            hidden
            type="file"
            accept=".json,application/json"
            onChange={(e) => void importFile(e.target.files?.[0])}
          />
        </div>
      </div>
      {error && (
        <div className="message error" role="alert">
          <span>{error}</span>
          <button aria-label="Dismiss error" onClick={() => setError("")}>
            ×
          </button>
        </div>
      )}
      {notice && (
        <div className="message notice" role="status">
          {notice}
          <button aria-label="Dismiss notice" onClick={() => setNotice("")}>
            ×
          </button>
        </div>
      )}
      <main className="workspace">
        <aside className="panel setup">
          <div className="panel-title">
            <h2>Experiment setup</h2>
            <span className="tag">DRAFT</span>
          </div>
          <div className="setup-body">
            <label>
              START FROM A PRESET
              <select
                aria-label="Experiment preset"
                defaultValue=""
                disabled={busy}
                onChange={(e) => {
                  const preset = presets.find((p) => p.id === e.target.value);
                  if (preset) {
                    apply(preset.config);
                    select(null);
                    setNotice("");
                  }
                }}
              >
                <option value="" disabled>
                  Choose a laboratory
                </option>
                {presets.map((p) => (
                  <option value={p.id} key={p.id}>
                    {p.title}
                  </option>
                ))}
              </select>
            </label>
            {draft && (
              <>
                <label>
                  EXPERIMENT NAME
                  <input
                    value={draft.name}
                    disabled={jsonDirty || busy}
                    onChange={(e) =>
                      edit((c) => {
                        c.name = e.target.value;
                      })
                    }
                  />
                </label>
                <div className="section-label">
                  01 <span>Robot & world</span>
                </div>
                <div className="robot-card">
                  <div className="mini-robot">◉</div>
                  <div>
                    <strong>Differential drive</strong>
                    <small>
                      {number(draft.robot.footprint_radius * 2, 2)} m footprint
                      · {draft.sensors.length} sensors
                    </small>
                  </div>
                </div>
                <div className="pair">
                  <label>
                    SEED
                    <input
                      type="number"
                      min="0"
                      step="1"
                      value={Number.isFinite(draft.seed) ? draft.seed : ""}
                      disabled={jsonDirty || busy}
                      onChange={(e) =>
                        edit((c) => {
                          c.seed = e.target.valueAsNumber;
                        })
                      }
                    />
                  </label>
                  <label>
                    TIME STEP (s)
                    <input
                      type="number"
                      min=".0001"
                      step=".01"
                      value={
                        Number.isFinite(draft.simulation.dt)
                          ? draft.simulation.dt
                          : ""
                      }
                      disabled={jsonDirty || busy}
                      onChange={(e) =>
                        edit((c) => {
                          c.simulation.dt = e.target.valueAsNumber;
                        })
                      }
                    />
                  </label>
                </div>
                <div className="section-label">
                  02 <span>Wheel commands</span>
                </div>
                {draft.commands.map((cmd, i) => (
                  <div className="command" key={i}>
                    <span className="command-caption">
                      SEGMENT {String(i + 1).padStart(2, "0")}
                    </span>
                    <div className="pair">
                      <label>
                        LEFT (rad/s)
                        <input
                          aria-label={`Segment ${i + 1} left wheel`}
                          type="number"
                          step=".5"
                          value={Number.isFinite(cmd.left) ? cmd.left : ""}
                          disabled={jsonDirty || busy}
                          onChange={(e) =>
                            edit((c) => {
                              c.commands[i].left = e.target.valueAsNumber;
                            })
                          }
                        />
                      </label>
                      <label>
                        RIGHT (rad/s)
                        <input
                          aria-label={`Segment ${i + 1} right wheel`}
                          type="number"
                          step=".5"
                          value={Number.isFinite(cmd.right) ? cmd.right : ""}
                          disabled={jsonDirty || busy}
                          onChange={(e) =>
                            edit((c) => {
                              c.commands[i].right = e.target.valueAsNumber;
                            })
                          }
                        />
                      </label>
                    </div>
                    <label>
                      STEPS
                      <input
                        aria-label={`Segment ${i + 1} steps`}
                        type="number"
                        min="1"
                        step="1"
                        value={Number.isFinite(cmd.steps) ? cmd.steps : ""}
                        disabled={jsonDirty || busy}
                        onChange={(e) =>
                          edit((c) => {
                            c.commands[i].steps = e.target.valueAsNumber;
                          })
                        }
                      />
                    </label>
                  </div>
                ))}
                <div className="setup-summary">
                  <span>Planned duration</span>
                  <strong>
                    {number(
                      draft.commands.reduce((n, c) => n + c.steps, 0) *
                        draft.simulation.dt,
                      2,
                    )}{" "}
                    s
                  </strong>
                </div>
                <details>
                  <summary>
                    Advanced configuration <span>JSON</span>
                  </summary>
                  <p className="hint">
                    Edit world geometry, sensors, control and timed faults.
                    Validate to apply changes.
                  </p>
                  <textarea
                    aria-label="Configuration JSON"
                    spellCheck={false}
                    value={json}
                    onChange={(e) => {
                      setJson(e.target.value);
                      setJsonDirty(true);
                    }}
                  />
                  <button onClick={validateDraft} disabled={busy}>
                    Validate setup
                  </button>
                  <button className="subtle" onClick={exportConfig}>
                    Export JSON
                  </button>
                  {jsonDirty && (
                    <p className="hint amber">
                      Unapplied edits. The world preview uses the last applied
                      setup.
                    </p>
                  )}
                </details>
                {selected && (
                  <button className="wide subtle" onClick={() => select(null)}>
                    ← Preview draft world
                  </button>
                )}
              </>
            )}
          </div>
        </aside>
        <section className="center-column">
          <div className="panel viewport">
            <div className="panel-title">
              <div className="title-inline">
                <h2>World view</h2>
                <span className={`tag ${ready(job?.status) ? "teal" : ""}`}>
                  {selected ? (job?.status ?? "LOADING") : "SETUP PREVIEW"}
                </span>
              </div>
              <span className="small mono">
                {displayConfig
                  ? `${displayConfig.environment.width} × ${displayConfig.environment.height} m`
                  : "—"}
              </span>
            </div>
            <div className="world-toolbar">
              <span className="small">
                {selected ? job?.name : draft?.name}
              </span>
              <div>
                <label>
                  <input
                    type="checkbox"
                    checked={showPath}
                    onChange={(e) => setShowPath(e.target.checked)}
                  />{" "}
                  Truth trail
                </label>
                <label>
                  <input
                    type="checkbox"
                    checked={showScan}
                    onChange={(e) => setShowScan(e.target.checked)}
                  />{" "}
                  LiDAR
                </label>
              </div>
            </div>
            <div className="world-container">
              {displayConfig ? (
                <World
                  config={displayConfig}
                  frame={frame}
                  trajectory={trajectory}
                  showPath={showPath}
                  showScan={showScan}
                />
              ) : (
                <div className="empty-chart">Loading world…</div>
              )}
              <div className="world-caption">
                <span className="dot" />{" "}
                {trajectory ? "RECORDED REPLAY" : "CONFIGURED WORLD"}
                <span>SI units · x / y plane</span>
              </div>
            </div>
            <div className="replay">
              <div className="replay-buttons">
                <button
                  aria-label="Return to start"
                  disabled={!trajectory}
                  onClick={() => {
                    setPlaying(false);
                    setCursor(0);
                  }}
                >
                  ↤
                </button>
                <button
                  className="play"
                  aria-label={playing ? "Pause replay" : "Play replay"}
                  disabled={!trajectory}
                  onClick={() => {
                    if (cursor >= duration) setCursor(0);
                    setPlaying(!playing);
                  }}
                >
                  {playing ? "Ⅱ" : "▶"}
                </button>
                <span className="mono" data-testid="replay-time">
                  {number(cursor, 2)}{" "}
                  <span className="muted">/ {number(duration, 2)} s</span>
                </span>
                <select
                  aria-label="Replay speed"
                  value={speed}
                  onChange={(e) => setSpeed(Number(e.target.value))}
                >
                  <option value=".5">0.5×</option>
                  <option value="1">1×</option>
                  <option value="2">2×</option>
                  <option value="4">4×</option>
                </select>
              </div>
              <input
                aria-label="Replay time"
                className="timeline"
                type="range"
                min="0"
                max={duration || 1}
                step=".01"
                value={cursor}
                disabled={!trajectory || duration === 0}
                onChange={(e) => {
                  setPlaying(false);
                  setCursor(Number(e.target.value));
                }}
              />
              {active(job?.status) ? (
                <div className="run-progress" role="status">
                  {job?.cancel_requested
                    ? "Cancellation requested…"
                    : `${job?.status === "queued" ? "Queued" : "Simulating"} — replay will be available when the run finishes.`}
                  <button
                    disabled={busy || job?.cancel_requested}
                    onClick={cancel}
                  >
                    Cancel run
                  </button>
                </div>
              ) : (
                <div className="replay-note">
                  {job?.error ||
                    (trajectory
                      ? "Replay reads saved results. Sensor rays use their capture-time pose."
                      : "Start a run to unlock recorded playback.")}
                </div>
              )}
            </div>
          </div>
          <div className="metrics">
            <div>
              <span>DISTANCE TRAVELED</span>
              <strong>
                {number(job?.metrics?.path_length_m, 2)} <small>m</small>
              </strong>
            </div>
            <div>
              <span>SIMULATED TIME</span>
              <strong>
                {number(job?.metrics?.duration_s, 2)} <small>s</small>
              </strong>
            </div>
            <div>
              <span>COLLISIONS</span>
              <strong>{job?.metrics?.collision_count ?? "—"}</strong>
            </div>
          </div>
          <div className="panel telemetry">
            <div className="panel-title">
              <h2>Wheel motion</h2>
              <div className="chart-legend">
                <span>● Left</span>
                <span>● Right</span>
                <small>rad/s · wheel shafts</small>
              </div>
            </div>
            <Chart trajectory={trajectory} cursor={cursor} />
            <div className="chart-note">
              {trajectory
                ? `${trajectory.total_states} recorded states${trajectory.sampled ? " · display sampled; full data in CSV" : ""}`
                : "Telemetry appears after a completed run."}
            </div>
          </div>
        </section>
        <aside className="right-column">
          <div className="panel inspector">
            <div className="panel-title">
              <h2>State inspector</h2>
              <span className="tag">TRUTH</span>
            </div>
            <div className="pose-grid">
              <div>
                <span>X POSITION</span>
                <strong>
                  {number(frame?.state.pose.x)}
                  <small> m</small>
                </strong>
              </div>
              <div>
                <span>Y POSITION</span>
                <strong>
                  {number(frame?.state.pose.y)}
                  <small> m</small>
                </strong>
              </div>
              <div>
                <span>HEADING</span>
                <strong>
                  {number(
                    frame ? (frame.state.pose.theta * 180) / Math.PI : null,
                    1,
                  )}
                  <small> °</small>
                </strong>
              </div>
              <div>
                <span>LINEAR SPEED</span>
                <strong>
                  {number(frame?.state.twist.linear)}
                  <small> m/s</small>
                </strong>
              </div>
            </div>
            <div className="inspector-note">
              {frame
                ? `Frame at ${number(frame.state.time, 2)} s`
                : "No replay frame selected"}{" "}
              · simulator ground truth
            </div>
            <div className="tabs" role="tablist" aria-label="Inspector">
              <button
                role="tab"
                aria-selected={tab === "sensors"}
                onClick={() => setTab("sensors")}
              >
                Sensors
              </button>
              <button
                role="tab"
                aria-selected={tab === "faults"}
                onClick={() => setTab("faults")}
              >
                Faults <span>{displayConfig?.faults.length ?? 0}</span>
              </button>
            </div>
            <div className="inspector-content">
              {tab === "sensors" ? (
                displayConfig?.sensors.length ? (
                  displayConfig.sensors.map((sensor) => {
                    const reading = frame?.sensors.find(
                      (s) => s.sensor === sensor.name,
                    );
                    return (
                      <div className="sensor-card" key={sensor.name}>
                        <div>
                          <strong>{sensor.name}</strong>
                          <span className="tag">{sensor.rate_hz} Hz</span>
                        </div>
                        <p>
                          {sensor.type === "encoder"
                            ? `${reading?.left_ticks ?? "—"} / ${reading?.right_ticks ?? "—"} ticks`
                            : sensor.type === "imu"
                              ? `${number(reading?.gyro_z)} rad/s · gyro z`
                              : `${reading?.ranges?.filter((r) => r !== null).length ?? "—"} valid rays`}
                        </p>
                        <small>
                          {reading
                            ? `Captured ${number(reading.capture_time, 2)} s · delivered ${number(reading.delivery_time, 2)} s`
                            : "Awaiting delivered measurement"}
                        </small>
                      </div>
                    );
                  })
                ) : (
                  <p className="hint">No sensors configured.</p>
                )
              ) : (
                <>
                  <p className="hint">
                    Timed faults saved with this configuration.
                  </p>
                  {displayConfig?.faults.length ? (
                    displayConfig.faults.map((fault) => (
                      <div className="sensor-card" key={fault.name}>
                        <div>
                          <strong>{fault.name}</strong>
                          <span
                            className={`tag ${frame?.active_faults.some((f) => f.name === fault.name) ? "amber" : ""}`}
                          >
                            {frame?.active_faults.some(
                              (f) => f.name === fault.name,
                            )
                              ? "ACTIVE"
                              : "SCHEDULED"}
                          </span>
                        </div>
                        <p>
                          {fault.kind.replaceAll("_", " ")} · {fault.target}
                        </p>
                        <small>
                          {fault.start}–{fault.end ?? "end"} s · magnitude{" "}
                          {fault.magnitude}
                        </small>
                      </div>
                    ))
                  ) : (
                    <p className="hint">
                      No faults scheduled. Try the wheel slip preset.
                    </p>
                  )}
                </>
              )}
            </div>
          </div>
          <div className="panel history">
            <div className="panel-title">
              <h2>Run history</h2>
              <span className="count">{total}</span>
            </div>
            <div className="history-list">
              {jobs.length ? (
                jobs.map((item) => (
                  <button
                    key={item.id}
                    className={`history-item ${selected === item.id ? "selected" : ""}`}
                    onClick={() => select(item.id)}
                  >
                    <span>
                      <strong>{item.name}</strong>
                      <small>
                        {new Date(item.created_utc).toLocaleTimeString([], {
                          hour: "2-digit",
                          minute: "2-digit",
                        })}{" "}
                        · seed {item.seed} · {item.id.slice(-6)}
                      </small>
                    </span>
                    <span
                      className={`status-dot ${item.status}`}
                      title={item.status}
                    />
                    <em>{item.status}</em>
                  </button>
                ))
              ) : (
                <p className="hint">
                  Your experiments will appear here. Runs are saved locally.
                </p>
              )}
            </div>
            {total > limit && (
              <button
                className="wide subtle"
                onClick={() => setLimit(Math.min(100, limit + 20))}
                disabled={limit >= 100}
              >
                Show more runs
              </button>
            )}
            {selected && (
              <div className="downloads">
                {runConfig && (
                  <button
                    className="subtle"
                    onClick={() => {
                      apply(runConfig);
                      select(null);
                    }}
                  >
                    Use setup
                  </button>
                )}
                <a href={`/api/runs/${selected}/artifacts/config.json`}>
                  Setup ↓
                </a>
                {ready(job?.status) && (
                  <>
                    <a href={`/api/runs/${selected}/artifacts/trajectory.csv`}>
                      Trajectory ↓
                    </a>
                    <a href={`/api/runs/${selected}/artifacts/manifest.json`}>
                      Manifest ↓
                    </a>
                  </>
                )}
              </div>
            )}
          </div>
        </aside>
      </main>
      <footer>
        <span>
          <span className="dot" /> Deterministic simulation · persisted results
        </span>
        <span>RoboForge / Research workspace</span>
      </footer>
    </div>
  );
}
createRoot(document.getElementById("root")!).render(<App />);
