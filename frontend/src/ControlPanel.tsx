import { useEffect, useRef, useState } from "react";
import { request } from "./api";
import type { Config } from "./types";

type PID = {
  setpoint: number;
  measurement: number;
  error: number;
  proportional: number;
  integral: number;
  derivative: number;
  feedforward: number;
  output: number;
  saturated: boolean;
};
type Sample = { time: number; capture_time: number; left: PID; right: PID };
type Analysis = {
  samples: Sample[];
  metrics: { left: Record<string, number>; right: Record<string, number> };
  total_updates: number;
  sampled: boolean;
  first_update_time: number;
  last_update_time: number;
  run_end_time: number;
};
const fmt = (v: number) =>
  v.toLocaleString("en", { maximumSignificantDigits: 5 });

function Plot({
  data,
  wheel,
  terms,
}: {
  data: Analysis;
  wheel: "left" | "right";
  terms: boolean;
}) {
  const lines: [
    keyof Pick<
      PID,
      | "setpoint"
      | "measurement"
      | "output"
      | "proportional"
      | "integral"
      | "derivative"
      | "feedforward"
    >,
    string,
  ][] = terms
    ? [
        ["proportional", "#b496ff"],
        ["integral", "#8ebca6"],
        ["derivative", "#e9ba79"],
        ["feedforward", "#88b4db"],
      ]
    : [
        ["setpoint", "#b496ff"],
        ["measurement", "#8ebca6"],
        ["output", "#e9ba79"],
      ];
  const values = data.samples.flatMap((s) => lines.map(([k]) => s[wheel][k]));
  const lo = Math.min(0, ...values),
    hi = Math.max(0.01, ...values),
    span = Math.max(hi - lo, 0.01);
  const start = data.first_update_time,
    end = data.last_update_time;
  const x = (t: number) =>
    48 + ((t - start) / Math.max(end - start, 0.001)) * 572;
  const y = (v: number) => 180 - ((v - lo) / span) * 150;
  return (
    <>
      <p className="hint">
        {lines.map(([name, color]) => (
          <span key={name} style={{ color, marginRight: 12 }}>
            {name}
          </span>
        ))}
      </p>
      <svg
        className="odometry-paths"
        viewBox="0 0 640 215"
        role="img"
        aria-label={`${wheel} wheel ${terms ? "PID contributions" : "setpoint, encoder measurement and requested output"} versus controller update time; rad/s`}
      >
        <line x1="48" x2="620" y1={y(0)} y2={y(0)} stroke="#343842" />
        <text x="3" y="30" fill="#b4bbc8" fontSize="10">
          {fmt(hi)}
        </text>
        <text x="3" y="180" fill="#b4bbc8" fontSize="10">
          {fmt(lo)}
        </text>
        {lines.map(([key, color]) => (
          <polyline
            key={key}
            points={data.samples
              .map((s) => `${x(s.time)},${y(s[wheel][key])}`)
              .join(" ")}
            fill="none"
            stroke={color}
            strokeWidth="1.5"
            strokeDasharray={key === "setpoint" ? "4 3" : undefined}
          />
        ))}
        {!terms &&
          data.samples
            .filter((s) => s[wheel].saturated)
            .map((s, i) => (
              <circle
                key={i}
                cx={x(s.time)}
                cy={y(s[wheel].output)}
                r="2.5"
                fill="#f48771"
              />
            ))}
        <text x="48" y="205" fill="#b4bbc8" fontSize="10">
          {fmt(start)} s
        </text>
        <text x="555" y="205" fill="#b4bbc8" fontSize="10">
          {fmt(end)} s
        </text>
      </svg>
    </>
  );
}

export function ControlPanel({
  runId,
  config,
}: {
  runId: string;
  config: Config;
}) {
  const [data, setData] = useState<Analysis | null>(null),
    [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  const [wheel, setWheel] = useState<"left" | "right">("left");
  const pending = useRef<AbortController | null>(null);
  useEffect(() => () => pending.current?.abort(), []);
  const url = `/api/runs/${runId}/control`;
  const analyze = async () => {
    pending.current?.abort();
    const controller = new AbortController();
    pending.current = controller;
    setBusy(true);
    setError("");
    setData(null);
    try {
      const result = await request<Analysis>(url, {
        signal: controller.signal,
      });
      if (!controller.signal.aborted) setData(result);
    } catch (e) {
      if (!controller.signal.aborted)
        setError(e instanceof Error ? e.message : String(e));
    } finally {
      if (!controller.signal.aborted) setBusy(false);
    }
  };
  return (
    <section className="panel" aria-label="Controller response">
      <div className="panel-title">
        <h2>Control · PID response</h2>
        <span className="tag">SAVED TELEMETRY</span>
      </div>
      <div className="odometry-body">
        {!config.wheel_controller ? (
          <p className="hint">
            This run used open-loop commands. Choose the PID feedback laboratory
            or enable wheel feedback for the next run.
          </p>
        ) : (
          <>
            <p className="hint">
              Inspect the feedback actually used by the controller. Encoder
              measurement is an interval-average shaft speed; requested output
              is before motor limits, delay and slip.
            </p>
            <button onClick={analyze} disabled={busy}>
              {busy ? "Analyzing response…" : "Analyze controller"}
            </button>
          </>
        )}
        {error && (
          <p role="alert" className="hint amber">
            {error}
          </p>
        )}
        {data && (
          <>
            <label className="sweep-metric">
              Inspect wheel
              <select
                value={wheel}
                onChange={(e) => setWheel(e.target.value as "left" | "right")}
              >
                <option value="left">Left wheel</option>
                <option value="right">Right wheel</option>
              </select>
            </label>
            <Plot data={data} wheel={wheel} terms={false} />
            <p className="hint">
              Red dots: saturated PID updates. Lines connect recorded samples
              {data.sampled ? " (display downsampled)" : ""}; all{" "}
              {data.total_updates} updates contribute to metrics. Samples cover{" "}
              {fmt(data.first_update_time)}–{fmt(data.last_update_time)} s of a{" "}
              {fmt(data.run_end_time)} s run.
            </p>
            <details>
              <summary>PID contributions</summary>
              <Plot data={data} wheel={wheel} terms={true} />
            </details>
            <div className="comparison-table-scroll">
              <table className="comparison-table">
                <caption>
                  Update-weighted feedback errors and PID saturation; not
                  ground-truth motion error
                </caption>
                <thead>
                  <tr>
                    <th>Metric</th>
                    <th>Left wheel</th>
                    <th>Right wheel</th>
                  </tr>
                </thead>
                <tbody>
                  {Object.keys(data.metrics.left).map((key) => (
                    <tr key={key}>
                      <th scope="row">{key.replaceAll("_", " ")}</th>
                      <td>{fmt(data.metrics.left[key])}</td>
                      <td>{fmt(data.metrics.right[key])}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
            <p className="hint">
              Adjust gains in Wheel feedback, run again, then compare saved
              runs. Saturation percentage counts controller updates, not elapsed
              time. Missing encoder pairs produce no update and may leave output
              held.
            </p>
            <a href={url} download>
              Export controller analysis JSON
            </a>
          </>
        )}
      </div>
    </section>
  );
}
