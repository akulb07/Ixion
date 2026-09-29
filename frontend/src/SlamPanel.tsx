import { useEffect, useRef, useState } from "react";
import { request } from "./api";
import type { Config, Pose } from "./types";

type Sample = {
  capture_time: number;
  estimate: Pose;
  odometry: Pose;
  truth: Pose;
  status: string;
  map_updated: boolean;
  rejection_reasons: string[];
  match: {
    status: string;
    iterations: number;
    pairs: number;
    rmse: number | null;
    residual_history: number[];
  } | null;
};
type Analysis = {
  samples: Sample[];
  map_points: number[][];
  counts: Record<string, number>;
  rejection_counts: Record<string, number>;
  skipped_scans: { capture_time: number; reason: string }[];
  metrics: Record<string, number>;
  odometry_metrics: Record<string, number>;
};
const format = (v: number) =>
  v.toLocaleString("en", { maximumSignificantDigits: 5 });
const label = (v: string) => v.replaceAll("_", " ");

function Paths({ data, selected }: { data: Analysis; selected: number }) {
  const all = [
    ...data.map_points.map(([x, y]) => ({ x, y })),
    ...data.samples.flatMap((s) => [s.truth, s.odometry, s.estimate]),
  ];
  const minX = Math.min(...all.map((p) => p.x)),
    maxX = Math.max(...all.map((p) => p.x));
  const minY = Math.min(...all.map((p) => p.y)),
    maxY = Math.max(...all.map((p) => p.y));
  const scale = Math.min(
    600 / Math.max(maxX - minX, 0.01),
    260 / Math.max(maxY - minY, 0.01),
  );
  const x = (v: number) => 320 + (v - (minX + maxX) / 2) * scale;
  const y = (v: number) => 150 - (v - (minY + maxY) / 2) * scale;
  const point = data.samples[selected];
  return (
    <svg
      viewBox="0 0 640 320"
      className="odometry-paths"
      role="img"
      aria-label="Final accepted SLAM map points and truth, odometry and SLAM paths; selected scan marked with a ring; equal spatial scale"
    >
      {data.map_points.map(([px, py], i) => (
        <circle key={i} cx={x(px)} cy={y(py)} r="1.5" fill="#737d90" />
      ))}
      {(
        [
          ["truth", "#8ebca6"],
          ["odometry", "#d8aa80"],
          ["estimate", "#b496ff"],
        ] as const
      ).map(([key, color]) => (
        <polyline
          key={key}
          points={data.samples
            .map((s) => `${x(s[key].x)},${y(s[key].y)}`)
            .join(" ")}
          fill="none"
          stroke={color}
          strokeWidth="2"
          strokeDasharray={key === "odometry" ? "5 3" : undefined}
        />
      ))}
      {data.samples
        .filter((s) => s.status === "rejected")
        .map((s) => (
          <circle
            key={s.capture_time}
            cx={x(s.estimate.x)}
            cy={y(s.estimate.y)}
            r="3"
            fill="#f48771"
          />
        ))}
      <circle
        cx={x(point.estimate.x)}
        cy={y(point.estimate.y)}
        r="7"
        fill="none"
        stroke="#dce0e8"
        strokeWidth="1.5"
      />
      <text x="12" y="310" fill="#b4bbc8" fontSize="11">
        x → · y ↑ · metres · final accepted map
      </text>
    </svg>
  );
}

export function SlamPanel({
  runId,
  config,
}: {
  runId: string;
  config: Config;
}) {
  const lidars = config.sensors.filter((s) => s.type === "lidar"),
    encoders = config.sensors.filter((s) => s.type === "encoder");
  const [sensor, setSensor] = useState(lidars[0]?.name ?? ""),
    [encoder, setEncoder] = useState(encoders[0]?.name ?? "");
  const [resolution, setResolution] = useState("0.1"),
    [gate, setGate] = useState("0.1");
  const [result, setResult] = useState<{ data: Analysis; url: string } | null>(
    null,
  );
  const [selected, setSelected] = useState(0),
    [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  const pending = useRef<AbortController | null>(null);
  useEffect(() => () => pending.current?.abort(), []);
  const change = (setter: (v: string) => void, value: string) => {
    setter(value);
    setResult(null);
    setError("");
  };
  const analyze = async () => {
    pending.current?.abort();
    const controller = new AbortController();
    pending.current = controller;
    setBusy(true);
    setError("");
    setResult(null);
    setSelected(0);
    const url = `/api/runs/${runId}/slam?${new URLSearchParams({ sensor, encoder, resolution, max_match_rmse: gate })}`;
    try {
      const data = await request<Analysis>(url, { signal: controller.signal });
      if (!controller.signal.aborted) setResult({ data, url });
    } catch (e) {
      if (!controller.signal.aborted)
        setError(e instanceof Error ? e.message : String(e));
    } finally {
      if (!controller.signal.aborted) setBusy(false);
    }
  };
  const data = result?.data,
    sample = data?.samples[selected];
  return (
    <section className="panel" aria-label="SLAM inspection">
      <div className="panel-title">
        <h2>SLAM · Scan matching</h2>
        <span className="tag">OFFLINE ANALYSIS</span>
      </div>
      <div className="odometry-body">
        <p className="hint">
          Match LiDAR scans against accepted map points using encoder priors.
          Truth only scores the result. This local SLAM front end has no loop
          closure or global relocalization.
        </p>
        {!lidars.length || !encoders.length ? (
          <p className="hint">
            This analysis needs LiDAR and encoder readings.
          </p>
        ) : (
          <fieldset className="map-controls" disabled={busy}>
            <label>
              SLAM LiDAR
              <select
                value={sensor}
                onChange={(e) => change(setSensor, e.target.value)}
              >
                {lidars.map((s) => (
                  <option key={s.name}>{s.name}</option>
                ))}
              </select>
            </label>
            <label>
              SLAM encoder
              <select
                value={encoder}
                onChange={(e) => change(setEncoder, e.target.value)}
              >
                {encoders.map((s) => (
                  <option key={s.name}>{s.name}</option>
                ))}
              </select>
            </label>
            <label>
              SLAM cell size (m)
              <input
                type="number"
                min="0.05"
                max="10"
                step="0.05"
                value={resolution}
                onChange={(e) => change(setResolution, e.target.value)}
              />
            </label>
            <label>
              Maximum match RMSE (m)
              <input
                type="number"
                min="0.001"
                max="1"
                step="0.01"
                value={gate}
                onChange={(e) => change(setGate, e.target.value)}
              />
            </label>
            <button onClick={analyze}>
              {busy ? "Matching scans…" : "Analyze SLAM"}
            </button>
          </fieldset>
        )}
        <p className="hint">
          Up to 300 points per cloud and 25 ICP iterations. Correction gates:
          0.5 m and 0.3 rad. Rejected scans propagate odometry and add no map
          evidence.
        </p>
        {error && (
          <p role="alert" className="hint amber">
            {error}
          </p>
        )}
        {data && sample && (
          <>
            <p className="hint">
              {data.counts.initialized ?? 0} initialized ·{" "}
              {data.counts.matched ?? 0} matched · {data.counts.rejected ?? 0}{" "}
              rejected · {data.skipped_scans.length} skipped without a matching
              encoder prior
            </p>
            <p className="hint">
              Green: truth · dashed copper: odometry · purple: SLAM · red dots:
              rejected scans · grey: final accepted map points
            </p>
            <Paths data={data} selected={selected} />
            <label>
              Inspect scan
              <select
                value={selected}
                onChange={(e) => setSelected(Number(e.target.value))}
              >
                {data.samples.map((s, i) => (
                  <option key={s.capture_time} value={i}>
                    {format(s.capture_time)} s · {s.status}
                  </option>
                ))}
              </select>
            </label>
            <p className="hint">
              {sample.map_updated ? "Map updated" : "Map unchanged"} ·{" "}
              {sample.rejection_reasons.map(label).join(", ") || sample.status}.{" "}
              {sample.match
                ? `${sample.match.pairs} pairs · ${sample.match.iterations} iterations · RMSE ${sample.match.rmse === null ? "unavailable" : format(sample.match.rmse) + " m"}`
                : "No ICP match attempted during initialization."}
            </p>
            {sample.match && (
              <details>
                <summary>Iteration residuals</summary>
                <p className="hint">
                  {sample.match.residual_history.length
                    ? sample.match.residual_history
                        .map((v, i) => `${i + 1}: ${format(v)} m`)
                        .join(" · ")
                    : "No valid residual was produced."}
                </p>
              </details>
            )}
            <div className="comparison-table-scroll">
              <table className="comparison-table">
                <caption>
                  Errors at the same processed scan times, including rejected
                  matches
                </caption>
                <thead>
                  <tr>
                    <th>Metric</th>
                    <th>Encoder only</th>
                    <th>SLAM</th>
                  </tr>
                </thead>
                <tbody>
                  {Object.entries(data.metrics).map(([name, value]) => (
                    <tr key={name}>
                      <th scope="row">{label(name)}</th>
                      <td>{format(data.odometry_metrics[name])}</td>
                      <td>{format(value)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
            <details>
              <summary>Failure diagnostics</summary>
              <p className="hint">
                Insufficient correspondences: too few nearby hits. Degenerate:
                collapsed or collinear geometry. Iteration limit: convergence
                was not reached. Residual, translation and rotation gates reject
                matches beyond the configured limits. Multiple reasons can apply
                to one scan.
              </p>
              <div className="comparison-table-scroll">
                <table className="comparison-table">
                  <caption>Rejected and skipped scans</caption>
                  <thead>
                    <tr>
                      <th>Capture time (s)</th>
                      <th>Reason</th>
                    </tr>
                  </thead>
                  <tbody>
                    {data.samples
                      .filter((s) => s.status === "rejected")
                      .map((s) => (
                        <tr key={s.capture_time}>
                          <td>{format(s.capture_time)}</td>
                          <td>{s.rejection_reasons.map(label).join(", ")}</td>
                        </tr>
                      ))}
                    {data.skipped_scans.map((s) => (
                      <tr key={s.capture_time}>
                        <td>{format(s.capture_time)}</td>
                        <td>{label(s.reason)}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
              {!data.counts.rejected && !data.skipped_scans.length && (
                <p className="hint">No rejected or skipped scans.</p>
              )}
            </details>
            <a href={result.url} download>
              Export SLAM analysis JSON
            </a>
          </>
        )}
      </div>
    </section>
  );
}
