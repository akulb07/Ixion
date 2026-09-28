import { useEffect, useRef, useState } from "react";
import { request } from "./api";
import type { Config, Pose } from "./types";

type Analysis = {
  total_estimates: number;
  dropped_readings: number;
  sampled: boolean;
  last_capture_time: number;
  run_end_time: number;
  metrics: Record<string, number>;
  samples: {
    capture_time: number;
    estimate: Pose;
    truth: Pose;
    position_error_m: number;
  }[];
};
const format = (v: number) =>
  v.toLocaleString("en", { maximumSignificantDigits: 5 });

function Paths({ data }: { data: Analysis }) {
  const poses = data.samples.flatMap((s) => [s.estimate, s.truth]);
  const minX = Math.min(...poses.map((p) => p.x)),
    maxX = Math.max(...poses.map((p) => p.x));
  const minY = Math.min(...poses.map((p) => p.y)),
    maxY = Math.max(...poses.map((p) => p.y));
  const scale = Math.min(
    600 / Math.max(maxX - minX, 0.01),
    200 / Math.max(maxY - minY, 0.01),
  );
  const points = (kind: "estimate" | "truth") =>
    data.samples
      .map(
        (s) =>
          `${320 + (s[kind].x - (minX + maxX) / 2) * scale},${120 - (s[kind].y - (minY + maxY) / 2) * scale}`,
      )
      .join(" ");
  return (
    <svg
      viewBox="0 0 640 250"
      className="odometry-paths"
      role="img"
      aria-label="Encoder estimate and ground-truth trajectory at matched capture times, equal spatial scale"
    >
      <polyline
        points={points("truth")}
        fill="none"
        stroke="#8ebca6"
        strokeWidth="3"
      />
      <polyline
        points={points("estimate")}
        fill="none"
        stroke="#d8aa80"
        strokeWidth="2"
        strokeDasharray="5 3"
      />
      <text x="15" y="242" fill="#c9c6c1" fontSize="11">
        x → · y ↑ · equal spatial scale · span {format(maxX - minX)} ×{" "}
        {format(maxY - minY)} m
      </text>
    </svg>
  );
}

export function OdometryPanel({
  runId,
  config,
}: {
  runId: string;
  config: Config;
}) {
  const encoders = config.sensors.filter((s) => s.type === "encoder");
  const [sensor, setSensor] = useState(encoders[0]?.name ?? "");
  const [data, setData] = useState<Analysis | null>(null);
  const [error, setError] = useState("");
  const [busy, setBusy] = useState(false);
  const pending = useRef<AbortController | null>(null);
  useEffect(() => () => pending.current?.abort(), []);
  const url = `/api/runs/${runId}/odometry?${new URLSearchParams({ sensor })}`;
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
    <section className="panel odometry-panel" aria-label="Odometry analysis">
      <div className="panel-title">
        <h2>Localization · Encoder odometry</h2>
        <span className="tag">OFFLINE ANALYSIS</span>
      </div>
      <div className="odometry-body">
        <p className="hint">
          Reconstruct an estimate from saved encoders using the configured
          initial pose and wheel geometry. Truth is used only for scoring. This
          does not rerun the simulation or change recorded navigation.
        </p>
        {encoders.length ? (
          <div className="comparison-actions">
            <label>
              Encoder stream{" "}
              <select
                value={sensor}
                disabled={busy}
                onChange={(e) => {
                  setSensor(e.target.value);
                  setData(null);
                  setError("");
                }}
              >
                {encoders.map((s) => (
                  <option key={s.name} value={s.name}>
                    {s.name}
                  </option>
                ))}
              </select>
            </label>
            <button onClick={analyze} disabled={busy}>
              {busy ? "Analyzing…" : "Analyze odometry"}
            </button>
          </div>
        ) : (
          <p className="hint">This run has no encoder sensor.</p>
        )}
        {error && (
          <p role="alert" className="hint amber">
            {error}
          </p>
        )}
        {data && (
          <>
            <p className="hint">
              Solid green: truth · dashed copper: encoder estimate
            </p>
            <Paths data={data} />
            <dl className="odometry-metrics">
              {Object.entries(data.metrics).map(([key, value]) => (
                <div key={key}>
                  <dt>{key.replaceAll("_", " ")}</dt>
                  <dd>{format(value)}</dd>
                </div>
              ))}
            </dl>
            <p role="status" className="hint">
              {data.total_estimates} estimates · {data.dropped_readings}{" "}
              incomplete readings · last capture{" "}
              {format(data.last_capture_time)} s / run end{" "}
              {format(data.run_end_time)} s.
            </p>
            <p className="hint">
              Only readings delivered by the run end are used. Metrics use every
              estimate at its capture time; dropouts leave gaps and are not
              scored as zero error.{" "}
              {data.sampled
                ? "The displayed path is sampled; metrics use all estimates."
                : "The displayed path includes every estimate."}{" "}
              Heading error is signed and wrapped in radians.
            </p>
            <a href={url}>Odometry report ↓</a>
          </>
        )}
      </div>
    </section>
  );
}
