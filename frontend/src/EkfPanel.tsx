import { useEffect, useRef, useState } from "react";
import { request } from "./api";
import type { Config, Pose } from "./types";

type Analysis = {
  metrics: Record<string, number>;
  odometry_metrics: Record<string, number>;
  total_estimates: number;
  sampled: boolean;
  gyro_accepted: number;
  gyro_rejected: number;
  encoder_only_intervals: number;
  dropped_encoder_readings: number;
  last_capture_time: number;
  run_end_time: number;
  final_position_ellipse95: { x: number; y: number }[];
  samples: {
    capture_time: number;
    estimate: Pose;
    odometry: Pose;
    truth: Pose;
    covariance: number[][];
  }[];
};
const format = (v: number) =>
  v.toLocaleString("en", { maximumSignificantDigits: 5 });

function Paths({ data }: { data: Analysis }) {
  const positions = [
    ...data.samples.flatMap((s) => [s.estimate, s.odometry, s.truth]),
    ...data.final_position_ellipse95,
  ];
  const minX = Math.min(...positions.map((p) => p.x)),
    maxX = Math.max(...positions.map((p) => p.x));
  const minY = Math.min(...positions.map((p) => p.y)),
    maxY = Math.max(...positions.map((p) => p.y));
  const scale = Math.min(
    600 / Math.max(maxX - minX, 0.01),
    220 / Math.max(maxY - minY, 0.01),
  );
  const points = (poses: { x: number; y: number }[]) =>
    poses
      .map(
        (p) =>
          `${320 + (p.x - (minX + maxX) / 2) * scale},${130 - (p.y - (minY + maxY) / 2) * scale}`,
      )
      .join(" ");
  return (
    <svg
      viewBox="0 0 640 270"
      className="odometry-paths"
      role="img"
      aria-label="Truth, encoder odometry and EKF trajectories with the final model-based 95 percent position ellipse; equal spatial scale"
    >
      <polygon
        points={points(data.final_position_ellipse95)}
        fill="#88b4db"
        fillOpacity="0.15"
        stroke="#88b4db"
        strokeWidth="1"
      />
      <polyline
        points={points(data.samples.map((s) => s.truth))}
        fill="none"
        stroke="#8ebca6"
        strokeWidth="3"
      />
      <polyline
        points={points(data.samples.map((s) => s.odometry))}
        fill="none"
        stroke="#d8aa80"
        strokeWidth="2"
        strokeDasharray="5 3"
      />
      <polyline
        points={points(data.samples.map((s) => s.estimate))}
        fill="none"
        stroke="#88b4db"
        strokeWidth="2"
      />
      <text x="12" y="261" fill="#c9c6c1" fontSize="11">
        x → · y ↑ · equal spatial scale
      </text>
    </svg>
  );
}

export function EkfPanel({ runId, config }: { runId: string; config: Config }) {
  const encoders = config.sensors.filter((s) => s.type === "encoder"),
    imus = config.sensors.filter((s) => s.type === "imu");
  const [encoder, setEncoder] = useState(encoders[0]?.name ?? "");
  const [imu, setImu] = useState(imus[0]?.name ?? "");
  const [wheel, setWheel] = useState("0.001"),
    [gyro, setGyro] = useState("0.01");
  const [result, setResult] = useState<{ data: Analysis; url: string } | null>(
    null,
  );
  const [busy, setBusy] = useState(false),
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
    const url = `/api/runs/${runId}/ekf?${new URLSearchParams({ encoder, imu, wheel_variance: wheel, gyro_stddev: gyro })}`;
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
  const data = result?.data;
  return (
    <section className="panel" aria-label="EKF localization">
      <div className="panel-title">
        <h2>Localization · Encoder + IMU EKF</h2>
        <span className="tag">OFFLINE ANALYSIS</span>
      </div>
      <div className="odometry-body">
        <p className="hint">
          Fuse encoder motion with matching gyro samples. Initial pose comes
          from the saved setup; truth only scores the result. Gyro updates can
          reduce heading drift but provide no absolute position fix.
        </p>
        {!encoders.length || !imus.length ? (
          <p className="hint">
            This analysis needs both an encoder and an IMU.
          </p>
        ) : (
          <fieldset className="map-controls" disabled={busy}>
            <label>
              EKF encoder
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
              EKF IMU
              <select
                value={imu}
                onChange={(e) => change(setImu, e.target.value)}
              >
                {imus.map((s) => (
                  <option key={s.name}>{s.name}</option>
                ))}
              </select>
            </label>
            <label>
              Wheel variance per metre (m)
              <input
                type="number"
                min="0"
                max="1"
                step="0.001"
                value={wheel}
                onChange={(e) => change(setWheel, e.target.value)}
              />
            </label>
            <label>
              Gyro standard deviation (rad/s)
              <input
                type="number"
                min="0.000001"
                max="10"
                step="0.01"
                value={gyro}
                onChange={(e) => change(setGyro, e.target.value)}
              />
            </label>
            <button onClick={analyze}>
              {busy ? "Analyzing EKF…" : "Analyze EKF"}
            </button>
          </fieldset>
        )}
        <p className="hint">
          Assumed initial standard deviation: 0.01 m in x/y, 0.01 rad in
          heading. Gyro innovation gate: 25. Noise settings describe the
          estimator's assumptions and do not change the saved sensor data.
        </p>
        {error && (
          <p role="alert" className="hint amber">
            {error}
          </p>
        )}
        {data && (
          <>
            <p className="hint">
              Green: truth · dashed copper: odometry · blue: EKF and final
              position ellipse
            </p>
            <Paths data={data} />
            <p className="hint">
              The ellipse is the assumed Gaussian model's 95% position contour,
              not a guarantee that truth lies inside it. Bias, slip and model
              mismatch can make the filter overconfident.
            </p>
            <div className="comparison-table-scroll">
              <table className="comparison-table">
                <caption>
                  Matched capture-time errors · all estimates, including those
                  omitted from the plotted path
                </caption>
                <thead>
                  <tr>
                    <th>Metric</th>
                    <th>Encoder only</th>
                    <th>EKF</th>
                  </tr>
                </thead>
                <tbody>
                  {Object.entries(data.metrics).map(([name, value]) => (
                    <tr key={name}>
                      <th scope="row">{name.replaceAll("_", " ")}</th>
                      <td>{format(data.odometry_metrics[name])}</td>
                      <td>{format(value)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
            <p role="status">
              {data.gyro_accepted} gyro updates accepted · {data.gyro_rejected}{" "}
              rejected · {data.encoder_only_intervals} intervals without a
              usable gyro update
            </p>
            <p className="hint">
              {data.total_estimates} estimates · {data.dropped_encoder_readings}{" "}
              incomplete encoder readings · last capture{" "}
              {format(data.last_capture_time)} s / end{" "}
              {format(data.run_end_time)} s.{" "}
              {data.sampled
                ? "The plotted path is sampled."
                : "Every estimate is plotted."}{" "}
              Only readings delivered by run end are used. Missing encoder
              intervals use encoder prediction on recovery; an endpoint gyro
              cannot represent the entire gap.
            </p>
            <a href={result.url}>EKF report ↓</a>
          </>
        )}
      </div>
    </section>
  );
}
