import { useEffect, useRef, useState } from "react";
import { request } from "./api";
import type { Config } from "./types";

type MapResult = {
  pose_source: "encoder" | "truth";
  grid: { columns: number; rows: number; resolution: number };
  states: number[][];
  used_scans: number;
  skipped_scans: number;
  valid_rays: number;
  counts: { unknown: number; free: number; occupied: number };
};
function GridView({ data }: { data: MapResult }) {
  const canvas = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const context = canvas.current?.getContext("2d");
    if (!context) return;
    const { columns, rows } = data.grid;
    const image = context.createImageData(columns, rows);
    for (let y = 0; y < rows; y++)
      for (let x = 0; x < columns; x++) {
        const state = data.states[y][x];
        const color =
          state === 100
            ? [220, 178, 131]
            : state === 0
              ? [222, 228, 218]
              : [69, 73, 79];
        image.data.set([...color, 255], ((rows - 1 - y) * columns + x) * 4);
      }
    context.putImageData(image, 0, 0);
  }, [data]);
  return (
    <canvas
      ref={canvas}
      width={data.grid.columns}
      height={data.grid.rows}
      className="occupancy-canvas"
      role="img"
      aria-label={`Occupancy map using ${data.pose_source} poses: ${data.counts.free} free, ${data.counts.occupied} occupied and ${data.counts.unknown} unknown cells. Origin bottom left, x right and y up.`}
    />
  );
}

export function MappingPanel({
  runId,
  config,
}: {
  runId: string;
  config: Config;
}) {
  const lidars = config.sensors.filter((s) => s.type === "lidar");
  const encoders = config.sensors.filter((s) => s.type === "encoder");
  const [sensor, setSensor] = useState(lidars[0]?.name ?? "");
  const [encoder, setEncoder] = useState(encoders[0]?.name ?? "");
  const [source, setSource] = useState("encoder");
  const [resolution, setResolution] = useState("0.1");
  const [result, setResult] = useState<{ data: MapResult; url: string } | null>(
    null,
  );
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const pending = useRef<AbortController | null>(null);
  useEffect(() => () => pending.current?.abort(), []);
  const build = async () => {
    pending.current?.abort();
    const controller = new AbortController();
    pending.current = controller;
    setBusy(true);
    setError("");
    setResult(null);
    const url = `/api/runs/${runId}/map?${new URLSearchParams({ sensor, pose_source: source, encoder: encoder || "encoders", resolution })}`;
    try {
      const data = await request<MapResult>(url, { signal: controller.signal });
      if (!controller.signal.aborted) setResult({ data, url });
    } catch (e) {
      if (!controller.signal.aborted)
        setError(e instanceof Error ? e.message : String(e));
    } finally {
      if (!controller.signal.aborted) setBusy(false);
    }
  };
  return (
    <section className="panel" aria-label="Occupancy mapping">
      <div className="panel-title">
        <h2>Mapping · Occupancy grid</h2>
        <span className="tag">OFFLINE ANALYSIS</span>
      </div>
      <div className="odometry-body">
        <p className="hint">
          Build a map from saved LiDAR scans. Encoder mode uses only matching
          capture-time estimates. Ground truth is an explicit reference mode;
          neither mode performs SLAM.
        </p>
        {!lidars.length ? (
          <p className="hint">This run has no LiDAR.</p>
        ) : (
          <fieldset className="map-controls" disabled={busy}>
            <label>
              LiDAR
              <select
                value={sensor}
                onChange={(e) => {
                  setSensor(e.target.value);
                  setResult(null);
                }}
              >
                {lidars.map((s) => (
                  <option key={s.name}>{s.name}</option>
                ))}
              </select>
            </label>
            <label>
              Pose source
              <select
                value={source}
                onChange={(e) => {
                  setSource(e.target.value);
                  setResult(null);
                }}
              >
                <option value="encoder">Encoder estimate</option>
                <option value="truth">Ground truth reference</option>
              </select>
            </label>
            {source === "encoder" && (
              <label>
                Map encoder
                <select
                  value={encoder}
                  onChange={(e) => {
                    setEncoder(e.target.value);
                    setResult(null);
                  }}
                >
                  {encoders.map((s) => (
                    <option key={s.name}>{s.name}</option>
                  ))}
                </select>
              </label>
            )}
            <label>
              Cell size (m)
              <input
                type="number"
                min="0.05"
                max="10"
                step="0.05"
                value={resolution}
                onChange={(e) => {
                  setResolution(e.target.value);
                  setResult(null);
                }}
              />
            </label>
            <button onClick={build} disabled={source === "encoder" && !encoder}>
              {busy ? "Building map…" : "Build occupancy map"}
            </button>
          </fieldset>
        )}
        {error && (
          <p role="alert" className="hint amber">
            {error}
          </p>
        )}
        {result && (
          <>
            <p className="hint">
              Pose source:{" "}
              <strong>
                {result.data.pose_source === "truth"
                  ? "GROUND TRUTH REFERENCE"
                  : "ENCODER ESTIMATE"}
              </strong>{" "}
              · pale: free · copper: occupied · grey: unknown/uncertain
            </p>
            <GridView data={result.data} />
            <p className="hint">
              Origin (0, 0) at bottom left · x → · y ↑ ·{" "}
              {result.data.grid.columns} × {result.data.grid.rows} cells ·{" "}
              {result.data.grid.resolution} m/cell
            </p>
            <p role="status">
              {result.data.used_scans} scans used · {result.data.skipped_scans}{" "}
              skipped without a matching pose · {result.data.valid_rays} valid
              rays
            </p>
            <p className="hint">
              {result.data.counts.free} free · {result.data.counts.occupied}{" "}
              occupied · {result.data.counts.unknown} unknown/uncertain cells.
              Only scans delivered by the run end are included. The grid covers
              the configured world, rounded up to whole cells.
            </p>
            <a href={result.url}>Map report ↓</a>
          </>
        )}
      </div>
    </section>
  );
}
