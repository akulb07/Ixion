import type { Config, Frame, Trajectory } from "./types";
import { scanSegments, sensorMount } from "./math.mjs";

export function World({
  config,
  frame,
  trajectory,
  showPath,
  showScan,
}: {
  config: Config;
  frame: Frame | null;
  trajectory: Trajectory | null;
  showPath: boolean;
  showScan: boolean;
}) {
  const { width: w, height: h, obstacles } = config.environment;
  const pose = frame?.state.pose ?? config.robot.initial_pose;
  const radius = config.robot.footprint_radius;
  const lidar = frame?.sensors.find((s) => s.kind === "lidar");
  const mount = sensorMount(config.robot, lidar?.frame);
  const capture = lidar ? frame?.sensor_capture_poses[lidar.sensor] : null;
  const segments = lidar && capture ? scanSegments(lidar, capture, mount) : [];
  const scale = Math.max(w, h) / 8;
  return (
    <svg
      className="world"
      viewBox={`${-0.4 * scale} ${-0.4 * scale} ${w + 0.8 * scale} ${h + 0.8 * scale}`}
      role="img"
      aria-label="Simulation world, ground-truth robot pose and recorded sensor rays"
    >
      <defs>
        <pattern
          id="grid"
          width={0.5 * scale}
          height={0.5 * scale}
          patternUnits="userSpaceOnUse"
        >
          <path
            d={`M ${0.5 * scale} 0 L 0 0 0 ${0.5 * scale}`}
            fill="none"
            stroke="#253238"
            strokeWidth={0.008 * scale}
          />
        </pattern>
      </defs>
      <g transform={`translate(0,${h}) scale(1,-1)`}>
        <rect
          width={w}
          height={h}
          rx={0.05 * scale}
          fill="#151e22"
          stroke="#485a62"
          strokeWidth={0.025 * scale}
        />
        <rect width={w} height={h} fill="url(#grid)" />
        {obstacles.map((o, i) =>
          o.type === "rectangle" ? (
            <rect
              key={i}
              x={o.x}
              y={o.y}
              width={o.width}
              height={o.height}
              fill="#35444c"
              stroke="#697d87"
              strokeWidth={0.025 * scale}
              rx={0.025 * scale}
            />
          ) : (
            <circle
              key={i}
              cx={o.x}
              cy={o.y}
              r={o.radius}
              fill="#35444c"
              stroke="#697d87"
              strokeWidth={0.025 * scale}
            />
          ),
        )}
        {showPath && trajectory && (
          <polyline
            points={trajectory.states
              .filter((s) => s.time <= (frame?.state.time ?? 0))
              .map((s) => `${s.pose.x},${s.pose.y}`)
              .join(" ")}
            fill="none"
            stroke="#64ddd1"
            strokeWidth={0.027 * scale}
            strokeLinejoin="round"
          />
        )}
        {showScan &&
          segments.map(
            (
              s: {
                x: number;
                y: number;
                endX: number;
                endY: number;
                hit: boolean;
              },
              i: number,
            ) => (
              <g key={i}>
                <line
                  x1={s.x}
                  y1={s.y}
                  x2={s.endX}
                  y2={s.endY}
                  stroke="#66b8fa"
                  opacity=".12"
                  strokeWidth={0.012 * scale}
                />
                {s.hit && (
                  <circle
                    cx={s.endX}
                    cy={s.endY}
                    r={0.024 * scale}
                    fill="#80c8ff"
                  />
                )}
              </g>
            ),
          )}
        <g
          transform={`translate(${pose.x},${pose.y}) rotate(${(pose.theta * 180) / Math.PI})`}
        >
          <circle r={radius * 1.55} fill="#61d9cd" opacity=".07" />
          <circle
            r={radius}
            fill="#29635e"
            stroke="#80f2e5"
            strokeWidth={0.025 * scale}
          />
          <rect
            x={-radius * 0.65}
            y={-radius * 1.05}
            width={radius * 1.3}
            height={radius * 0.22}
            rx={0.025 * scale}
            fill="#d2e6e5"
          />
          <rect
            x={-radius * 0.65}
            y={radius * 0.83}
            width={radius * 1.3}
            height={radius * 0.22}
            rx={0.025 * scale}
            fill="#d2e6e5"
          />
          <path
            d={`M ${radius * 0.75} 0 L ${-radius * 0.2} ${radius * 0.45} L ${-radius * 0.2} ${-radius * 0.45} Z`}
            fill="#9bfff2"
          />
        </g>
        <path
          d={`M .2 .7 V .2 H .7`}
          fill="none"
          stroke="#84959d"
          strokeWidth={0.018 * scale}
        />
      </g>
      <text x={0.78} y={h - 0.16} className="axis-label">
        x
      </text>
      <text x={0.15} y={h - 0.78} className="axis-label">
        y
      </text>
    </svg>
  );
}
