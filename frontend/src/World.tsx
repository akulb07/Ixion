import type { Config, Frame, Trajectory, PlanningDocument } from "./types";
import { scanSegments, sensorMount } from "./math.mjs";

export function World({
  config,
  frame,
  trajectory,
  showPath,
  showScan,
  plan,
}: {
  config: Config;
  frame: Frame | null;
  trajectory: Trajectory | null;
  showPath: boolean;
  showScan: boolean;
  plan: PlanningDocument | null;
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
      aria-label="Simulation world, ground-truth robot, sensor rays and navigation estimate"
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
            stroke="#3c3e44"
            strokeWidth={0.008 * scale}
          />
        </pattern>
      </defs>
      <g transform={`translate(0,${h}) scale(1,-1)`}>
        <rect
          width={w}
          height={h}
          rx={0.05 * scale}
          fill="#26282d"
          stroke="#606060"
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
              fill="#494c53"
              stroke="#929397"
              strokeWidth={0.025 * scale}
              rx={0.025 * scale}
            />
          ) : (
            <circle
              key={i}
              cx={o.x}
              cy={o.y}
              r={o.radius}
              fill="#494c53"
              stroke="#929397"
              strokeWidth={0.025 * scale}
            />
          ),
        )}
        {config.navigation && (
          <polyline
            points={config.navigation.path
              .map((p) => `${p.x},${p.y}`)
              .join(" ")}
            fill="none"
            stroke="#b9a5c7"
            strokeWidth={0.025 * scale}
            strokeDasharray={`${0.1 * scale} ${0.06 * scale}`}
          >
            <title>Reference route for navigation</title>
          </polyline>
        )}
        {showPath && trajectory && (
          <polyline
            points={trajectory.states
              .filter((s) => s.time <= (frame?.state.time ?? 0))
              .map((s) => `${s.pose.x},${s.pose.y}`)
              .join(" ")}
            fill="none"
            stroke="#8db6a4"
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
                  stroke="#99aebe"
                  opacity=".12"
                  strokeWidth={0.012 * scale}
                />
                {s.hit && (
                  <circle
                    cx={s.endX}
                    cy={s.endY}
                    r={0.024 * scale}
                    fill="#afc0cc"
                  />
                )}
              </g>
            ),
          )}
        <g
          transform={`translate(${pose.x},${pose.y}) rotate(${(pose.theta * 180) / Math.PI})`}
        >
          <circle
            r={radius}
            fill="#6a4c38"
            stroke="#e5af7f"
            strokeWidth={0.025 * scale}
          />
          <rect
            x={-radius * 0.65}
            y={-radius * 1.05}
            width={radius * 1.3}
            height={radius * 0.22}
            rx={0.025 * scale}
            fill="#cccccc"
          />
          <rect
            x={-radius * 0.65}
            y={radius * 0.83}
            width={radius * 1.3}
            height={radius * 0.22}
            rx={0.025 * scale}
            fill="#cccccc"
          />
          <path
            d={`M ${radius * 0.75} 0 L ${-radius * 0.2} ${radius * 0.45} L ${-radius * 0.2} ${-radius * 0.45} Z`}
            fill="#d4d4d4"
          />
        </g>
        {frame?.navigation && (
          <g
            transform={`translate(${frame.navigation.estimate.x},${frame.navigation.estimate.y})`}
          >
            <title>
              Encoder pose estimate at capture time{" "}
              {frame.navigation.estimate_time ?? "unavailable"} s
            </title>
            <path
              d={`M 0 ${radius * 1.5} L ${radius * 1.5} 0 L 0 ${-radius * 1.5} L ${-radius * 1.5} 0 Z`}
              fill="none"
              stroke="#9cbaa7"
              strokeWidth={0.022 * scale}
            />
          </g>
        )}
        <path
          d={`M .2 .7 V .2 H .7`}
          fill="none"
          stroke="#929397"
          strokeWidth={0.018 * scale}
        />
        {plan && (
          <g>
            <title>
              Planned path preview: {plan.result.status}. Path not executed.
            </title>
            <polyline
              points={plan.result.path.map((p) => `${p.x},${p.y}`).join(" ")}
              fill="none"
              stroke="#b9a5c7"
              strokeWidth={0.035 * scale}
              strokeDasharray={`${0.1 * scale} ${0.06 * scale}`}
            />
            <circle
              cx={plan.request.goal.x}
              cy={plan.request.goal.y}
              r={plan.planning_radius_m}
              fill="none"
              stroke="#d7ba7d"
              strokeWidth={0.015 * scale}
              strokeDasharray={`${0.05 * scale} ${0.04 * scale}`}
            />
            <path
              d={`M ${plan.request.goal.x - 0.1 * scale} ${plan.request.goal.y} h ${0.2 * scale} M ${plan.request.goal.x} ${plan.request.goal.y - 0.1 * scale} v ${0.2 * scale}`}
              stroke="#d7ba7d"
              strokeWidth={0.03 * scale}
            />
          </g>
        )}
      </g>
      {plan && (
        <text
          x={0.2 * scale}
          y={0.28 * scale}
          fill="#b9a5c7"
          fontSize={0.12 * scale}
        >
          PLANNED PATH · {plan.result.status.toUpperCase()} · NOT EXECUTED
        </text>
      )}
      <text x={0.78} y={h - 0.16} className="axis-label">
        x
      </text>
      <text x={0.15} y={h - 0.78} className="axis-label">
        y
      </text>
    </svg>
  );
}
