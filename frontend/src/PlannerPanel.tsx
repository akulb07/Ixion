import { useEffect, useRef, useState } from "react";
import { request } from "./api";
import type { Config, Navigation, PlanningDocument } from "./types";

const labels: Record<string, string> = {
  astar: "A*",
  dijkstra: "Dijkstra",
  rrt: "RRT",
  rrt_star: "RRT*",
};
export function PlannerPanel({
  config,
  disabled,
  onPlan,
  onNavigate,
}: {
  config: Config;
  disabled: boolean;
  onPlan: (plan: PlanningDocument | null) => void;
  onNavigate: (navigation: Navigation) => Promise<void>;
}) {
  const [goalX, setGoalX] = useState(
    Math.max(0, config.environment.width - 0.75),
  );
  const [goalY, setGoalY] = useState(
    Math.max(0, config.environment.height - 0.75),
  );
  const [algorithm, setAlgorithm] = useState("astar"),
    [budget, setBudget] = useState(500),
    [clearance, setClearance] = useState(0.1),
    [resolution, setResolution] = useState(0.25);
  const [result, setResult] = useState<PlanningDocument | null>(null),
    [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  const pending = useRef<AbortController | null>(null);
  const [maxSteps, setMaxSteps] = useState(5000),
    [maxSpeed, setMaxSpeed] = useState(0.25),
    [lookahead, setLookahead] = useState(0.25);
  const encoder = config.sensors.find((sensor) => sensor.type === "encoder");
  const invalidate = () => {
    pending.current?.abort();
    setBusy(false);
    setResult(null);
    onPlan(null);
    setError("");
  };
  useEffect(() => {
    pending.current?.abort();
    setBusy(false);
    setResult(null);
    onPlan(null);
    setError("");
    return () => pending.current?.abort();
  }, [config, disabled, onPlan]);
  const plan = async () => {
    invalidate();
    setBusy(true);
    const controller = new AbortController();
    pending.current = controller;
    try {
      const data = await request<PlanningDocument>("/api/plans", {
        method: "POST",
        signal: controller.signal,
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          environment: config.environment,
          start: {
            x: config.robot.initial_pose.x,
            y: config.robot.initial_pose.y,
          },
          goal: { x: goalX, y: goalY },
          footprint_radius: config.robot.footprint_radius,
          clearance,
          algorithm,
          seed: config.seed,
          budget,
          resolution,
        }),
      });
      if (!controller.signal.aborted) {
        setResult(data);
        onPlan(data);
      }
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
    const a = document.createElement("a");
    a.href = url;
    a.download = `roboforge-plan-${algorithm}.json`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  const numeric = (value: number) => (Number.isFinite(value) ? value : "");
  return (
    <section className="planner-panel" aria-label="Planning controls">
      <p className="hint">
        Find a route from the draft start position, then run it with
        encoder-guided path following.
      </p>
      <fieldset disabled={disabled || busy}>
        <label>
          PLANNER
          <select
            aria-label="Path planner"
            value={algorithm}
            onChange={(e) => {
              invalidate();
              setAlgorithm(e.target.value);
            }}
          >
            {Object.entries(labels).map(([value, label]) => (
              <option key={value} value={value}>
                {label}
              </option>
            ))}
          </select>
        </label>
        <div className="pair">
          <label>
            GOAL X (m)
            <input
              aria-label="Goal X"
              type="number"
              step=".25"
              value={numeric(goalX)}
              onChange={(e) => {
                invalidate();
                setGoalX(e.target.valueAsNumber);
              }}
            />
          </label>
          <label>
            GOAL Y (m)
            <input
              aria-label="Goal Y"
              type="number"
              step=".25"
              value={numeric(goalY)}
              onChange={(e) => {
                invalidate();
                setGoalY(e.target.valueAsNumber);
              }}
            />
          </label>
        </div>
        <div className="pair">
          <label>
            EXTRA CLEARANCE (m)
            <input
              aria-label="Extra clearance"
              type="number"
              min="0"
              step=".05"
              value={numeric(clearance)}
              onChange={(e) => {
                invalidate();
                setClearance(e.target.valueAsNumber);
              }}
            />
          </label>
          <label>
            {algorithm === "astar" || algorithm === "dijkstra"
              ? "MAX EXPANSIONS"
              : "ITERATIONS"}
            <input
              aria-label="Planning budget"
              type="number"
              min="1"
              max="1000"
              step="1"
              value={numeric(budget)}
              onChange={(e) => {
                invalidate();
                setBudget(e.target.valueAsNumber);
              }}
            />
          </label>
        </div>
        {(algorithm === "astar" || algorithm === "dijkstra") && (
          <label>
            GRID RESOLUTION (m)
            <input
              aria-label="Grid resolution"
              type="number"
              min=".05"
              step=".05"
              value={numeric(resolution)}
              onChange={(e) => {
                invalidate();
                setResolution(e.target.valueAsNumber);
              }}
            />
          </label>
        )}
        <button className="wide" onClick={plan}>
          Find path
        </button>
      </fieldset>
      {busy && (
        <p className="hint" role="status">
          Searching within the configured budget…
        </p>
      )}
      {disabled && (
        <p className="hint">
          Return to the draft preview and apply any JSON edits to plan.
        </p>
      )}
      {error && (
        <p className="hint amber" role="alert">
          {error}
        </p>
      )}
      {result && (
        <div className="plan-result" role="status">
          <strong>
            {labels[result.result.algorithm]} ·{" "}
            {result.result.status.replaceAll("_", " ")}
          </strong>
          {result.result.status === "success" ? (
            <p>
              {result.result.length.toFixed(3)} m · {result.result.path.length}{" "}
              waypoints
            </p>
          ) : (
            <p>
              {result.result.status === "budget_exceeded"
                ? "Search budget exhausted. This does not prove no path exists."
                : result.result.status === "no_path"
                  ? "No route found on this grid. A different resolution may change connectivity."
                  : "The start or goal footprint intersects an obstacle or boundary."}
            </p>
          )}
          <small>
            {result.result.expanded} expansions ·{" "}
            {result.result.collision_checks} collision checks
            <br />
            Planning radius {result.planning_radius_m.toFixed(3)} m · seed{" "}
            {result.request.seed}
          </small>
          <button className="wide subtle" onClick={download}>
            Export planning result ↓
          </button>
        </div>
      )}
      {result?.result.status === "success" && result.result.path.length > 1 && (
        <fieldset className="navigation-controls" disabled={disabled || busy}>
          <p className="hint">
            Pure Pursuit · encoder: {encoder?.name ?? "none configured"}. Uses
            the draft sensors, actuators and faults with collision stopping.
          </p>
          <div className="pair">
            <label>
              MAX SPEED (m/s)
              <input
                aria-label="Navigation max speed"
                type="number"
                min="0.01"
                max="5"
                step="0.05"
                value={numeric(maxSpeed)}
                onChange={(e) => setMaxSpeed(e.target.valueAsNumber)}
              />
            </label>
            <label>
              LOOKAHEAD (m)
              <input
                aria-label="Navigation lookahead"
                type="number"
                min="0.01"
                max="10"
                step="0.05"
                value={numeric(lookahead)}
                onChange={(e) => setLookahead(e.target.valueAsNumber)}
              />
            </label>
          </div>
          <label>
            MAX STEPS
            <input
              aria-label="Navigation max steps"
              type="number"
              min="1"
              max="50000"
              value={numeric(maxSteps)}
              onChange={(e) => setMaxSteps(e.target.valueAsNumber)}
            />
          </label>
          <button
            className="wide primary"
            disabled={
              !encoder ||
              !Number.isInteger(maxSteps) ||
              maxSteps < 1 ||
              maxSteps > 50000 ||
              !Number.isFinite(maxSpeed) ||
              maxSpeed <= 0 ||
              maxSpeed > 5 ||
              !Number.isFinite(lookahead) ||
              lookahead <= 0 ||
              lookahead > 10
            }
            onClick={() => {
              if (encoder)
                void onNavigate({
                  path: result.result.path,
                  encoder: encoder.name,
                  max_steps: maxSteps,
                  max_speed: maxSpeed,
                  lookahead,
                  clearance: result.request.clearance,
                });
            }}
          >
            ▶ Run this path
          </button>
          <p className="hint">
            Stops at estimated goal tolerance (5 cm), collision or the step
            limit. Arrival does not mean a settled physical stop.
          </p>
        </fieldset>
      )}
    </section>
  );
}
