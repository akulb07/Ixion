import type { Config, Fault } from "./types";

const kinds = {
  wheel_slip: {
    title: "Wheel slip",
    label: "Fraction of ground motion lost",
    value: 0.5,
    min: 0,
    max: 1,
    step: 0.05,
    note: "Shaft encoders keep turning while ground motion is reduced.",
  },
  sensor_dropout: {
    title: "Sensor dropout",
    label: "Dropout probability",
    value: 0.5,
    min: 0,
    max: 1,
    step: 0.05,
    note: "Encoder/IMU packets or individual LiDAR rays become missing.",
  },
  encoder_scale: {
    title: "Encoder scale error",
    label: "Relative count error",
    value: 0.1,
    min: -0.999999,
    max: undefined,
    step: 0.05,
    note: "Scales cumulative counts; switching this fault can create a readout jump.",
  },
  gyro_bias: {
    title: "Gyro bias",
    label: "Bias (rad/s)",
    value: 0.1,
    min: undefined,
    max: undefined,
    step: 0.05,
    note: "Adds a constant offset to gyro measurements.",
  },
  gyro_drift: {
    title: "Gyro drift",
    label: "Drift rate (rad/s²)",
    value: 0.1,
    min: undefined,
    max: undefined,
    step: 0.05,
    note: "The offset grows from the start time and resets at the end.",
  },
  lidar_noise: {
    title: "LiDAR noise",
    label: "Range standard deviation (m)",
    value: 0.05,
    min: 0,
    max: undefined,
    step: 0.01,
    note: "Adds Gaussian noise to hit ranges; out-of-range returns become missing.",
  },
  actuator_saturation: {
    title: "Wheel speed cap",
    label: "Speed limit (rad/s)",
    value: 2,
    min: 0.000001,
    max: undefined,
    step: 0.5,
    note: "Caps physical shaft speed on the selected wheel or wheels.",
  },
  actuator_delay: {
    title: "Command delay",
    label: "Delay (simulation steps)",
    value: 0,
    min: 1,
    max: 10000,
    step: 1,
    note: "Delays both wheel commands. Overlapping delays add.",
  },
} as const;
type Kind = keyof typeof kinds;
function targets(kind: string, config: Config) {
  if (kind === "actuator_delay") return ["both"];
  if (kind === "wheel_slip" || kind === "actuator_saturation")
    return ["left", "right", "both"];
  const type =
    kind === "encoder_scale"
      ? "encoder"
      : kind.startsWith("gyro_")
        ? "imu"
        : kind === "lidar_noise"
          ? "lidar"
          : null;
  return config.sensors
    .filter((s) => !type || s.type === type)
    .map((s) => s.name);
}
function nameFor(config: Config) {
  let i = 1;
  while (config.faults.some((f) => f.name === `fault_${i}`)) i++;
  return `fault_${i}`;
}
const numeric = (v: number) => (Number.isFinite(v) ? v : "");
export function FaultEditor({
  config,
  disabled,
  edit,
  onValidate,
}: {
  config: Config;
  disabled: boolean;
  edit: (fn: (c: Config) => void) => void;
  onValidate: () => void;
}) {
  const duration =
    (config.navigation?.max_steps ??
      config.commands.reduce((n, c) => n + c.steps, 0)) * config.simulation.dt;
  const change = (index: number, fn: (f: Fault) => void) =>
    edit((c) => fn(c.faults[index]));
  return (
    <details className="fault-editor">
      <summary>
        Fault injection <span>{config.faults.length} SCHEDULED</span>
      </summary>
      <fieldset className="controller-fields" disabled={disabled}>
        <p className="hint">
          Faults apply to the next run. Intervals include the start and exclude
          the end. Leave the end blank to continue until the run stops.
        </p>
        {config.faults.map((fault, i) => {
          const definition = kinds[fault.kind as Kind],
            options = targets(fault.kind, config);
          return (
            <div className="command" key={i}>
              <label>
                Name
                <input
                  aria-label={`Fault ${i + 1} name`}
                  value={fault.name}
                  pattern="[a-z][a-z0-9_]*"
                  onChange={(e) =>
                    change(i, (f) => {
                      f.name = e.target.value;
                    })
                  }
                />
              </label>
              <label>
                Effect
                <select
                  aria-label={`Fault ${i + 1} effect`}
                  value={fault.kind}
                  onChange={(e) =>
                    change(i, (f) => {
                      const kind = e.target.value as Kind;
                      f.kind = kind;
                      f.target = targets(kind, config)[0] ?? "";
                      f.magnitude = kinds[kind].value;
                      f.delay_steps = kind === "actuator_delay" ? 5 : 0;
                    })
                  }
                >
                  {Object.entries(kinds).map(([key, value]) => (
                    <option
                      key={key}
                      value={key}
                      disabled={!targets(key, config).length}
                    >
                      {value.title}
                    </option>
                  ))}
                </select>
              </label>
              <label>
                Target
                <select
                  aria-label={`Fault ${i + 1} target`}
                  value={fault.target}
                  onChange={(e) =>
                    change(i, (f) => {
                      f.target = e.target.value;
                    })
                  }
                >
                  {!options.includes(fault.target) && (
                    <option value={fault.target}>
                      {fault.target || "Choose a target"} (unavailable)
                    </option>
                  )}
                  {options.map((t) => (
                    <option key={t}>{t}</option>
                  ))}
                </select>
              </label>
              <div className="pair">
                <label>
                  Start (s)
                  <input
                    aria-label={`Fault ${i + 1} start`}
                    type="number"
                    min="0"
                    step="0.1"
                    value={numeric(fault.start)}
                    onChange={(e) =>
                      change(i, (f) => {
                        f.start = e.target.valueAsNumber;
                      })
                    }
                  />
                </label>
                <label>
                  End (s)
                  <input
                    aria-label={`Fault ${i + 1} end`}
                    type="number"
                    min="0"
                    step="0.1"
                    placeholder="Run end"
                    value={fault.end === null ? "" : numeric(fault.end)}
                    onChange={(e) =>
                      change(i, (f) => {
                        f.end =
                          e.target.value === "" ? null : e.target.valueAsNumber;
                      })
                    }
                  />
                </label>
              </div>
              <label>
                {definition?.label ?? "Magnitude"}
                <input
                  aria-label={`Fault ${i + 1} magnitude`}
                  type="number"
                  min={definition?.min}
                  max={definition?.max}
                  step={definition?.step ?? 0.1}
                  value={numeric(
                    fault.kind === "actuator_delay"
                      ? fault.delay_steps
                      : fault.magnitude,
                  )}
                  onChange={(e) =>
                    change(i, (f) => {
                      if (f.kind === "actuator_delay")
                        f.delay_steps = e.target.valueAsNumber;
                      else f.magnitude = e.target.valueAsNumber;
                    })
                  }
                />
              </label>
              <p className="hint">
                {definition?.note}
                {fault.kind === "actuator_delay" &&
                  ` Delay: ${(fault.delay_steps * config.simulation.dt).toFixed(3)} s.`}
              </p>
              {Number.isFinite(fault.start) &&
                Number.isFinite(duration) &&
                fault.start >= duration && (
                  <p className="hint amber">
                    Starts at or after the planned run end; it cannot affect an
                    earlier step.
                  </p>
                )}
              <button
                onClick={() =>
                  edit((c) => {
                    c.faults.splice(i, 1);
                  })
                }
                aria-label={`Remove fault ${i + 1}`}
              >
                Remove fault
              </button>
            </div>
          );
        })}
        <div className="comparison-actions">
          <button
            disabled={config.faults.length >= 64}
            onClick={() =>
              edit((c) => {
                c.faults.push({
                  name: nameFor(c),
                  kind: "wheel_slip",
                  target: "left",
                  start: 0,
                  end: null,
                  magnitude: 0.5,
                  delay_steps: 0,
                });
              })
            }
          >
            Add fault
          </button>
          <button onClick={onValidate}>Validate faults</button>
        </div>
        {!!config.faults.length && (
          <button
            className="wide subtle"
            onClick={() =>
              edit((c) => {
                c.faults = [];
              })
            }
          >
            Clear faults for baseline
          </button>
        )}
        <p className="hint">
          Keep the same seed, fault names and setup to repeat a run. For a
          baseline, save this experiment first, then clear the faults and run
          again. Saved history is unchanged. Sensor faults use capture time;
          wheel faults apply on simulation ticks.
        </p>
      </fieldset>
    </details>
  );
}
