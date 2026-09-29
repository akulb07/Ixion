import type { Config, PIDConfig } from "./types";

const defaults = (): PIDConfig => ({
  kp: 1,
  ki: 0,
  kd: 0,
  output_min: -20,
  output_max: 20,
  derivative_time_constant: 0.02,
  integral_limit: 20,
});
export function ControllerSetup({
  config,
  disabled,
  edit,
}: {
  config: Config;
  disabled: boolean;
  edit: (update: (c: Config) => void) => void;
}) {
  const encoders = config.sensors.filter((s) => s.type === "encoder"),
    controller = config.wheel_controller;
  return (
    <details className="controller-setup">
      <summary>
        Wheel feedback <span>{controller ? "PID" : "OPEN LOOP"}</span>
      </summary>
      <fieldset className="controller-fields" disabled={disabled}>
        <label>
          <input
            type="checkbox"
            checked={!!controller}
            disabled={!encoders.length}
            onChange={(e) =>
              edit((c) => {
                c.wheel_controller = e.target.checked
                  ? {
                      encoder: encoders[0].name,
                      feedforward: 1,
                      left: defaults(),
                      right: defaults(),
                    }
                  : null;
              })
            }
          />{" "}
          Enable encoder PID
        </label>
        {!encoders.length && (
          <p className="hint">Add an encoder before enabling feedback.</p>
        )}
        {controller && (
          <>
            <label>
              Feedback encoder
              <select
                value={controller.encoder}
                onChange={(e) =>
                  edit((c) => {
                    c.wheel_controller!.encoder = e.target.value;
                  })
                }
              >
                {encoders.map((s) => (
                  <option key={s.name}>{s.name}</option>
                ))}
              </select>
            </label>
            <label>
              Feedforward gain
              <input
                type="number"
                min="0"
                step="0.1"
                value={
                  Number.isFinite(controller.feedforward)
                    ? controller.feedforward
                    : ""
                }
                onChange={(e) =>
                  edit((c) => {
                    c.wheel_controller!.feedforward = e.target.valueAsNumber;
                  })
                }
              />
            </label>
            {(["left", "right"] as const).map((wheel) => (
              <div className="command" key={wheel}>
                <p className="command-caption">{wheel.toUpperCase()} WHEEL</p>
                {(
                  [
                    ["kp", "Kp"],
                    ["ki", "Ki"],
                    ["kd", "Kd"],
                    ["output_min", "Output minimum (rad/s)"],
                    ["output_max", "Output maximum (rad/s)"],
                    ["derivative_time_constant", "Derivative filter (s)"],
                    ["integral_limit", "Integral limit (rad/s)"],
                  ] as const
                ).map(([key, title]) => (
                  <label key={key}>
                    {title}
                    <input
                      aria-label={`${wheel} PID ${title}`}
                      type="number"
                      step="0.1"
                      min={
                        key === "output_min" || key === "output_max"
                          ? undefined
                          : 0
                      }
                      value={
                        Number.isFinite(controller[wheel][key])
                          ? controller[wheel][key]
                          : ""
                      }
                      onChange={(e) =>
                        edit((c) => {
                          c.wheel_controller![wheel][key] =
                            e.target.valueAsNumber;
                        })
                      }
                    />
                  </label>
                ))}
              </div>
            ))}
            <p className="hint">
              Edits apply to the next run. Start with the PID feedback
              laboratory to include motor lag. Output is a requested wheel
              speed; actuator limits apply afterward.
            </p>
          </>
        )}
      </fieldset>
    </details>
  );
}
