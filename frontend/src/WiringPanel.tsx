import { useEffect, useState } from "react";
import { post } from "./api";

type Endpoint = { component: string; pin: string };
type Net = { id: string; endpoints: Endpoint[] };
type Assignment = { endpoint: Endpoint; function: string };
type Pin = {
  id: string;
  kind: string;
  capabilities?: string[];
  accepted_voltage?: { minimum_v: number; maximum_v: number } | null;
};
type Project = {
  components: { id: string; category: string; pins: Pin[] }[];
  nets: Net[];
  assignments?: Assignment[];
};
type Diagnostic = {
  severity: string;
  code: string;
  title: string;
  message: string;
  component: string | null;
  suggestion: string;
  context?: string[];
};
const same = (a: Endpoint, b: Endpoint) =>
  a.component === b.component && a.pin === b.pin;
const functions = [
  "DIGITAL_IN",
  "DIGITAL_OUT",
  "PWM",
  "INTERRUPT",
  "I2C_SDA",
  "I2C_SCL",
];

export function WiringPanel<T extends Project>({
  project,
  visible,
  disabled,
  onApply,
  onPending,
}: {
  project: T;
  visible: boolean;
  disabled: boolean;
  onApply: (project: T, diagnostics: Diagnostic[]) => void;
  onPending: (value: boolean) => void;
}) {
  const [pending, setPending] = useState<T | null>(null),
    [base, setBase] = useState("");
  const [selected, setSelected] = useState(project.nets[0]?.id ?? "");
  const [component, setComponent] = useState(
    project.components.find((part) => part.pins.length)?.id ?? "",
  );
  const [pin, setPin] = useState("");
  const [newName, setNewName] = useState("");
  const [busy, setBusy] = useState(false),
    [error, setError] = useState("");
  const [diagnostics, setDiagnostics] = useState<Diagnostic[]>([]);
  const [undo, setUndo] = useState<{ project: T; after: string } | null>(null);
  const draft = pending ?? project;
  const net = draft.nets.find((row) => row.id === selected) ?? draft.nets[0];
  const part = draft.components.find((row) => row.id === component);
  const stale = !!pending && base !== JSON.stringify(project);
  useEffect(() => onPending(pending !== null), [pending, onPending]);
  const mutate = (change: (copy: T) => void) => {
    const copy = structuredClone(draft);
    change(copy);
    if (!pending) setBase(JSON.stringify(project));
    setPending(copy);
    setError("");
    setDiagnostics([]);
  };
  const discard = () => {
    setPending(null);
    setError("");
    setDiagnostics([]);
  };
  const apply = async () => {
    if (!pending || stale) return;
    setBusy(true);
    setError("");
    try {
      const result = await post<{ project: T; diagnostics: Diagnostic[] }>(
        "/api/hardware/validate",
        pending,
      );
      setUndo({ project, after: JSON.stringify(result.project) });
      onApply(result.project, result.diagnostics);
      setDiagnostics(result.diagnostics);
      setPending(null);
    } catch (error) {
      setError(error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  };
  const setAssignment = (endpoint: Endpoint, value: string) =>
    mutate((copy) => {
      const assignments = (copy.assignments ?? []).filter(
        (row) => !same(row.endpoint, endpoint),
      );
      if (value)
        assignments.push({ endpoint: { ...endpoint }, function: value });
      copy.assignments = assignments;
    });
  return (
    <section className="wiring-panel" hidden={!visible}>
      <aside className="wiring-net-list">
        <h2>Electrical nets</h2>
        <p className="hint">{draft.nets.length} canonical connections</p>
        {draft.nets.map((row) => (
          <button
            key={row.id}
            aria-pressed={net?.id === row.id}
            onClick={() => setSelected(row.id)}
          >
            {row.id}
            <small>{row.endpoints.length} pins</small>
          </button>
        ))}
      </aside>
      <div className="hardware-content wiring-body">
        <div className="hardware-actions">
          <button
            className="primary"
            disabled={!pending || stale || busy || disabled}
            onClick={() => void apply()}
          >
            {busy ? "Checking…" : "Apply wiring changes"}
          </button>
          <button disabled={!pending || busy} onClick={discard}>
            Discard wiring edits
          </button>
          <button
            disabled={
              !undo ||
              undo.after !== JSON.stringify(project) ||
              !!pending ||
              busy ||
              disabled
            }
            onClick={() => {
              if (undo) {
                onApply(undo.project, []);
                setUndo(null);
                setDiagnostics([]);
              }
            }}
          >
            Undo applied wiring
          </button>
        </div>
        <p className="hint">
          This is logical net membership and MCU assignment, with diagram layout
          for readability. It doesn't simulate a circuit or establish a physical
          wiring plan.
        </p>
        {pending && (
          <p className="message notice">
            {stale
              ? "The project changed after these edits began. Discard them to start from the current design."
              : "Wiring edits are pending. Apply or discard them before running or exporting the setup."}
          </p>
        )}
        {error && (
          <p className="message error" role="alert">
            {error}
          </p>
        )}
        <fieldset disabled={busy || disabled || stale}>
          <legend>Add a net</legend>
          <label>
            New net identifier
            <input
              value={newName}
              onChange={(event) => setNewName(event.target.value)}
            />
          </label>
          <button
            disabled={!newName}
            onClick={() => {
              const id = newName.trim();
              if (!id || draft.nets.some((row) => row.id === id)) {
                setError("Use a new, nonempty net identifier.");
                return;
              }
              mutate((copy) => copy.nets.push({ id, endpoints: [] }));
              setSelected(id);
              setNewName("");
            }}
          >
            Create net draft
          </button>
        </fieldset>
        {net && (
          <>
            <h2>{net.id}</h2>
            <svg
              className="wiring-map"
              viewBox={`0 0 760 ${Math.max(180, Math.ceil(net.endpoints.length / 2) * 80 + 70)}`}
              role="img"
              aria-label={`Logical connection map for ${net.id}`}
            >
              <line
                x1="380"
                x2="380"
                y1="20"
                y2={Math.max(
                  130,
                  Math.ceil(net.endpoints.length / 2) * 80 + 20,
                )}
                stroke="#b496ff"
                strokeWidth="3"
              />
              {net.endpoints.map((endpoint, index) => {
                const left = index % 2 === 0,
                  x = left ? 20 : 430,
                  y = 40 + Math.floor(index / 2) * 80;
                return (
                  <g key={`${endpoint.component}.${endpoint.pin}`}>
                    <path
                      d={`M ${left ? 330 : 430} ${y + 22} H 380`}
                      stroke="#827096"
                      fill="none"
                    />
                    <rect
                      x={x}
                      y={y}
                      width="310"
                      height="48"
                      rx="2"
                      fill="#151b25"
                      stroke="#414958"
                    />
                    <text x={x + 12} y={y + 20} fill="#dce0e8">
                      {endpoint.component}.{endpoint.pin}
                    </text>
                    <text x={x + 12} y={y + 38} fill="#969dab" fontSize="11">
                      {(draft.assignments ?? []).find((row) =>
                        same(row.endpoint, endpoint),
                      )?.function ??
                        draft.components
                          .find((row) => row.id === endpoint.component)
                          ?.pins.find((row) => row.id === endpoint.pin)?.kind}
                    </text>
                  </g>
                );
              })}
            </svg>
            <fieldset disabled={busy || disabled || stale}>
              <legend>Connected pins</legend>
              <div className="wiring-endpoints">
                {net.endpoints.map((endpoint) => {
                  const owner = draft.components.find(
                      (row) => row.id === endpoint.component,
                    ),
                    signal = owner?.pins.find((row) => row.id === endpoint.pin);
                  return (
                    <div key={`${endpoint.component}.${endpoint.pin}`}>
                      <strong>
                        {endpoint.component}.{endpoint.pin}
                      </strong>
                      <small>
                        {signal?.kind} · {signal?.capabilities?.join(", ")}
                      </small>
                      {signal?.accepted_voltage && (
                        <small>
                          Accepted voltage: {signal.accepted_voltage.minimum_v}–
                          {signal.accepted_voltage.maximum_v} V
                        </small>
                      )}
                      {owner?.category === "mcu" && (
                        <label>
                          MCU function for {endpoint.component}.{endpoint.pin}
                          <select
                            value={
                              (draft.assignments ?? []).find((row) =>
                                same(row.endpoint, endpoint),
                              )?.function ?? ""
                            }
                            onChange={(event) =>
                              setAssignment(endpoint, event.target.value)
                            }
                          >
                            <option value="">Unassigned</option>
                            {functions.map((value) => (
                              <option
                                key={value}
                                value={value}
                                disabled={
                                  !signal?.capabilities?.includes(value)
                                }
                              >
                                {value}
                              </option>
                            ))}
                          </select>
                        </label>
                      )}
                      <button
                        onClick={() =>
                          mutate((copy) => {
                            const target = copy.nets.find(
                              (row) => row.id === net.id,
                            )!;
                            target.endpoints = target.endpoints.filter(
                              (row) => !same(row, endpoint),
                            );
                          })
                        }
                      >
                        Remove {endpoint.component}.{endpoint.pin}
                      </button>
                    </div>
                  );
                })}
              </div>
              <div className="wiring-add">
                <label>
                  Component
                  <select
                    value={component}
                    onChange={(event) => {
                      setComponent(event.target.value);
                      setPin("");
                    }}
                  >
                    {draft.components
                      .filter((row) => row.pins.length)
                      .map((row) => (
                        <option key={row.id} value={row.id}>
                          {row.id}
                        </option>
                      ))}
                  </select>
                </label>
                <label>
                  Pin
                  <select
                    value={pin}
                    onChange={(event) => setPin(event.target.value)}
                  >
                    <option value="">Choose a pin…</option>
                    {part?.pins.map((row) => (
                      <option key={row.id} value={row.id}>
                        {row.id} · {row.kind}
                      </option>
                    ))}
                  </select>
                </label>
                <button
                  disabled={!pin || !component}
                  onClick={() => {
                    if (
                      net.endpoints.some((endpoint) =>
                        same(endpoint, { component, pin }),
                      )
                    ) {
                      setError("This pin already belongs to the selected net.");
                      return;
                    }
                    mutate((copy) =>
                      copy.nets
                        .find((row) => row.id === net.id)!
                        .endpoints.push({ component, pin }),
                    );
                  }}
                >
                  Add pin to net
                </button>
              </div>
              <button
                onClick={() =>
                  mutate((copy) => {
                    copy.nets = copy.nets.filter((row) => row.id !== net.id);
                  })
                }
              >
                Remove net from draft
              </button>
            </fieldset>
          </>
        )}
        {diagnostics.length > 0 && (
          <>
            <h3>Applied design diagnostics</h3>
            {diagnostics.map((item, index) => (
              <article
                className={`hardware-diagnostic ${item.severity.toLowerCase()}`}
                key={index}
              >
                <strong>
                  {item.severity} · {item.title}
                </strong>
                <p>{item.message}</p>
                <small>{item.suggestion}</small>
              </article>
            ))}
          </>
        )}
      </div>
    </section>
  );
}
