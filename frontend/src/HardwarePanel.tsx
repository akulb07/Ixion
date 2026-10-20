import { useEffect, useRef, useState } from "react";
import { post, request } from "./api";
import { AssemblyViewport } from "./AssemblyViewport";
import { EngineeringPanel } from "./EngineeringPanel";
import { WiringPanel } from "./WiringPanel";
import { ReplayPanel } from "./ReplayPanel";
import { HardwareComparison, type HardwareJob } from "./HardwareComparison";

type Parameter = {
  name: string;
  value: number;
  unit: string;
  evidence: string;
  reference: string;
};
type Component = {
  id: string;
  type: string;
  category: string;
  parameters: Parameter[];
  pins: { id: string; kind: string }[];
};
type Draft = {
  project: {
    name: string;
    assembly: {
      total_mass_kg: number;
      wheel_radius_m: number;
      track_width_m: number;
    };
    components: Component[];
    nets: { id: string; endpoints: { component: string; pin: string }[] }[];
  };
  suite: {
    name: string;
    cases: {
      name: string;
      controller: string;
      scenario: Record<string, unknown>;
    }[];
  };
};
type Diagnostic = {
  severity: string;
  code: string;
  title: string;
  message: string;
  component: string | null;
  suggestion: string;
};
type Job = HardwareJob;
const storageKey = "ixion.hardware.draft.v1";
const pretty = (value: unknown) => JSON.stringify(value, null, 2);
const message = (error: unknown) =>
  error instanceof Error ? error.message : String(error);

export function HardwarePanel({ visible }: { visible: boolean }) {
  const [draft, setDraft] = useState<Draft | null>(null);
  const [templates, setTemplates] = useState<{ id: string; title: string }[]>(
    [],
  );
  const [jobs, setJobs] = useState<Job[]>([]);
  const [selected, setSelected] = useState<string | null>(null);
  const [componentId, setComponentId] = useState("left_motor");
  const [tab, setTab] = useState<
    | "assembly"
    | "parts"
    | "wiring"
    | "sizing"
    | "experiment"
    | "source"
    | "results"
    | "compare"
  >("assembly");
  const [source, setSource] = useState("");
  const [sourceDirty, setSourceDirty] = useState(false);
  const [wiringDirty, setWiringDirty] = useState(false);
  const [diagnostics, setDiagnostics] = useState<Diagnostic[]>([]);
  const [checked, setChecked] = useState(false);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [resultView, setResultView] = useState<"replay" | "report">("replay");
  const loaded = useRef(false),
    readyToSave = useRef(false);
  const fileInput = useRef<HTMLInputElement>(null);
  const job = jobs.find((row) => row.id === selected);
  const component = draft?.project.components.find(
    (row) => row.id === componentId,
  );
  const apply = (value: Draft, checks: Diagnostic[] = []) => {
    setDraft({ project: value.project, suite: value.suite });
    setSource(pretty({ project: value.project, suite: value.suite }));
    setSourceDirty(false);
    setDiagnostics(checks);
    setChecked(checks.length > 0);
  };
  const update = (change: (copy: Draft) => void) => {
    if (!draft) return;
    const copy = structuredClone(draft);
    change(copy);
    apply(copy);
    setChecked(false);
  };
  const perform = async (action: () => Promise<void>) => {
    setBusy(true);
    setError("");
    setNotice("");
    try {
      await action();
    } catch (error) {
      setError(message(error));
    } finally {
      setBusy(false);
    }
  };
  useEffect(() => {
    if (!visible || loaded.current) return;
    loaded.current = true;
    void perform(async () => {
      setTemplates(await request("/api/hardware/templates"));
      let saved: string | null = null;
      try {
        saved = localStorage.getItem(storageKey);
      } catch {
        setNotice(
          "Browser draft storage is unavailable. Export your setup to keep it.",
        );
      }
      if (saved) {
        try {
          const result = await post<Draft & { diagnostics: Diagnostic[] }>(
            "/api/hardware/specification/validate",
            JSON.parse(saved),
          );
          apply(result, result.diagnostics);
          setNotice("Restored your hardware draft.");
        } catch {
          apply(await request<Draft>("/api/hardware/templates/feedback"));
          setNotice(
            "The saved browser draft could not be restored. Loaded the starter rover.",
          );
        }
      } else apply(await request<Draft>("/api/hardware/templates/feedback"));
      readyToSave.current = true;
    });
  }, [visible]);
  useEffect(() => {
    if (!draft || !readyToSave.current) return;
    try {
      localStorage.setItem(storageKey, pretty(draft));
    } catch {
      setNotice(
        "Browser draft storage is unavailable. Export your setup to keep it.",
      );
    }
  }, [draft]);
  useEffect(() => {
    if (!visible) return;
    const controller = new AbortController();
    const refresh = () =>
      request<{ items: Job[] }>("/api/hardware/runs?limit=100", {
        signal: controller.signal,
      })
        .then((value) => {
          if (!controller.signal.aborted) setJobs(value.items);
        })
        .catch((error) => {
          if (!controller.signal.aborted) setError(message(error));
        });
    void refresh();
    const timer = setInterval(() => void refresh(), 2000);
    return () => {
      controller.abort();
      clearInterval(timer);
    };
  }, [visible]);
  const validate = async () => {
    const result = await post<Draft & { diagnostics: Diagnostic[] }>(
      "/api/hardware/specification/validate",
      draft,
    );
    apply(result, result.diagnostics);
    setChecked(true);
    setNotice(
      "Design checks updated. Missing ratings and assumptions remain visible below.",
    );
  };
  const run = () =>
    perform(async () => {
      const result = await post<Job>("/api/hardware/runs", draft);
      setJobs((rows) => [result, ...rows]);
      setSelected(result.id);
      setTab("results");
    });
  const exportSetup = () => {
    const url = URL.createObjectURL(
      new Blob([pretty(draft)], { type: "application/json" }),
    );
    const link = document.createElement("a");
    link.href = url;
    link.download = "ixion-hardware.json";
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  return (
    <main className="hardware-workspace" hidden={!visible}>
      <aside className="panel hardware-explorer">
        <div className="panel-heading">ROBOT HARDWARE</div>
        <label className="hardware-template">
          Starter experiment
          <select
            disabled={busy || sourceDirty || wiringDirty}
            defaultValue=""
            onChange={(e) => {
              const id = e.target.value;
              if (!id) return;
              void perform(async () => {
                apply(await request<Draft>(`/api/hardware/templates/${id}`));
                setTab("parts");
                setNotice(
                  "Loaded starter inputs. Saved runs remain in history.",
                );
              });
            }}
          >
            <option value="" disabled>
              Choose a starter…
            </option>
            {templates.map((row) => (
              <option key={row.id} value={row.id}>
                {row.title}
              </option>
            ))}
          </select>
        </label>
        {draft && (
          <>
            <h2>{draft.project.name}</h2>
            <p className="hint">
              {draft.project.components.length} parts ·{" "}
              {draft.project.nets.length} nets
            </p>
            <div className="hardware-tree">
              {draft.project.components.map((part) => (
                <button
                  key={part.id}
                  aria-pressed={componentId === part.id}
                  onClick={() => {
                    setComponentId(part.id);
                    setTab("parts");
                  }}
                >
                  <span>{part.id}</span>
                  <small>{part.category.replaceAll("_", " ")}</small>
                </button>
              ))}
            </div>
          </>
        )}
        <div className="panel-heading">SAVED HARDWARE RUNS</div>
        <div className="hardware-history">
          {jobs.length === 0 && (
            <p className="hint">Your completed experiments will appear here.</p>
          )}
          {jobs.map((row) => (
            <button
              key={row.id}
              aria-pressed={selected === row.id}
              onClick={() => {
                setSelected(row.id);
                setTab("results");
              }}
            >
              <strong>{row.name}</strong>
              <small>
                {row.status} {row.outcome && `· ${row.outcome}`}
              </small>
              <small>
                {new Date(row.created_at).toLocaleString()} · {row.id.slice(-8)}
              </small>
            </button>
          ))}
        </div>
      </aside>
      <section className="panel hardware-editor">
        <div
          className="hardware-tabs"
          role="tablist"
          aria-label="Hardware views"
        >
          {(
            [
              ["assembly", "Assembly"],
              ["parts", "Parts"],
              ["wiring", "Wiring"],
              ["sizing", "Sizing"],
              ["experiment", "Experiment"],
              ["source", "Setup JSON"],
              ["results", "Results"],
              ["compare", "Compare"],
            ] as const
          ).map(([id, title]) => (
            <button
              role="tab"
              aria-selected={tab === id}
              key={id}
              onClick={() => setTab(id)}
            >
              {title}
              {id === "source" && sourceDirty ? " •" : ""}
            </button>
          ))}
        </div>
        <div className="hardware-actions">
          <button
            disabled={busy || !draft || sourceDirty || wiringDirty}
            onClick={() => void perform(validate)}
          >
            Check design
          </button>
          <button
            className="primary"
            disabled={busy || !draft || sourceDirty || wiringDirty}
            onClick={() => void run()}
          >
            {busy ? "Working…" : "▶ Run hardware suite"}
          </button>
          <button
            disabled={!draft || sourceDirty || wiringDirty}
            onClick={exportSetup}
          >
            Export setup
          </button>
          <button
            disabled={busy || sourceDirty || wiringDirty}
            onClick={() => fileInput.current?.click()}
          >
            Import setup
          </button>
          <input
            ref={fileInput}
            hidden
            type="file"
            accept=".json,application/json"
            onChange={(e) => {
              const file = e.target.files?.[0];
              e.target.value = "";
              if (file)
                void perform(async () => {
                  if (file.size > 1_000_000)
                    throw new Error("Setup file exceeds 1 MB.");
                  const result = await post<
                    Draft & { diagnostics: Diagnostic[] }
                  >(
                    "/api/hardware/specification/validate",
                    JSON.parse(await file.text()),
                  );
                  apply(result, result.diagnostics);
                  setTab("parts");
                });
            }}
          />
        </div>
        {error && (
          <p className="message error" role="alert">
            {error}
          </p>
        )}
        {notice && (
          <p className="message notice" role="status">
            {notice}
          </p>
        )}
        {!draft && (
          <p className="hardware-content">Loading the reference rover…</p>
        )}
        {draft && tab === "assembly" && (
          <div className="hardware-assembly">
            <AssemblyViewport
              project={draft.project}
              selected={componentId}
              onSelect={setComponentId}
            />
            <aside className="assembly-inspector">
              <h2>{component?.id ?? "Select a part"}</h2>
              <p className="hint">{component?.type}</p>
              {component?.parameters.map((parameter) => (
                <div key={parameter.name}>
                  <small>{parameter.name.replaceAll("_", " ")}</small>
                  <strong>
                    {parameter.value} {parameter.unit}
                  </strong>
                  <small>{parameter.evidence}</small>
                </div>
              ))}
              <button onClick={() => setTab("parts")}>
                Edit selected part
              </button>
            </aside>
          </div>
        )}
        {draft && (
          <EngineeringPanel
            project={draft.project}
            visible={tab === "sizing"}
            disabled={busy || sourceDirty || wiringDirty}
          />
        )}
        {draft && (
          <WiringPanel
            project={draft.project}
            visible={tab === "wiring"}
            disabled={sourceDirty || busy}
            onPending={setWiringDirty}
            onApply={(project, checks) => {
              apply({ project, suite: draft.suite }, checks);
              setNotice(
                "Wiring applied. Review the design diagnostics before running.",
              );
            }}
          />
        )}
        <HardwareComparison
          jobs={jobs}
          visible={tab === "compare"}
          onOpenRun={(id) => {
            setSelected(id);
            setResultView("replay");
            setTab("results");
          }}
        />
        {wiringDirty && tab !== "wiring" && (
          <p className="message notice" role="status">
            Apply or discard pending changes in Wiring before editing or running
            this setup.
          </p>
        )}
        {draft && tab === "parts" && (
          <div className="hardware-content">
            <div className="hardware-summary">
              <label>
                Project name
                <input
                  disabled={busy || sourceDirty || wiringDirty}
                  value={draft.project.name}
                  onChange={(e) =>
                    update((copy) => {
                      copy.project.name = e.target.value;
                    })
                  }
                />
              </label>
              {(
                [
                  ["total_mass_kg", "Total mass · kg"],
                  ["wheel_radius_m", "Wheel radius · m"],
                  ["track_width_m", "Track width · m"],
                ] as const
              ).map(([key, label]) => (
                <label key={key}>
                  {label}
                  <input
                    disabled={busy || sourceDirty || wiringDirty}
                    type="number"
                    step="any"
                    value={draft.project.assembly[key]}
                    onChange={(e) => {
                      const value = e.target.valueAsNumber;
                      if (Number.isFinite(value))
                        update((copy) => {
                          copy.project.assembly[key] = value;
                        });
                    }}
                  />
                </label>
              ))}
            </div>
            {component && (
              <>
                <h2>{component.id}</h2>
                <p className="hint">
                  {component.type} · {component.pins.length} pins
                </p>
                <div className="hardware-parameters">
                  {component.parameters.map((parameter) => (
                    <label key={parameter.name}>
                      <span>
                        {parameter.name.replaceAll("_", " ")} · {parameter.unit}
                      </span>
                      <input
                        disabled={busy || sourceDirty || wiringDirty}
                        type="number"
                        step="any"
                        value={parameter.value}
                        onChange={(e) => {
                          const value = e.target.valueAsNumber;
                          if (Number.isFinite(value))
                            update((copy) => {
                              const parameterCopy = copy.project.components
                                .find((part) => part.id === component.id)!
                                .parameters.find(
                                  (p) => p.name === parameter.name,
                                )!;
                              parameterCopy.value = value;
                              parameterCopy.evidence = "assumed";
                              parameterCopy.reference = "Workspace edit";
                            });
                        }}
                      />
                      <small>
                        {parameter.evidence} · {parameter.reference}
                      </small>
                    </label>
                  ))}
                </div>
                {component.parameters.length === 0 && (
                  <p className="hint">
                    No numerical model parameters declared for this part.
                  </p>
                )}
              </>
            )}
            <p className="hint">
              Estimates use the declared part data. This fixture supports
              straight travel with host Python controllers. Physical validation
              and broader robot mechanics remain ahead.
            </p>
            <h2>Design checks {checked ? "" : "· update after editing"}</h2>
            {diagnostics.map((item, index) => (
              <article
                className={`hardware-diagnostic ${item.severity.toLowerCase()}`}
                key={index}
              >
                <strong>
                  {item.severity} · {item.title}
                </strong>
                <p>{item.message}</p>
                <small>
                  {item.component} {item.suggestion}
                </small>
              </article>
            ))}
          </div>
        )}
        {draft && tab === "experiment" && (
          <div className="hardware-content">
            <h2>{draft.suite.name}</h2>
            <p className="hint">
              Every case starts with a fresh robot, controller and sensor state.
              Values use SI units. Additional settings and acceptance limits are
              in Setup JSON.
            </p>
            {draft.suite.cases.map((test, index) => (
              <fieldset
                key={test.name}
                disabled={busy || sourceDirty || wiringDirty}
              >
                <legend>
                  {test.name} · {test.controller}
                </legend>
                <div className="hardware-parameters">
                  {(
                    [
                      ["wall_distance_m", "Wall distance · m"],
                      ["brake_threshold_m", "Brake distance · m"],
                      ["duration_s", "Duration · s"],
                      [
                        "control_every_steps",
                        "Control interval · physics steps",
                      ],
                    ] as const
                  ).map(([key, label]) => (
                    <label key={key}>
                      {label}
                      <input
                        type="number"
                        step="any"
                        value={Number(test.scenario[key])}
                        onChange={(e) => {
                          const value = e.target.valueAsNumber;
                          if (Number.isFinite(value))
                            update((copy) => {
                              copy.suite.cases[index].scenario[key] = value;
                            });
                        }}
                      />
                    </label>
                  ))}
                </div>
              </fieldset>
            ))}
          </div>
        )}
        {tab === "source" && (
          <div className="hardware-source">
            <p className="hint">
              Complete project and suite settings. Apply to validate the inputs
              before running.
            </p>
            <textarea
              aria-label="Hardware setup JSON"
              readOnly={wiringDirty || busy}
              spellCheck={false}
              value={source}
              onChange={(e) => {
                setSource(e.target.value);
                setSourceDirty(true);
              }}
            />
            <button
              disabled={busy || !sourceDirty}
              onClick={() =>
                void perform(async () => {
                  const result = await post<
                    Draft & { diagnostics: Diagnostic[] }
                  >("/api/hardware/specification/validate", JSON.parse(source));
                  apply(result, result.diagnostics);
                  setNotice("Setup applied.");
                })
              }
            >
              Apply setup
            </button>
            <button
              disabled={busy || !sourceDirty}
              onClick={() => {
                setSource(pretty(draft));
                setSourceDirty(false);
                setError("");
              }}
            >
              Discard JSON edits
            </button>
          </div>
        )}
        {tab === "results" && (
          <div className="hardware-results">
            {job ? (
              <>
                <div className="hardware-result-heading">
                  <strong>
                    {job.project_name} / {job.name}
                  </strong>
                  <span>
                    {job.status} {job.outcome && `· ${job.outcome}`}
                  </span>
                  {job.summary && (
                    <span>
                      {job.summary.passed}/{job.summary.total} cases passed ·{" "}
                      {job.summary.failed} failed · {job.summary.error} model
                      errors
                    </span>
                  )}
                  <button
                    disabled={busy || sourceDirty || wiringDirty}
                    onClick={() =>
                      void perform(async () => {
                        const value = await request<Draft>(
                          `/api/hardware/runs/${job.id}/specification`,
                        );
                        apply(value);
                        setTab("parts");
                        setNotice("Loaded the inputs from this saved run.");
                      })
                    }
                  >
                    Load inputs
                  </button>
                  {job.status === "completed" && (
                    <>
                      <a
                        href={`/api/hardware/runs/${job.id}/report?format=json`}
                      >
                        JSON ↓
                      </a>
                      <a
                        href={`/api/hardware/runs/${job.id}/report?format=xml`}
                      >
                        JUnit ↓
                      </a>
                    </>
                  )}
                </div>
                {job.error && <p className="message error">{job.error}</p>}
                {job.status === "completed" ? (
                  <>
                    <div className="hardware-actions" aria-label="Result views">
                      <button
                        aria-pressed={resultView === "replay"}
                        onClick={() => setResultView("replay")}
                      >
                        3D replay + telemetry
                      </button>
                      <button
                        aria-pressed={resultView === "report"}
                        onClick={() => setResultView("report")}
                      >
                        Full evidence report
                      </button>
                    </div>
                    {resultView === "replay" ? (
                      <ReplayPanel jobId={job.id} />
                    ) : (
                      <iframe
                        title={`Hardware results: ${job.name}`}
                        sandbox="allow-scripts"
                        src={`/api/hardware/runs/${job.id}/report?format=html`}
                      />
                    )}
                  </>
                ) : (
                  <p className="hardware-content">
                    {job.status === "failed"
                      ? "This run could not complete. Its original inputs are available above."
                      : "Running the declared cases. The saved report will appear when the suite completes."}
                  </p>
                )}
              </>
            ) : (
              <p className="hardware-content">
                Run a hardware suite or select an experiment from history.
              </p>
            )}
          </div>
        )}
      </section>
    </main>
  );
}
