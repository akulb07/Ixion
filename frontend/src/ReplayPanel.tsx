import { useEffect, useMemo, useRef, useState } from "react";
import { request } from "./api";
import { AssemblyViewport, type AssemblyProject } from "./AssemblyViewport";
import { deliveredEvents, sampleIndexAt } from "./replay.mjs";

type Sample = {
  time_s: number;
  distance_m: number;
  speed_m_s: number;
  front_clearance_m: number;
  pack_current_a: number;
  pack_voltage_v: number;
  soc: number;
  driver_pwm_duty: [number, number];
  driver_modes: [string, string] | null;
  encoder_counts: { left: number; right: number };
  decoded_encoder_counts: { left: number; right: number };
};
type Event = { time_s: number; kind: string; title: string; detail: unknown };
type Run = {
  status: string;
  grade_rad: number;
  brake_reason: string | null;
  samples: Sample[];
  events: Event[];
  configuration: {
    scenario: { wall_distance_m: number; front_offset_m: number };
  };
};
type Evidence = {
  job_id: string;
  project: AssemblyProject & {
    name: string;
    assembly: { total_mass_kg: number };
  };
  evidence: {
    source_sha256: string;
    cases: {
      name: string;
      status: string;
      error: string | null;
      checks: {
        name: string;
        passed: boolean;
        actual: unknown;
        expected: unknown;
      }[];
      run: Run | null;
    }[];
  };
};
export const quantity = (n: number | null | undefined, unit = "") =>
  n == null || !Number.isFinite(n)
    ? "Unavailable"
    : `${Number(n.toPrecision(6))}${unit ? ` ${unit}` : ""}`;

function Trace({
  run,
  field,
  unit,
  time,
  onTime,
}: {
  run: Run;
  field: "speed_m_s" | "pack_voltage_v" | "pack_current_a";
  unit: string;
  time: number;
  onTime: (value: number) => void;
}) {
  const chart = useMemo(() => {
    const points = run.samples;
    let min = 0,
      max = 0;
    for (const sample of points) {
      min = Math.min(min, sample[field]);
      max = Math.max(max, sample[field]);
    }
    if (max === min) max = min + 1;
    const start = points[0].time_s,
      duration = points.at(-1)!.time_s - start;
    const x = (t: number) => 48 + (duration ? (t - start) / duration : 0) * 584;
    const scale = Math.max(Math.abs(min), Math.abs(max), Number.MIN_VALUE);
    const y = (v: number) =>
      104 - ((v / scale - min / scale) / (max / scale - min / scale)) * 88;
    return {
      min,
      max,
      x,
      path: points
        .map(
          (s, i) =>
            `${i ? "L" : "M"}${x(s.time_s).toFixed(3)},${y(s[field]).toFixed(3)}`,
        )
        .join(" "),
    };
  }, [run, field]);
  return (
    <div className="replay-trace">
      <strong>
        {
          {
            speed_m_s: "Speed",
            pack_voltage_v: "Pack voltage",
            pack_current_a: "Pack current",
          }[field]
        }{" "}
        · {unit}
      </strong>
      <svg
        viewBox="0 0 650 126"
        role="img"
        aria-label={`${field.replaceAll("_", " ")} recorded trace, click to move replay cursor`}
        onClick={(e) => {
          const bounds = e.currentTarget.getBoundingClientRect();
          const fraction = Math.max(
            0,
            Math.min(
              1,
              (((e.clientX - bounds.left) / bounds.width) * 650 - 48) / 584,
            ),
          );
          onTime(
            run.samples[0].time_s +
              fraction * (run.samples.at(-1)!.time_s - run.samples[0].time_s),
          );
        }}
      >
        <line x1="48" y1="104" x2="632" y2="104" className="replay-axis" />
        <text x="2" y="20">
          {quantity(chart.max)}
        </text>
        <text x="2" y="104">
          {quantity(chart.min)}
        </text>
        <path d={chart.path} className="replay-line" />
        <line
          x1={chart.x(time)}
          x2={chart.x(time)}
          y1="10"
          y2="108"
          className="replay-cursor"
        />
        <text x="48" y="123">
          {quantity(run.samples[0].time_s, "s")}
        </text>
        <text x="632" y="123" textAnchor="end">
          {quantity(run.samples.at(-1)!.time_s, "s")}
        </text>
      </svg>
    </div>
  );
}

export function ReplayPanel({ jobId }: { jobId: string }) {
  const [data, setData] = useState<Evidence | null>(null);
  const [error, setError] = useState("");
  const [caseIndex, setCaseIndex] = useState(0);
  const [time, setTime] = useState(0),
    [playing, setPlaying] = useState(false);
  const [rate, setRate] = useState(1),
    [part, setPart] = useState("");
  const cursor = useRef(time);
  cursor.current = time;
  const row = data?.evidence.cases[caseIndex],
    run = row?.run;
  const samples = run?.samples ?? [];
  const index = Math.max(0, sampleIndexAt(samples, time));
  const sample = samples[index];
  const motion = useMemo(
    () =>
      sample && { distance_m: sample.distance_m, grade_rad: run!.grade_rad },
    [sample, run],
  );
  useEffect(() => {
    const controller = new AbortController();
    setData(null);
    setError("");
    setPlaying(false);
    setCaseIndex(0);
    setTime(0);
    request<Evidence>(`/api/hardware/runs/${jobId}/view`, {
      signal: controller.signal,
    })
      .then((value) => {
        if (!controller.signal.aborted) {
          setData(value);
          setTime(value.evidence.cases[0]?.run?.samples[0]?.time_s ?? 0);
          setPart(value.project.components[0]?.id ?? "");
        }
      })
      .catch((error) => {
        if (!controller.signal.aborted) setError(error.message);
      });
    return () => controller.abort();
  }, [jobId]);
  useEffect(() => {
    if (!playing || !run) return;
    let frame = 0;
    const origin = cursor.current,
      started = performance.now(),
      end = run.samples.at(-1)!.time_s;
    const advance = (now: number) => {
      const next = Math.min(end, origin + ((now - started) / 1000) * rate);
      setTime(next);
      if (next >= end) setPlaying(false);
      else frame = requestAnimationFrame(advance);
    };
    frame = requestAnimationFrame(advance);
    return () => cancelAnimationFrame(frame);
  }, [playing, run, rate]);
  const voltageReading = useMemo(() => {
    if (!run) return null;
    for (let i = sampleIndexAt(run.events, time); i >= 0; i--)
      if (run.events[i].kind === "voltage")
        return run.events[i].detail as {
          voltage_v: number;
          time_s: number;
          sample_time_s: number;
          over_range: boolean;
        };
    return null;
  }, [run, time]);
  const seek = (value: number) => {
    setPlaying(false);
    setTime(value);
  };
  if (error)
    return (
      <p className="message error" role="alert">
        Replay unavailable: {error}
      </p>
    );
  if (!data || data.job_id !== jobId)
    return (
      <p className="hardware-content" role="status">
        Loading verified recorded evidence…
      </p>
    );
  const events: Event[] = run ? deliveredEvents(run.events, time) : [];
  return (
    <section className="hardware-replay">
      <div className="replay-heading">
        <label>
          Recorded case
          <select
            value={caseIndex}
            onChange={(e) => {
              const next = Number(e.target.value);
              setCaseIndex(next);
              setPlaying(false);
              setTime(data.evidence.cases[next].run?.samples[0]?.time_s ?? 0);
            }}
          >
            {data.evidence.cases.map((test, i) => (
              <option value={i} key={test.name}>
                {test.name} · {test.status}
              </option>
            ))}
          </select>
        </label>
        <p>
          Saved robot: <strong>{data.project.name}</strong> ·{" "}
          {data.project.assembly.total_mass_kg} kg · {row?.status} ·{" "}
          {run?.status ?? "No motion evidence"}
        </p>
      </div>
      {!run || !sample ? (
        <p className="message error" role="status">
          {row?.error ?? "No samples were recorded for this case."}
        </p>
      ) : (
        <>
          <div className="replay-controls">
            <button
              onClick={() => {
                if (!playing && time >= samples.at(-1)!.time_s)
                  setTime(samples[0].time_s);
                setPlaying((value) => !value);
              }}
            >
              {playing ? "Pause replay" : "Play replay"}
            </button>
            <button
              disabled={index === 0}
              onClick={() => seek(samples[Math.max(0, index - 1)].time_s)}
            >
              Previous sample
            </button>
            <button
              disabled={index >= samples.length - 1}
              onClick={() =>
                seek(samples[Math.min(samples.length - 1, index + 1)].time_s)
              }
            >
              Next sample
            </button>
            <label>
              Replay speed
              <select
                value={rate}
                onChange={(e) => setRate(Number(e.target.value))}
              >
                {[0.25, 1, 2, 5].map((value) => (
                  <option value={value} key={value}>
                    {value}×
                  </option>
                ))}
              </select>
            </label>
            <output aria-live={playing ? "off" : "polite"}>
              Cursor {quantity(time, "s")} · sample {index + 1}/{samples.length}{" "}
              at {quantity(sample.time_s, "s")}
            </output>
            <input
              aria-label="Recorded sample cursor"
              type="range"
              min="0"
              max={samples.length - 1}
              step="1"
              value={index}
              onChange={(e) => seek(samples[Number(e.target.value)].time_s)}
            />
          </div>
          <div className="replay-body">
            <AssemblyViewport
              project={data.project}
              selected={part}
              onSelect={setPart}
              motion={motion}
            />
            <aside className="replay-telemetry">
              <h2>Recorded telemetry</h2>
              <dl>
                {(
                  [
                    ["Travel", sample.distance_m, "m"],
                    ["Speed", sample.speed_m_s, "m/s"],
                    ["Front clearance", sample.front_clearance_m, "m"],
                    ["Pack voltage", sample.pack_voltage_v, "V"],
                    ["Pack current", sample.pack_current_a, "A"],
                    ["Charge state", sample.soc * 100, "%"],
                    ["Left PWM", sample.driver_pwm_duty[0] * 100, "%"],
                    ["Right PWM", sample.driver_pwm_duty[1] * 100, "%"],
                  ] as const
                ).map(([title, value, unit]) => (
                  <div key={title}>
                    <dt>{title}</dt>
                    <dd>{quantity(value, unit)}</dd>
                  </div>
                ))}
              </dl>
              <p>
                Observed counters L/R:{" "}
                <strong>
                  {sample.encoder_counts.left} / {sample.encoder_counts.right}
                </strong>
              </p>
              <p>
                Decoded edge counts L/R:{" "}
                <strong>
                  {sample.decoded_encoder_counts.left} /{" "}
                  {sample.decoded_encoder_counts.right}
                </strong>
              </p>
              <p>
                Driver modes L/R:{" "}
                <strong>
                  {sample.driver_modes?.join(" / ") ?? "Not recorded"}
                </strong>
              </p>
              <p className="hint">
                PWM is the recorded output duty; interpret it with driver mode.
                Brake duty is not forward throttle.
              </p>
              <p>
                Delivered voltage reading:{" "}
                <strong>
                  {voltageReading
                    ? quantity(voltageReading.voltage_v, "V")
                    : "None at this cursor"}
                </strong>
              </p>
              {voltageReading && (
                <p className="hint">
                  Sampled at {quantity(voltageReading.sample_time_s, "s")};
                  delivered at {quantity(voltageReading.time_s, "s")}
                  {voltageReading.over_range ? " · OVER RANGE" : ""}.
                </p>
              )}
              <p className="hint">
                Motion and telemetry hold the last recorded sample at or before
                the cursor. Delivered events use their own timestamps.
              </p>
            </aside>
          </div>
          <div className="replay-charts">
            <Trace
              run={run}
              field="speed_m_s"
              unit="m/s"
              time={time}
              onTime={seek}
            />
            <Trace
              run={run}
              field="pack_voltage_v"
              unit="V"
              time={time}
              onTime={seek}
            />
            <Trace
              run={run}
              field="pack_current_a"
              unit="A"
              time={time}
              onTime={seek}
            />
          </div>
          <div className="replay-events">
            <h2>Events delivered by the cursor</h2>
            <label>
              Jump to recorded event
              <select
                value=""
                onChange={(e) => {
                  if (e.target.value !== "")
                    seek(run.events[Number(e.target.value)].time_s);
                }}
              >
                <option value="">Choose event…</option>
                {run.events.map((event, i) => (
                  <option value={i} key={i}>
                    {quantity(event.time_s, "s")} · {event.title}
                  </option>
                ))}
              </select>
            </label>
            {!events.length && <p className="hint">No events delivered yet.</p>}
            {[...events].reverse().map((event, i) => (
              <details key={`${event.time_s}-${i}`}>
                <summary>
                  {quantity(event.time_s, "s")} · {event.title}
                </summary>
                <pre>{JSON.stringify(event.detail, null, 2)}</pre>
              </details>
            ))}
          </div>
        </>
      )}
      <details className="replay-acceptance">
        <summary>Case acceptance evidence</summary>
        {row?.checks.map((check, i) => (
          <p key={i}>
            <strong>
              {check.passed ? "PASS" : "FAIL"} · {check.name}
            </strong>{" "}
            · actual {JSON.stringify(check.actual)} · expected{" "}
            {JSON.stringify(check.expected)}
          </p>
        ))}
      </details>
      <p className="hint replay-source">
        Report SHA-256: {data.evidence.source_sha256}. Replay reads saved
        evidence and doesn't rerun physics. These are model estimates, with
        illustrative 3D shapes and no additional contact dynamics.
      </p>
    </section>
  );
}
