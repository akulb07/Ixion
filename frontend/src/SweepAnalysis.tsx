import { useEffect, useState } from "react";
import { request } from "./api";
import type { Batch } from "./types";

type Paired = {
  pairs: {
    seed: number;
    baseline_value: number;
    challenger_value: number;
    difference: number;
  }[];
  exclusions: { seed: number; reasons: string[] }[];
  matched_pairs: number;
  mean_difference: number | null;
};
const format = (value: number | null) =>
  value === null
    ? "—"
    : value.toLocaleString("en", { maximumSignificantDigits: 5 });

function MeasurementPlot({ batch, metric }: { batch: Batch; metric: string }) {
  const groups = [...new Set(batch.trials.map((t) => t.group))];
  const measured = batch.trials.filter((t) =>
    Number.isFinite(t.metrics[metric]),
  );
  if (!measured.length)
    return <p className="hint">No measurements available for this metric.</p>;
  // Normalizing first keeps the axis finite even for very large or tiny values.
  const magnitude =
    Math.max(...measured.map((t) => Math.abs(t.metrics[metric]))) || 1;
  const values = measured.map((t) => t.metrics[metric] / magnitude);
  const lower = Math.min(0, ...values),
    upper = Math.max(0, ...values);
  const span = upper - lower || 1;
  const x = (value: number) => 125 + ((value / magnitude - lower) / span) * 465;
  const height = 58 + groups.length * 38;
  return (
    <figure className="sweep-plot">
      <figcaption>{metric.replaceAll("_", " ")} · individual trials</figcaption>
      <svg
        viewBox={`0 0 660 ${height}`}
        role="img"
        aria-label={`Individual trial measurements for ${metric}; group order matches the summary table`}
      >
        <line
          x1={x(0)}
          x2={x(0)}
          y1={20}
          y2={height - 30}
          className="plot-axis"
        />
        {[0, 0.5, 1].map((fraction) => (
          <text
            key={fraction}
            x={125 + fraction * 465}
            y={height - 8}
            textAnchor="middle"
          >
            {format((lower + fraction * span) * magnitude)}
          </text>
        ))}
        {groups.map((group, index) => (
          <g key={group}>
            <text x={8} y={35 + index * 38}>
              {group}
            </text>
            <line
              x1={125}
              x2={590}
              y1={31 + index * 38}
              y2={31 + index * 38}
              className="plot-guide"
            />
            {measured
              .filter((t) => t.group === group)
              .map((trial, i, trials) => (
                <circle
                  key={trial.index}
                  cx={x(trial.metrics[metric])}
                  cy={31 + index * 38 + (i - (trials.length - 1) / 2) * 3}
                  r={4}
                  className={
                    trial.status === "completed"
                      ? "plot-completed"
                      : "plot-incomplete"
                  }
                >
                  <title>{`Seed ${trial.seed}: ${format(trial.metrics[metric])} (${trial.status})`}</title>
                </circle>
              ))}
          </g>
        ))}
      </svg>
      <p className="hint">
        Filled dots: completed. Hollow dots: other outcomes with measurements.
        Dots are offset vertically to show repeated values; missing measurements
        are omitted. Exact values are available in the report.
      </p>
    </figure>
  );
}

export function SweepAnalysis({
  batch,
  metric,
  visible,
}: {
  batch: Batch;
  metric: string;
  visible: boolean;
}) {
  const groups = [...new Set(batch.trials.map((t) => t.group))];
  const [base, setBase] = useState("");
  const [other, setOther] = useState("");
  const baseline = groups.includes(base) ? base : groups[0];
  const challenger =
    groups.includes(other) && other !== baseline
      ? other
      : groups.find((g) => g !== baseline);
  const [result, setResult] = useState<{ url: string; data: Paired } | null>(
    null,
  );
  const [error, setError] = useState("");
  const url =
    challenger && metric
      ? `/api/experiments/${batch.id}/paired?${new URLSearchParams({ baseline, challenger, metric })}`
      : "";
  useEffect(() => {
    if (!visible || !url) return;
    const controller = new AbortController();
    setError("");
    setResult(null);
    void request<Paired>(url, { signal: controller.signal })
      .then((data) => {
        if (!controller.signal.aborted) setResult({ url, data });
      })
      .catch((e: unknown) => {
        if (!controller.signal.aborted)
          setError(e instanceof Error ? e.message : String(e));
      });
    return () => controller.abort();
  }, [url, visible, batch.finished_trials, batch.status]);
  const paired = result?.url === url ? result.data : null;
  return (
    <section aria-label="Sweep analysis">
      <MeasurementPlot batch={batch} metric={metric} />
      <h2>Paired seed comparison</h2>
      {groups.length < 2 ? (
        <p className="hint">
          Add a parameter axis with at least two values to compare groups.
        </p>
      ) : (
        <>
          <p className="hint">
            Challenger minus baseline, matched by seed. Both trials must
            complete normally and contain the selected metric. A positive
            difference means a larger value, not necessarily a better result.
            Matching seeds does not guarantee identical noise when
            configurations change sensor timing.
          </p>
          <div className="paired-selectors">
            <label>
              Baseline group
              <select
                value={baseline}
                onChange={(e) => setBase(e.target.value)}
              >
                {groups.map((g) => (
                  <option key={g} value={g}>
                    {g} ·{" "}
                    {JSON.stringify(
                      batch.trials.find((t) => t.group === g)?.parameters,
                    )}
                  </option>
                ))}
              </select>
            </label>
            <label>
              Challenger group
              <select
                value={challenger}
                onChange={(e) => setOther(e.target.value)}
              >
                {groups
                  .filter((g) => g !== baseline)
                  .map((g) => (
                    <option key={g} value={g}>
                      {g} ·{" "}
                      {JSON.stringify(
                        batch.trials.find((t) => t.group === g)?.parameters,
                      )}
                    </option>
                  ))}
              </select>
            </label>
          </div>
          {error && (
            <p role="alert" className="hint amber">
              {error}
            </p>
          )}
          {!metric && <p className="hint">No measured metrics yet.</p>}
          {url && !paired && !error && (
            <p role="status">Loading paired measurements…</p>
          )}
          {paired && (
            <>
              <p role="status">
                {paired.matched_pairs} matched pairs · mean difference{" "}
                {format(paired.mean_difference)} · {paired.exclusions.length}{" "}
                excluded seeds
              </p>
              <p className="hint">
                This is a descriptive comparison of the available pairs; it is
                not a significance test. Active experiments can have incomplete
                pairs.
              </p>
              <a href={url}>Paired report ↓</a>
              <div className="comparison-table-scroll">
                <table className="comparison-table">
                  <caption>
                    {metric.replaceAll("_", " ")} · challenger minus baseline
                  </caption>
                  <thead>
                    <tr>
                      <th>Seed</th>
                      <th>Baseline</th>
                      <th>Challenger</th>
                      <th>Difference / exclusion</th>
                    </tr>
                  </thead>
                  <tbody>
                    {paired.pairs.map((pair) => (
                      <tr key={pair.seed}>
                        <th scope="row">{pair.seed}</th>
                        <td>{format(pair.baseline_value)}</td>
                        <td>{format(pair.challenger_value)}</td>
                        <td>{format(pair.difference)}</td>
                      </tr>
                    ))}
                    {paired.exclusions.map((item) => (
                      <tr key={item.seed}>
                        <th scope="row">{item.seed}</th>
                        <td>—</td>
                        <td>—</td>
                        <td>{item.reasons.join("; ")}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </>
          )}
        </>
      )}
    </section>
  );
}
