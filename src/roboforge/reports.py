"""Portable reports from recorded results; no simulation or metric recomputation."""

import csv
import io
import json
from html import escape


def _text(value):
    if value is None:
        return ""
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True)
    return str(value)


def _trial_table(report, kind):
    trials = report["trials"]
    if kind == "benchmark":
        columns = [
            "case",
            "algorithm",
            "seed",
            "status",
            "path_length_m",
            "planner_runtime_s",
            "expanded",
            "collision_checks",
            "verified_collision_free",
            "artifact",
            "error",
        ]
        return columns, [[trial.get(key) for key in columns] for trial in trials]
    columns = [
        "run_id",
        "trial_id",
        "group",
        "seed",
        "status",
        "parameters",
        "config_sha256",
        "navigation_outcome",
        "wall_runtime_s",
        "error",
    ]
    metrics = sorted({key for trial in trials for key in trial.get("metrics", {})})
    rows = [
        [trial.get(key) for key in columns] + [trial.get("metrics", {}).get(key) for key in metrics]
        for trial in trials
    ]
    return columns + [f"metric.{key}" for key in metrics], rows


def trial_csv(report, kind="experiment"):
    """One row per trial, including unfinished/failed rows. Missing data stays blank."""
    columns, rows = _trial_table(report, kind)
    stream = io.StringIO(newline="")
    writer = csv.writer(stream)

    def cell(value):
        text = _text(value)
        # Keep user strings as text when opened in a spreadsheet. Numeric negatives
        # remain numbers, and the unchanged machine-readable values stay in JSON.
        if isinstance(value, str) and text.lstrip().startswith(("=", "+", "-", "@")):
            return "'" + text
        return text

    writer.writerow([cell(value) for value in columns])
    writer.writerows([cell(value) for value in row] for row in rows)
    return stream.getvalue()


def _table(columns, rows):
    heading = "".join(f'<th scope="col">{escape(_text(value))}</th>' for value in columns)
    body = "".join(
        "<tr>" + "".join(f"<td>{escape(_text(value))}</td>" for value in row) + "</tr>"
        for row in rows
    )
    return f'<div class="table"><table><thead><tr>{heading}</tr></thead><tbody>{body}</tbody></table></div>'


def report_html(report, *, kind="experiment", design=None, cases=None, summaries=None):
    """Self-contained, printable HTML. All recorded text is escaped, including JSON."""
    title = "Planner benchmark" if kind == "benchmark" else report.get("name", "Experiment")
    metadata = {
        key: report[key]
        for key in (
            "id",
            "status",
            "suite_version",
            "software_version",
            "created_utc",
            "finished_utc",
            "expected_trials",
            "finished_trials",
            "specification_sha256",
        )
        if key in report
    }
    if kind == "benchmark":
        note = (
            "Path lengths describe successful, independently collision-checked paths only. "
            "Read them alongside failures. Budget exhaustion does not prove there is no route. "
            "Runtime is local wall time, not a cross-machine ranking; successful planner timing "
            "excludes validation and export. Deterministic planners ignore the seed."
        )
        summary = summaries or []
        columns = list(summary[0]) if summary else []
        summary_html = _table(columns, [[row.get(key) for key in columns] for row in summary])
    else:
        note = (
            "This is a saved snapshot; pending and running trials may change later. A completed "
            "batch can contain failed trials. Group statistics include finished trials only; "
            "failure means any status other than completed. Metrics include all available "
            "measurements, including from failed trials. Blank cells mean unavailable, not zero. "
            "Wilson intervals assume independent trials; repeated deterministic runs are not "
            "independent evidence. Navigation outcome is separate from execution status."
        )
        groups = report.get("groups", {})
        summary_html = _table(
            ["group", "finished trials", "failed trials", "failure rate", "Wilson 95% interval"],
            [
                [
                    key,
                    value["trials"],
                    value["failed_trials"],
                    value["failure_rate"],
                    value["failure_rate_wilson95"],
                ]
                for key, value in groups.items()
            ],
        )
        summary_html += _table(
            ["group", "metric", "measured trials", "mean", "sample stddev", "median", "p05", "p95"],
            [
                [
                    group,
                    metric,
                    *[
                        values.get(key)
                        for key in (
                            "measured_trials",
                            "mean",
                            "sample_stddev",
                            "median",
                            "p05",
                            "p95",
                        )
                    ],
                ]
                for group, value in groups.items()
                for metric, values in value["metrics"].items()
            ],
        )
    columns, rows = _trial_table(report, kind)
    if kind == "experiment":
        compact = ["group", "seed", "status", "navigation_outcome", "error"]
        trials_html = _table(
            compact, [[trial.get(key) for key in compact] for trial in report["trials"]]
        )
        trials_html += "".join(
            f"<details><summary>Trial {index + 1} · {escape(_text(trial.get('group')))} · "
            f"seed {escape(_text(trial.get('seed')))}</summary>"
            + _table(["Field / metric", "Recorded value"], zip(columns, row))
            + "</details>"
            for index, (trial, row) in enumerate(zip(report["trials"], rows))
        )
    else:
        trials_html = _table(columns, rows)
    attachments = {"Recorded snapshot": report}
    if design is not None:
        attachments["Design and budgets"] = design
    if cases is not None:
        attachments["Exact benchmark geometry"] = cases
    details = "".join(
        f"<details><summary>{label}</summary><pre>"
        + escape(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False))
        + "</pre></details>"
        for label, value in attachments.items()
    )
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src 'unsafe-inline'; base-uri 'none'; form-action 'none'">
<title>{escape(title)} — RoboForge</title>
<style>
body{{font:14px/1.5 system-ui,sans-serif;background:#0c0e11;color:#dce0e8;margin:32px}}
main{{max-width:1400px;margin:auto}}h1{{border-bottom:2px solid #b496ff;padding-bottom:12px}}
h2,summary{{color:#b496ff}}.table{{overflow:auto;margin:16px 0}}table{{border-collapse:collapse;width:100%}}
th,td{{border:1px solid #363c48;padding:8px;text-align:left;vertical-align:top;overflow-wrap:anywhere}}
th{{background:#191c22}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#111419;padding:16px}}
summary{{cursor:pointer;padding:10px 0}}.note{{border-left:3px solid #b496ff;padding:12px;background:#171a20}}
@media print{{body{{background:white;color:black;margin:0;font-size:10px}}h2,summary{{color:#503080}}
th,pre,.note{{background:white}}.table{{overflow:visible}}thead{{display:table-header-group}}tr{{break-inside:avoid}}}}
</style></head><body><main><p>RoboForge / {escape(kind)} report</p>
<h1>{escape(title)}</h1>{_table(["Field", "Recorded value"], metadata.items())}
<p class="note">{note}</p><h2>Summary</h2>{summary_html}
<h2>Every trial</h2><p>Blank cells mean unavailable. Times are seconds and path lengths are metres.</p>
{trials_html}<h2>Reproduction details</h2>{details}
</main></body></html>"""
