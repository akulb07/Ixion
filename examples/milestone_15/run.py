"""Run and visualize actual planner outcomes on four versioned environments."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from roboforge.benchmarks import BenchmarkConfig, run_benchmarks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "milestone_15")
    args = parser.parse_args()
    config = BenchmarkConfig(iterations=300)
    directory = run_benchmarks(config, args.output)
    report = json.loads((directory / "report.json").read_text())
    assert len(report["trials"]) == 16
    assert all(
        r["status"] == "success"
        for r in report["trials"]
        if r["algorithm"] in ("astar", "dijkstra")
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ax, name in zip(axes.flat, config.cases):
        group = [r for r in report["trials"] if r["case"] == name]
        for index, record in enumerate(group):
            if record["status"] == "success":
                ax.bar(index, record["path_length_m"], color="#0099aa")
                ax.text(
                    index,
                    record["path_length_m"] + 0.2,
                    f"{record['path_length_m']:.2f}",
                    ha="center",
                )
            else:
                ax.text(
                    index, 0.1, record["status"].replace("_", "\n"), ha="center", color="#bd493d"
                )
        ax.set_xticks(range(4), config.algorithms)
        ax.set(
            ylabel="Successful path length [m]",
            title=name,
            ylim=(0, max((r["path_length_m"] or 0 for r in group), default=1) * 1.2),
        )
        ax.grid(axis="y", alpha=0.2)
    fig.suptitle("Planner benchmarks · seed 42 · 300 sampling iterations · failures retained")
    fig.tight_layout()
    fig.savefig(args.output / "benchmarks.png", dpi=150)
    plt.close(fig)
    summary = {
        "directory": directory.name,
        "trials": len(report["trials"]),
        "successes": sum(r["status"] == "success" for r in report["trials"]),
        "failures": [
            {"case": r["case"], "algorithm": r["algorithm"], "status": r["status"]}
            for r in report["trials"]
            if r["status"] != "success"
        ],
    }
    (args.output / "validation.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
