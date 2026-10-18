"""Compare an assumed extra payload on the same rover and feedback suite."""

import argparse
import hashlib
import json
from pathlib import Path

from roboforge.hardware import RobotProject, load_project
from roboforge.hardware.rover_comparison import compare_rover_suites
from roboforge.hardware.rover_report import render_rover_evidence
from roboforge.hardware.rover_suite import load_rover_suite, run_rover_suite


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extra-mass-kg", type=float, default=1)
    parser.add_argument(
        "--output", type=Path, required=True, help="new directory for inputs and results"
    )
    args = parser.parse_args()
    try:
        if args.output.exists():
            raise ValueError("comparison directory already exists; choose a new path")
        root = Path(__file__).parent
        baseline = load_project(root / "robot.yaml")
        data = baseline.model_dump()
        data["assembly"]["total_mass_kg"] += args.extra_mass_kg
        if args.extra_mass_kg < 0:
            raise ValueError("extra payload mass must be nonnegative")
        candidate = RobotProject.model_validate(data)
        suite = load_rover_suite(root / "rover_feedback_suite.yaml")
        before, after = run_rover_suite(baseline, suite), run_rover_suite(candidate, suite)
        comparison = compare_rover_suites(before, after)
        documents = {
            "baseline-project.json": baseline.model_dump(mode="json"),
            "candidate-project.json": candidate.model_dump(mode="json"),
            "suite.json": suite.model_dump(mode="json"),
            "baseline.json": before,
            "candidate.json": after,
            "comparison.json": comparison,
        }
        payloads = {
            name: json.dumps(document, indent=2, allow_nan=False) + "\n"
            for name, document in documents.items()
        }
        html = render_rover_evidence(
            comparison, hashlib.sha256(payloads["comparison.json"].encode("utf-8")).hexdigest()
        )
        args.output.mkdir(parents=True)
        for name, text in payloads.items():
            with (args.output / name).open("x", encoding="utf-8", newline="\n") as stream:
                stream.write(text)
        with (args.output / "comparison.html").open("x", encoding="utf-8") as stream:
            stream.write(html)
    except (OSError, ValueError) as exc:
        parser.exit(2, f"{exc}\n")
    print(
        f"Candidate acceptance: {comparison['status']}; inputs and comparison saved in {args.output}"
    )
    return 0 if comparison["status"] == "passed" else 3


if __name__ == "__main__":
    raise SystemExit(main())
