import json

import pytest

from roboforge.cli import main
from roboforge.config import RunConfig
from roboforge.experiments import ExperimentConfig, SweepAxis, paired_differences, run_experiment
from roboforge.replay import ReplayLog
from roboforge.simulation import Simulator


def base():
    return RunConfig.model_validate(
        {
            "commands": [{"left": 2, "right": 4, "steps": 10}],
            "sensors": [{"type": "encoder", "rate_hz": 100, "latency": 0.02}],
        }
    )


def test_sweep_identity_statistics_and_paired_seeds(tmp_path):
    spec = ExperimentConfig(
        base=base(), seeds=(1, 2, 3), axes=(SweepAxis(path="commands.0.left", values=(2, 3)),)
    )
    first, second = run_experiment(spec, tmp_path), run_experiment(spec, tmp_path)
    assert first != second
    report = json.loads((first / "report.json").read_text())
    repeated = json.loads((second / "report.json").read_text())
    assert report["finished_trials"] == 6
    assert [r["metrics"] for r in report["trials"]] == [r["metrics"] for r in repeated["trials"]]
    assert [r["config_sha256"] for r in report["trials"]] == [
        r["config_sha256"] for r in repeated["trials"]
    ]
    group = report["groups"]["group-0000"]
    assert group["failure_rate"] == 0 and group["metrics"]["duration_s"][
        "sample_stddev"
    ] == pytest.approx(0)
    paired = paired_differences(report, "group-0000", "group-0001", "path_length_m")
    assert len(paired["pairs"]) == 3 and paired["excluded_seeds"] == []
    assert paired["mean_difference"] == pytest.approx(0.0025)


def test_failures_and_invalid_variants_are_retained(tmp_path):
    spec = ExperimentConfig(
        base=base(), seeds=(1, 2), axes=(SweepAxis(path="simulation.dt", values=(0.01, -1)),)
    )

    def runner(config):
        if config.seed == 2:
            raise RuntimeError("deliberate test failure")
        return Simulator(config).run()

    root = run_experiment(spec, tmp_path, runner=runner)
    report = json.loads((root / "report.json").read_text())
    assert [r["status"] for r in report["trials"]] == ["completed", "error", "error", "error"]
    assert report["groups"]["group-0001"]["failure_rate"] == 1
    assert report["trials"][1]["error"]["type"] == "RuntimeError"
    assert (
        len(
            paired_differences(report, "group-0000", "group-0001", "path_length_m")[
                "excluded_seeds"
            ]
        )
        == 2
    )


def test_budgets_and_path_errors_fail_before_creating_runs(tmp_path):
    with pytest.raises(ValueError, match="run budget"):
        ExperimentConfig(base=base(), seeds=(1, 2), max_runs=1)
    spec = ExperimentConfig(base=base(), axes=(SweepAxis(path="robot.typo", values=(1,)),))
    with pytest.raises(ValueError, match="unknown sweep"):
        run_experiment(spec, tmp_path)
    assert list(tmp_path.iterdir()) == []
    with pytest.raises(ValueError, match="step budget"):
        run_experiment(ExperimentConfig(base=base(), max_total_steps=1), tmp_path)


def test_replay_matches_exact_substep_and_respects_delivery(tmp_path, monkeypatch):
    config = base()
    original = Simulator(config).run()
    root = run_experiment(ExperimentConfig(base=config), tmp_path)
    report = json.loads((root / "report.json").read_text())
    trial = root / report["trials"][0]["trial_id"]

    def forbidden(*args):
        raise AssertionError("replay must not rerun the simulator")

    monkeypatch.setattr(Simulator, "run", forbidden)
    replay = ReplayLog(trial)
    assert replay.at(0.1).state == original.states[-1]
    assert replay.at(0.025).state.pose.as_array() == pytest.approx(
        original.motions[2].pose_at(0.5).as_array(), rel=0, abs=1e-15
    )
    assert all(r.delivery_time <= 0.025 for r in replay.at(0.025).readings)
    assert replay.at(0).readings == ()
    assert main(["replay", str(trial), "--time", ".025"]) == 0
    with pytest.raises(ValueError):
        replay.at(100)
    with (trial / "trajectory.csv").open("a") as stream:
        stream.write("tampered")
    with pytest.raises(ValueError, match="integrity"):
        ReplayLog(trial)


def test_custom_metric_failure_cannot_report_success(tmp_path):
    root = run_experiment(
        ExperimentConfig(base=base()),
        tmp_path,
        metric_functions={"bad": lambda result: float("nan")},
    )
    report = json.loads((root / "report.json").read_text())
    assert report["trials"][0]["status"] == "error"


def test_cli_experiment(tmp_path):
    config_path = tmp_path / "experiment.json"
    config_path.write_text(ExperimentConfig(base=base()).model_dump_json())
    assert main(["experiment", str(config_path), "--output", str(tmp_path / "runs")]) == 0
