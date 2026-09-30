import json

import pytest
from fastapi.testclient import TestClient

from roboforge.api import create_app
from roboforge.cli import main
from roboforge.regression import RegressionPolicy, RegressionRequest, check_saved_runs
from tests.integration.test_service import config, finished


def policy(**changes):
    return RegressionPolicy.model_validate(
        {
            "name": "acceptance",
            "rules": [
                {"name": "no collisions", "metric": "collision_count", "maximum": 0},
                {"name": "duration change", "metric": "duration_s", "mode": "delta", "maximum": 0},
            ],
            **changes,
        }
    )


def test_api_cli_same_result_missing_evidence_and_explicit_setup_changes(tmp_path):
    store = tmp_path / "runs"
    with TestClient(create_app(store)) as client:
        service = client.app.state.service
        a = finished(service, service.submit(config())["id"])
        b = finished(service, service.submit(config())["id"])
        request = RegressionRequest(baseline_id=a["id"], candidate_id=b["id"], policy=policy())
        api = client.post("/api/regressions", json=request.model_dump(mode="json"))
        assert api.status_code == 200 and api.json()["status"] == "pass"
        local = check_saved_runs(store, request)
        assert local["checks"] == api.json()["checks"]
        design = tmp_path / "policy.json"
        design.write_text(policy().model_dump_json())
        args = [
            "check",
            str(design),
            "--store",
            str(store),
            "--baseline",
            a["id"],
            "--candidate",
            b["id"],
            "--output",
            str(tmp_path / "pass.json"),
        ]
        before = {p: p.read_bytes() for p in store.rglob("*.json")}
        assert main(args) == 0
        assert main(args) == 2  # Refuse to replace a previous report.
        assert {p: p.read_bytes() for p in before} == before
        changed = config().model_dump(mode="json")
        changed["commands"][0]["steps"] *= 2
        from roboforge.config import RunConfig

        c = finished(service, service.submit(RunConfig.model_validate(changed))["id"])
        body = {**request.model_dump(mode="json"), "candidate_id": c["id"]}
        result = client.post("/api/regressions", json=body).json()
        assert result["status"] == "inconclusive"
        assert result["checks"][1]["unexpected_config_changes"] == ["commands"]
        body["policy"]["allowed_config_changes"] = ["commands"]
        result = client.post("/api/regressions", json=body).json()
        assert result["status"] == "fail"
        assert result["checks"][-1]["measured"] == pytest.approx(0.1)
        design.write_text(json.dumps(body["policy"]))
        args[args.index("--candidate") + 1] = c["id"]
        args[-1] = str(tmp_path / "fail.json")
        assert main(args) == 3
        body["policy"]["rules"] = [{"name": "unknown", "metric": "not_recorded", "maximum": 0}]
        design.write_text(json.dumps(body["policy"]))
        args[-1] = str(tmp_path / "missing.json")
        assert main(args) == 4
        assert json.loads((tmp_path / "missing.json").read_text())["checks"][-1]["measured"] is None
        # Artifact corruption is an input error, never a pass or ordinary regression.
        (store / b["id"] / "metrics.json").write_text('{"collision_count": 0}')
        assert (
            client.post("/api/regressions", json=request.model_dump(mode="json")).status_code == 409
        )


@pytest.mark.parametrize(
    "rules",
    [
        [],
        [{"name": "x", "metric": "x"}],
        [{"name": "x", "metric": "x", "minimum": 2, "maximum": 1}],
        [{"name": "x", "metric": "x", "maximum": float("nan")}],
        [{"name": "x", "metric": "x", "maximum": 0}] * 2,
    ],
)
def test_invalid_policies_rejected(rules):
    with pytest.raises(ValueError):
        policy(rules=rules)


def test_failed_candidate_cannot_pass_with_missing_metrics(tmp_path, monkeypatch):
    from roboforge.simulation import Simulator

    with TestClient(create_app(tmp_path)) as client:
        service = client.app.state.service
        a = finished(service, service.submit(config())["id"])

        def broken(*args, **kwargs):
            raise RuntimeError("controller crashed")

        monkeypatch.setattr(Simulator, "run", broken)
        b = finished(service, service.submit(config())["id"])
        result = client.post(
            "/api/regressions",
            json=RegressionRequest(
                baseline_id=a["id"], candidate_id=b["id"], policy=policy()
            ).model_dump(mode="json"),
        ).json()
        assert result["status"] == "fail"
        assert result["checks"][0]["status"] == "fail"
        assert result["checks"][-1]["status"] == "inconclusive"


def test_signed_bounds_zero_baseline_and_software_acknowledgment(monkeypatch):
    import roboforge.regression as regression

    comparison = {
        "runs": [
            {"status": "completed", "metrics": {"heading": 0}},
            {"status": "completed", "metrics": {"heading": -0.2}},
        ],
        "config_differences": [],
        "same_software": False,
        "created_utc": "2026-09-30T00:00:00Z",
        "software_version": "test",
    }
    monkeypatch.setattr(regression, "compare_runs", lambda *args: comparison)
    design = policy(
        rules=[
            {
                "name": "signed heading",
                "metric": "heading",
                "mode": "delta",
                "minimum": -0.2,
                "maximum": 0,
            }
        ]
    )
    request = RegressionRequest(baseline_id="a", candidate_id="b", policy=design)
    assert regression.check_regression(None, request)["status"] == "inconclusive"
    request = request.model_copy(
        update={"policy": design.model_copy(update={"allow_software_change": True})}
    )
    result = regression.check_regression(None, request)
    assert result["status"] == "pass" and result["checks"][-1]["measured"] == -0.2
    comparison["runs"][1]["metrics"]["heading"] = -0.3
    assert regression.check_regression(None, request)["status"] == "fail"
