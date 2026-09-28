import pytest
from fastapi.testclient import TestClient

from roboforge.api import create_app
from roboforge.experiments import paired_differences
from tests.integration.test_batches import batch_finished, specification


def trial(group, seed, value=None, status="completed"):
    return {
        "group": group,
        "seed": seed,
        "status": status,
        "metrics": {} if value is None else {"distance": value},
    }


def test_pairs_match_seed_not_order_and_retain_exclusion_reasons():
    report = {
        "trials": [
            trial("a", 2, 20),
            trial("b", 1, 7),
            trial("a", 1, 3),
            trial("b", 2, 5),
            trial("a", 3, 1),
            trial("b", 3, 8, "collision"),
            trial("a", 4, 1),
            trial("b", 4),
            trial("a", 5, 1),
        ]
    }
    result = paired_differences(report, "a", "b", "distance")
    assert [p["seed"] for p in result["pairs"]] == [1, 2]
    assert [p["difference"] for p in result["pairs"]] == [4, -15]
    assert result["mean_difference"] == -5.5
    assert result["matched_pairs"] == 2
    assert result["exclusions"] == [
        {"seed": 3, "reasons": ["challenger: collision"]},
        {"seed": 4, "reasons": ["challenger: missing metric"]},
        {"seed": 5, "reasons": ["challenger: missing trial"]},
    ]
    assert paired_differences(report, "b", "a", "distance")["mean_difference"] == 5.5


def test_empty_pairs_and_nonfinite_differences_are_not_zero():
    report = {
        "trials": [
            trial("a", 1, 1e308),
            trial("b", 1, -1e308),
            trial("a", 2, 1),
            trial("b", 2, float("nan")),
        ]
    }
    result = paired_differences(report, "a", "b", "distance")
    assert result["mean_difference"] is None
    assert result["matched_pairs"] == 0
    assert result["excluded_seeds"] == [1, 2]
    assert result["exclusions"][0]["reasons"] == ["difference is non-finite"]


def test_ambiguous_or_invalid_pairing_is_rejected():
    report = {"trials": [trial("a", 1, 2), trial("b", 1, 5)]}
    for baseline, challenger, metric in [
        ("a", "a", "distance"),
        ("a", "c", "distance"),
        ("a", "b", "typo"),
    ]:
        with pytest.raises(ValueError):
            paired_differences(report, baseline, challenger, metric)
    report["trials"].append(trial("a", 1, 9))
    with pytest.raises(ValueError, match="duplicate seed"):
        paired_differences(report, "a", "b", "distance")


def test_paired_endpoint_uses_saved_trials_without_new_runs(tmp_path):
    with TestClient(create_app(tmp_path)) as client:
        batches = client.app.state.batches
        batch = batch_finished(batches, batches.submit(specification())["id"])
        endpoint = f"/api/experiments/{batch['id']}/paired"
        params = {"baseline": "group-0000", "challenger": "group-0001", "metric": "path_length_m"}
        response = client.get(endpoint, params=params)
        assert response.status_code == 200
        assert "attachment" in response.headers["content-disposition"]
        result = response.json()
        assert result["batch_id"] == batch["id"] and result["batch_status"] == "completed"
        assert result["matched_pairs"] == 2 and result["excluded_seeds"] == []
        expected = (
            batch["trials"][2]["metrics"]["path_length_m"]
            - batch["trials"][0]["metrics"]["path_length_m"]
        )
        assert result["mean_difference"] == pytest.approx(expected)
        assert client.get(endpoint, params={**params, "challenger": "bad"}).status_code == 422
        assert client.get(endpoint, params={**params, "metric": "bad"}).status_code == 422
        assert client.get(endpoint).status_code == 422
        assert (
            client.get("/api/experiments/batch-" + "0" * 32 + "/paired", params=params).status_code
            == 404
        )
        assert client.app.state.service.list()["total"] == 4
