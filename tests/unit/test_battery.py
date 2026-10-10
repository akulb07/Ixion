import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from roboforge.cli import main
from roboforge.hardware import load_project
from roboforge.hardware.battery import BatteryModel, BatteryState, OCVPoint


def model(**changes):
    return BatteryModel(
        **(
            dict(
                chemistry="LiPo",
                capacity_ah=2,
                internal_resistance_ohm=0.2,
                ocv_curve=[
                    dict(soc=0, voltage_v=6),
                    dict(soc=0.5, voltage_v=7.4),
                    dict(soc=1, voltage_v=8.4),
                ],
                max_continuous_current_a=3,
                max_burst_current_a=5,
            )
            | changes
        )
    )


def test_no_load_preserves_charge_and_has_no_sag():
    result = model().discharge(BatteryState(), 0, 100)
    assert result.state.soc == 1
    assert result.end.terminal_voltage_v == 8.4
    assert result.end.power_w == 0
    assert result.end.charge_limited_runtime_s is None


def test_known_load_sag_and_coulomb_count():
    result = model().discharge(BatteryState(), 2, 900)
    assert result.discharged_ah == 0.5
    assert result.state.soc == 0.75
    assert result.start.terminal_voltage_v == 8
    assert result.end.open_circuit_voltage_v == pytest.approx(7.9)
    assert result.end.terminal_voltage_v == pytest.approx(7.5)
    assert result.end.charge_limited_runtime_s == 2700
    assert result.state.peak_current_a == 2
    assert result.start.power_w + result.start.internal_loss_w == pytest.approx(8.4 * 2)


def test_piecewise_curve_is_used_instead_of_nominal_voltage():
    assert model().open_circuit_voltage(0.25) == pytest.approx(6.7)
    assert model().open_circuit_voltage(0.5) == 7.4


def test_exact_depletion_stops_end_current_and_rejects_further_discharge():
    result = model().discharge(BatteryState(), 2, 3600)
    assert result.state.soc == 0 and result.end.current_a == 0
    with pytest.raises(ValueError, match="depleted"):
        model().operating_point(result.state, 1)
    with pytest.raises(ValueError, match="remaining charge"):
        model().discharge(result.state, 1, 1)


def test_no_partial_state_mutation_on_failure():
    state = BatteryState(soc=0.1)
    with pytest.raises(ValueError, match="remaining charge"):
        model().discharge(state, 2, 1000)
    assert state.soc == 0.1 and state.peak_current_a == 0


def test_current_limits_warn_without_clipping():
    point = model().operating_point(BatteryState(), 6)
    assert point.current_a == 6
    assert point.diagnostics == ("continuous_current_exceeded", "burst_current_exceeded")
    assert model().operating_point(BatteryState(), 3).diagnostics == ()


def test_missing_current_ratings_are_unknown():
    point = model(max_continuous_current_a=None, max_burst_current_a=None).operating_point(
        BatteryState(), 1
    )
    assert point.diagnostics == (
        "continuous_current_rating_unknown",
        "burst_current_rating_unknown",
    )


def test_peak_current_survives_later_lighter_load():
    battery = model()
    first = battery.discharge(BatteryState(), 4, 1)
    second = battery.discharge(first.state, 1, 1)
    assert second.state.peak_current_a == 4
    assert battery.discharge(second.state, 5, 0).state.peak_current_a == 4


@pytest.mark.parametrize("current", [-1, True, float("nan"), float("inf")])
def test_invalid_or_charging_current_rejected(current):
    with pytest.raises(ValueError):
        model().discharge(BatteryState(), current, 1)


def test_voltage_collapse_detected_at_start_and_within_step():
    battery = model(internal_resistance_ohm=2)
    with pytest.raises(ValueError, match="collapses"):
        battery.operating_point(BatteryState(), 5)
    with pytest.raises(ValueError, match="collapses"):
        battery.discharge(BatteryState(), 4, 1800)


@pytest.mark.parametrize(
    "curve", [[(0, 8), (1, 6)], [(0.1, 6), (1, 8)], [(0, 6), (0.5, 7), (0.5, 8), (1, 9)]]
)
def test_invalid_curves_rejected(curve):
    with pytest.raises(ValidationError):
        model(ocv_curve=[OCVPoint(soc=s, voltage_v=v) for s, v in curve])


def test_burst_rating_must_not_be_below_continuous():
    with pytest.raises(ValidationError):
        model(max_burst_current_a=2)


def test_split_steps_match_one_constant_current_step():
    battery = model(chemistry="Li-ion")
    whole = battery.discharge(BatteryState(), 1, 120)
    half = battery.discharge(BatteryState(), 1, 60)
    split = battery.discharge(half.state, 1, 60)
    assert split.state.soc == pytest.approx(whole.state.soc)
    assert split.end.terminal_voltage_v == pytest.approx(whole.end.terminal_voltage_v)


def test_reference_cli_preserves_pack_assumptions(capsys):
    path = Path(__file__).parents[2] / "examples/esp32_diff_drive/robot.yaml"
    component = next(c for c in load_project(path).components if c.id == "battery")
    assert BatteryModel.from_component(component).capacity_ah == 2.2
    assert (
        main(
            [
                "battery-step",
                str(path),
                "--component",
                "battery",
                "--current",
                "2",
                "--seconds",
                "60",
            ]
        )
        == 0
    )
    report = json.loads(capsys.readouterr().out)
    assert report["result_type"] == "estimate"
    assert report["result"]["start"]["terminal_voltage_v"] == pytest.approx(8.1)
    assert "continuous_current_rating_unknown" in report["result"]["start"]["diagnostics"]
    assert all(p["evidence"] == "assumed" for p in report["inputs"])
