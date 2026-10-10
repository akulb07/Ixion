import math

import pytest

from roboforge.hardware.battery import BatteryState
from tests.unit.test_powertrain import system


def test_trace_recomputes_sag_current_and_preserves_initial_state():
    model = system()
    initial = BatteryState()
    trace = model.discharge_trace(initial, 60, 1)
    assert len(trace.samples) == 61
    assert initial.soc == 1
    assert trace.samples[-1].time_s == 60
    assert trace.samples[-1].state.soc < 1
    assert trace.samples[-1].point.battery.current_a < trace.samples[0].point.battery.current_a
    assert (
        trace.samples[-1].point.battery.terminal_voltage_v
        < trace.samples[0].point.battery.terminal_voltage_v
    )
    assert trace == model.discharge_trace(initial, 60, 1)


def test_charge_accounting_uses_interval_start_current():
    trace = system().discharge_trace(BatteryState(), 2.5, 1)
    assert [s.time_s for s in trace.samples] == [0, 1, 2, 2.5]
    charge = sum(
        a.point.battery.current_a * (b.time_s - a.time_s) / 3600
        for a, b in zip(trace.samples, trace.samples[1:])
    )
    assert trace.samples[-1].state.soc == pytest.approx(1 - charge / 2)


def test_smaller_steps_converge_to_linear_ocv_analytic_solution():
    # OCV=6+2*SOC; at stall I=A*OCV/(1+Rb*A), A=2/(2+.5).
    rate = 0.8 / (1 + 0.2 * 0.8) / (2 * 3600)
    exact = (1 + 3) * math.exp(-2 * rate * 60) - 3
    coarse = system().discharge_trace(BatteryState(), 60, 10).samples[-1].state.soc
    fine = system().discharge_trace(BatteryState(), 60, 1).samples[-1].state.soc
    assert abs(fine - exact) < abs(coarse - exact) / 5


@pytest.mark.parametrize(
    "duration,step", [(0, 1), (1, 0), (1, -1), (10001, 1), (float("inf"), 1), (True, 1)]
)
def test_invalid_or_unbounded_trace_rejected(duration, step):
    with pytest.raises(ValueError):
        system().discharge_trace(BatteryState(), duration, step)


def test_depletion_does_not_publish_a_complete_trace():
    with pytest.raises(ValueError):
        system().discharge_trace(BatteryState(soc=0.001), 60, 1)
