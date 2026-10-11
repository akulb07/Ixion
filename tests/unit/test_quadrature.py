import math

import pytest

from roboforge.hardware.quadrature import AngularSample, QuadratureEncoder


def samples(*angles):
    return tuple(AngularSample(time_s=i, angle_rad=a) for i, a in enumerate(angles))


def test_full_turn_has_four_edges_per_cycle_and_only_one_pin_changes():
    trace = QuadratureEncoder(cycles_per_revolution=10).trace(samples(0, math.tau))
    assert trace.final_count == 40
    assert len(trace.edges) == 40
    assert [(e.a, e.b) for e in trace.edges[:4]] == [
        (True, False),
        (True, True),
        (False, True),
        (False, False),
    ]
    for a, b in zip((trace.initial,) + trace.edges, trace.edges):
        assert (a.a != b.a) + (a.b != b.b) == 1
    assert trace.edges[0].time_s == pytest.approx(0.5 / 40)
    assert trace.edges[-1].time_s == pytest.approx(39.5 / 40)


def test_reverse_turn_and_return():
    encoder = QuadratureEncoder(cycles_per_revolution=2)
    trace = encoder.trace(samples(0, -math.tau, 0))
    assert len(trace.edges) == 16
    assert trace.edges[7].count == -8
    assert trace.final_count == 0
    assert trace.edges[0].a is False and trace.edges[0].b is True


def test_stationary_and_sub_count_motion_generate_no_edges():
    trace = QuadratureEncoder(cycles_per_revolution=1).trace(samples(0, 0, 0.1, 0))
    assert trace.edges == ()


def test_sample_subdivision_does_not_duplicate_threshold_edge():
    encoder = QuadratureEncoder(cycles_per_revolution=1)
    trace = encoder.trace(samples(0, math.pi / 4, math.pi / 2))
    assert len(trace.edges) == 1
    assert trace.edges[0].time_s == pytest.approx(1)


def test_initial_angle_sets_phase_without_inventing_prior_events():
    trace = QuadratureEncoder(cycles_per_revolution=1).trace(samples(math.pi))
    assert trace.initial.count == 2
    assert trace.initial.a and trace.initial.b
    assert trace.edges == ()


@pytest.mark.parametrize("angles", [(0, math.tau * 100_000), (0, 1e300)])
def test_excessive_output_or_precision_rejected(angles):
    with pytest.raises(ValueError):
        QuadratureEncoder(cycles_per_revolution=1).trace(samples(*angles))


def test_invalid_resolution_and_times_rejected():
    with pytest.raises(ValueError):
        QuadratureEncoder(cycles_per_revolution=True)
    with pytest.raises(ValueError, match="strictly increase"):
        QuadratureEncoder(cycles_per_revolution=1).trace(
            (AngularSample(time_s=0, angle_rad=0),) * 2
        )
