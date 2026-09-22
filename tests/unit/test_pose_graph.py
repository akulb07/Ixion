import numpy as np
import pytest

from roboforge.geometry import Pose2
from roboforge.pose_graph import PoseGraph, edge_model
from roboforge.slam import relative


def test_edge_jacobians():
    a, b, z = Pose2(1, 2, 0.3), Pose2(2, 3, 0.5), Pose2(0.2, 0.4, 0.1)
    _, ja, jb = edge_model(a, b, z)
    epsilon = 1e-6
    for pose_index, jacobian in enumerate((ja, jb)):
        for axis in range(3):
            delta = np.eye(3)[axis] * epsilon
            plus, minus = [a, b], [a, b]
            plus[pose_index] = Pose2(*([a, b][pose_index].as_array() + delta))
            minus[pose_index] = Pose2(*([a, b][pose_index].as_array() - delta))
            numerical = (edge_model(*plus, z)[0] - edge_model(*minus, z)[0]) / (2 * epsilon)
            np.testing.assert_allclose(jacobian[:, axis], numerical, atol=1e-9)


def test_anchored_loop_recovers_consistent_square():
    truth = [Pose2(0, 0, 0), Pose2(1, 0, 0.5), Pose2(1, 1, 1), Pose2(0, 1, 1.5), Pose2(0, 0, 0)]
    graph = PoseGraph()
    for i, pose in enumerate(truth):
        graph.add_pose(Pose2(pose.x + 0.1 * i, pose.y + 0.05 * i, pose.theta + 0.03 * i))
    for i in range(len(truth) - 1):
        graph.add_constraint(i, i + 1, relative(truth[i], truth[i + 1]), np.eye(3) * 100)
    graph.add_constraint(0, 4, Pose2(), np.eye(3) * 100, "loop")
    result = graph.optimize()
    assert result.status == "converged"
    assert result.poses[0] == truth[0]
    np.testing.assert_allclose(
        [p.as_array() for p in result.poses], [p.as_array() for p in truth], atol=1e-7
    )
    assert all(b <= a for a, b in zip(result.costs, result.costs[1:]))
    assert result == graph.optimize()
    assert graph.poses[-1] != result.poses[-1]  # optimizer returns a snapshot


def test_disconnected_and_bad_edges_rejected():
    graph = PoseGraph()
    graph.add_pose(Pose2())
    graph.add_pose(Pose2(1, 0))
    with pytest.raises(ValueError, match="connected"):
        graph.optimize()
    with pytest.raises(ValueError):
        graph.add_constraint(0, 2, Pose2(), np.eye(3))
    with pytest.raises(ValueError):
        graph.add_constraint(0, 1, Pose2(), -np.eye(3))


def test_single_anchor_is_already_solved():
    graph = PoseGraph()
    graph.add_pose(Pose2(2, 3, 0.4))
    result = graph.optimize()
    assert result.status == "converged" and result.costs == (0.0,)
