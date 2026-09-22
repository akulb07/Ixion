"""Small anchored SE(2) pose graphs with robust, damped least squares."""

import math
from dataclasses import dataclass

import numpy as np

from roboforge.core import positive, wrap_angle
from roboforge.geometry import Pose2
from roboforge.localization import covariance
from roboforge.slam import relative


@dataclass(frozen=True, slots=True)
class PoseConstraint:
    source: int
    target: int
    measurement: Pose2
    information: tuple[tuple[float, ...], ...]
    kind: str


@dataclass(frozen=True, slots=True)
class GraphResult:
    poses: tuple[Pose2, ...]
    status: str
    iterations: int
    costs: tuple[float, ...]


def edge_model(a: Pose2, b: Pose2, measurement: Pose2):
    """Residual z^-1*(a^-1*b); Jacobians use additive world-coordinate perturbations."""
    predicted = relative(a, b)
    cz, sz = math.cos(measurement.theta), math.sin(measurement.theta)
    rz_t = np.array([[cz, sz], [-sz, cz]])
    ca, sa = math.cos(a.theta), math.sin(a.theta)
    ra_t = np.array([[ca, sa], [-sa, ca]])
    position = rz_t @ np.array([predicted.x - measurement.x, predicted.y - measurement.y])
    error = np.array([*position, wrap_angle(predicted.theta - measurement.theta)])
    ja, jb = np.zeros((3, 3)), np.zeros((3, 3))
    ja[:2, :2], jb[:2, :2] = -rz_t @ ra_t, rz_t @ ra_t
    ja[:2, 2] = rz_t @ np.array([predicted.y, -predicted.x])
    ja[2, 2], jb[2, 2] = -1, 1
    return error, ja, jb


class PoseGraph:
    """Dense solver capped at 200 nodes; first pose fixes all three gauge freedoms."""

    def __init__(self):
        self.poses: list[Pose2] = []
        self.constraints: list[PoseConstraint] = []

    def add_pose(self, pose: Pose2) -> int:
        if not isinstance(pose, Pose2):
            raise ValueError("graph node must be a Pose2")
        if len(self.poses) >= 200:
            raise ValueError("pose graph exceeds 200-node budget")
        self.poses.append(pose)
        return len(self.poses) - 1

    def add_constraint(
        self, source: int, target: int, measurement: Pose2, information, kind: str = "odometry"
    ) -> None:
        if (
            any(
                isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < len(self.poses)
                for i in (source, target)
            )
            or source == target
        ):
            raise ValueError("constraint requires two distinct existing node indices")
        if not isinstance(measurement, Pose2) or kind not in ("odometry", "scan", "loop"):
            raise ValueError("constraint requires a pose and known kind")
        matrix = covariance(information, 3, definite=True)
        self.constraints.append(
            PoseConstraint(
                source,
                target,
                measurement,
                tuple(tuple(float(v) for v in row) for row in matrix),
                kind,
            )
        )

    def _connected(self):
        connected = {0}
        while True:
            before = len(connected)
            for edge in self.constraints:
                if edge.source in connected or edge.target in connected:
                    connected.update((edge.source, edge.target))
            if len(connected) == before:
                return len(connected) == len(self.poses)

    def optimize(
        self, *, iterations: int = 50, huber_delta: float = 3.0, tolerance: float = 1e-8
    ) -> GraphResult:
        if isinstance(iterations, bool) or not isinstance(iterations, int) or iterations < 1:
            raise ValueError("iterations must be a positive integer")
        huber_delta, tolerance = (
            positive(huber_delta, "Huber delta"),
            positive(tolerance, "tolerance"),
        )
        if not self.poses or not self._connected():
            raise ValueError("pose graph must be nonempty and connected to fixed node zero")
        poses = tuple(self.poses)
        size = 3 * (len(poses) - 1)
        roots = [np.linalg.cholesky(np.array(e.information)).T for e in self.constraints]

        def system(current, linearize):
            cost = 0.0
            hessian, gradient = np.zeros((size, size)), np.zeros(size)
            for edge, root in zip(self.constraints, roots):
                residual, ja, jb = edge_model(
                    current[edge.source], current[edge.target], edge.measurement
                )
                r = root @ residual
                norm = float(np.linalg.norm(r))
                cost += (
                    0.5 * norm**2
                    if norm <= huber_delta
                    else huber_delta * (norm - 0.5 * huber_delta)
                )
                if not linearize:
                    continue
                weight = 1.0 if norm <= huber_delta else huber_delta / norm
                blocks = [
                    (3 * (node - 1), root @ jacobian)
                    for node, jacobian in ((edge.source, ja), (edge.target, jb))
                    if node != 0
                ]
                for start, jacobian in blocks:
                    gradient[start : start + 3] += weight * jacobian.T @ r
                    for other, jacobian_other in blocks:
                        hessian[start : start + 3, other : other + 3] += (
                            weight * jacobian.T @ jacobian_other
                        )
            if (
                not math.isfinite(cost)
                or not np.isfinite(hessian).all()
                or not np.isfinite(gradient).all()
            ):
                raise ValueError("pose graph arithmetic overflow")
            return cost, hessian, gradient

        costs, damping = [], 1e-4
        for iteration in range(iterations):
            cost, hessian, gradient = system(poses, True)
            if not costs:
                costs.append(cost)
            if size == 0 or np.max(np.abs(gradient)) <= tolerance:
                return GraphResult(poses, "converged", iteration, tuple(costs))
            accepted = False
            for _ in range(12):
                diagonal = np.maximum(np.diag(hessian), 1.0)
                step = np.linalg.solve(hessian + damping * np.diag(diagonal), -gradient)
                candidate = (poses[0],) + tuple(
                    Pose2(*(p.as_array() + step[3 * i : 3 * i + 3]))
                    for i, p in enumerate(poses[1:])
                )
                new_cost = system(candidate, False)[0]
                if new_cost <= cost:
                    poses, accepted = candidate, True
                    costs.append(new_cost)
                    damping = max(1e-12, damping / 3)
                    break
                damping *= 10
            if not accepted:
                return GraphResult(poses, "stalled", iteration + 1, tuple(costs))
            if np.max(np.abs(step)) <= tolerance:
                return GraphResult(poses, "converged", iteration + 1, tuple(costs))
        return GraphResult(poses, "iteration_limit", iterations, tuple(costs))
