"""Bounded grid and seeded sampling planners for a circular robot in a known map."""

import heapq
import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

from roboforge.config import Environment
from roboforge.core import positive
from roboforge.geometry import Pose2, Vector2
from roboforge.physics import CollisionWorld, KinematicMotion
from roboforge.robotics import DifferentialDrive, WheelSpeeds


@dataclass(frozen=True, slots=True)
class PlanResult:
    algorithm: str
    status: Literal["success", "invalid_start", "invalid_goal", "no_path", "budget_exceeded"]
    path: tuple[Vector2, ...] = ()
    length: float = 0.0
    expanded: int = 0
    collision_checks: int = 0
    seed: int | None = None


def _budget(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def path_length(path: tuple[Vector2, ...]) -> float:
    return sum((b - a).norm for a, b in zip(path, path[1:]))


class PlanningWorld:
    """Collision oracle shared by planners. Clearance includes the disk radius."""

    def __init__(
        self, environment: Environment, radius: float, spatial_tolerance: float = 1e-6
    ) -> None:
        self.environment = environment
        self.radius = positive(radius, "planner footprint radius")
        self.tolerance = positive(spatial_tolerance, "planner collision tolerance")
        self.world = CollisionWorld(environment)
        self.checks = 0

    def point_free(self, point: Vector2) -> bool:
        self.checks += 1
        return not self.world.query(point, self.radius).collision

    def segment_free(self, start: Vector2, end: Vector2) -> bool:
        self.checks += 1
        delta = end - start
        distance = delta.norm
        motion = KinematicMotion(
            Pose2(start.x, start.y, math.atan2(delta.y, delta.x)),
            DifferentialDrive(1, 2),
            WheelSpeeds(distance, distance),
            1,
        )
        return not self.world.sweep(motion, self.radius, spatial_tolerance=self.tolerance).blocked


def _endpoints(
    world: PlanningWorld, start: Vector2, goal: Vector2, algorithm: str, seed: int | None = None
) -> PlanResult | None:
    if not world.point_free(start):
        return PlanResult(algorithm, "invalid_start", collision_checks=world.checks, seed=seed)
    if not world.point_free(goal):
        return PlanResult(algorithm, "invalid_goal", collision_checks=world.checks, seed=seed)
    if start == goal:
        return PlanResult(algorithm, "success", (start,), collision_checks=world.checks, seed=seed)
    return None


def grid_plan(
    environment: Environment,
    start: Vector2,
    goal: Vector2,
    radius: float,
    resolution: float = 0.25,
    algorithm: Literal["astar", "dijkstra"] = "astar",
    max_expansions: int = 100000,
) -> PlanResult:
    """Shortest path on an 8-connected grid plus local start/goal connectors.

    Each connector and diagonal is swept, preventing corner cutting. Optimality
    refers to this graph, not the continuous world. No whole-grid allocation.
    """
    if algorithm not in ("astar", "dijkstra"):
        raise ValueError("grid algorithm must be astar or dijkstra")
    resolution = positive(resolution, "grid resolution")
    max_expansions = _budget(max_expansions, "max expansions")
    world = PlanningWorld(environment, radius)
    early = _endpoints(world, start, goal, algorithm)
    if early is not None:
        return early
    # Cell centers strictly inside the world; partial final cells are omitted.
    nx, ny = math.floor(environment.width / resolution), math.floor(environment.height / resolution)
    start_key, goal_key = (-1, -1), (-2, -2)

    def point(key):
        if key == start_key:
            return start
        if key == goal_key:
            return goal
        return Vector2((key[0] + 0.5) * resolution, (key[1] + 0.5) * resolution)

    def nearby(p):
        ix, iy = math.floor(p.x / resolution), math.floor(p.y / resolution)
        return {
            (x, y)
            for x in range(ix - 1, ix + 2)
            for y in range(iy - 1, iy + 2)
            if 0 <= x < nx and 0 <= y < ny
        }

    start_neighbors, goal_neighbors = nearby(start), nearby(goal)
    edge_cache = {}

    def edge_free(a, b):
        key = tuple(sorted((a, b)))
        if key not in edge_cache:
            edge_cache[key] = world.segment_free(point(a), point(b))
        return edge_cache[key]

    costs, parents = {start_key: 0.0}, {}
    queue = [(0.0, 0.0, start_key)]
    expanded = 0
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        if current == goal_key:
            keys = [current]
            while keys[-1] != start_key:
                keys.append(parents[keys[-1]])
            path = tuple(point(k) for k in reversed(keys))
            return PlanResult(algorithm, "success", path, path_length(path), expanded, world.checks)
        if expanded >= max_expansions:
            return PlanResult(
                algorithm, "budget_exceeded", expanded=expanded, collision_checks=world.checks
            )
        expanded += 1
        if current == start_key:
            neighbors = set(start_neighbors)
            if (goal - start).norm <= resolution * math.sqrt(2):
                neighbors.add(goal_key)
        else:
            x, y = current
            neighbors = {
                (x + dx, y + dy)
                for dx in (-1, 0, 1)
                for dy in (-1, 0, 1)
                if (dx or dy) and 0 <= x + dx < nx and 0 <= y + dy < ny
            }
            if current in goal_neighbors:
                neighbors.add(goal_key)
        for neighbor in sorted(neighbors):
            length = (point(neighbor) - point(current)).norm
            candidate = cost + length
            if candidate >= costs.get(neighbor, math.inf) or not edge_free(current, neighbor):
                continue
            costs[neighbor], parents[neighbor] = candidate, current
            heuristic = (point(neighbor) - goal).norm if algorithm == "astar" else 0.0
            heapq.heappush(queue, (candidate + heuristic, candidate, neighbor))
    return PlanResult(algorithm, "no_path", expanded=expanded, collision_checks=world.checks)


def sampling_plan(
    environment: Environment,
    start: Vector2,
    goal: Vector2,
    radius: float,
    algorithm: Literal["rrt", "rrt_star"] = "rrt",
    seed: int = 42,
    iterations: int = 2000,
    step_size: float = 0.5,
    goal_bias: float = 0.1,
    rewire_radius: float = 1.0,
) -> PlanResult:
    """Seeded RRT or fixed-radius RRT* with descendant cost propagation.

    Sampling budget exhaustion does not establish infeasibility. RRT* keeps
    improving until its iteration budget is used, even after finding a path.
    """
    if algorithm not in ("rrt", "rrt_star"):
        raise ValueError("sampling algorithm must be rrt or rrt_star")
    iterations = _budget(iterations, "iterations")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    step_size = positive(step_size, "step size")
    rewire_radius = positive(rewire_radius, "rewire radius")
    from roboforge.core import finite

    goal_bias = finite(goal_bias, "goal bias")
    if not 0 <= goal_bias <= 1:
        raise ValueError("goal bias must be in [0, 1]")
    world = PlanningWorld(environment, radius)
    early = _endpoints(world, start, goal, algorithm, seed)
    if early is not None:
        return early
    rng = np.random.Generator(np.random.PCG64(seed))
    nodes, parents, costs, children = [start], [-1], [0.0], [set()]
    goals = set()
    if (goal - start).norm <= step_size and world.segment_free(start, goal):
        goals.add(0)
    expanded = 0
    for _ in range(iterations):
        if goals and algorithm == "rrt":
            break
        expanded += 1
        target = (
            goal
            if rng.random() < goal_bias
            else Vector2(rng.uniform(0, environment.width), rng.uniform(0, environment.height))
        )
        distances = [(node - target).norm for node in nodes]
        nearest = min(range(len(nodes)), key=distances.__getitem__)
        distance = distances[nearest]
        if distance <= 1e-12:
            continue
        candidate = nodes[nearest] + (target - nodes[nearest]) * min(1, step_size / distance)
        if not world.segment_free(nodes[nearest], candidate):
            continue
        parent = nearest
        near = []
        if algorithm == "rrt_star":
            near = [i for i, node in enumerate(nodes) if (node - candidate).norm <= rewire_radius]
            for i in near:
                if (
                    costs[i] + (nodes[i] - candidate).norm
                    < costs[parent] + (nodes[parent] - candidate).norm
                ):
                    if world.segment_free(nodes[i], candidate):
                        parent = i
        index = len(nodes)
        nodes.append(candidate)
        parents.append(parent)
        costs.append(costs[parent] + (nodes[parent] - candidate).norm)
        children.append(set())
        children[parent].add(index)
        for i in near:
            new_cost = costs[index] + (nodes[i] - candidate).norm
            if i == 0 or new_cost >= costs[i] - 1e-12:
                continue
            if not world.segment_free(candidate, nodes[i]):
                continue
            children[parents[i]].remove(i)
            parents[i] = index
            children[index].add(i)
            adjustment = new_cost - costs[i]
            stack = [i]
            while stack:
                descendant = stack.pop()
                costs[descendant] += adjustment
                stack.extend(sorted(children[descendant]))
        if (candidate - goal).norm <= step_size and world.segment_free(candidate, goal):
            goals.add(index)
    if not goals:
        return PlanResult(
            algorithm,
            "budget_exceeded",
            expanded=expanded,
            collision_checks=world.checks,
            seed=seed,
        )
    best = min(goals, key=lambda i: (costs[i] + (nodes[i] - goal).norm, i))
    chain = []
    while best != -1:
        chain.append(nodes[best])
        best = parents[best]
    path = tuple(reversed(chain))
    if path[-1] != goal:
        path += (goal,)
    return PlanResult(algorithm, "success", path, path_length(path), expanded, world.checks, seed)
