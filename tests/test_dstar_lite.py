"""D* Lite must return the same optimal cost as Dijkstra, always.

The point of an incremental search is that it is faster, never that it is
different. Every test here pins the result against `networkx.dijkstra_path`,
including after edge costs change, which is the case the incremental machinery
exists for and the one most likely to be subtly wrong.
"""

from __future__ import annotations

import random
from math import hypot, inf

import networkx as nx
import pytest

from research_sdk.planners.dstar_lite import DStarLite, path_cost


def _nx_graph(graph):
    g = nx.DiGraph()
    for u, succs in graph.items():
        g.add_node(u)
        for v, cost in succs.items():
            g.add_edge(u, v, weight=cost)
    return g


def _dijkstra_cost(graph, start, goal) -> float:
    try:
        path = nx.dijkstra_path(_nx_graph(graph), start, goal, weight="weight")
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return inf
    return path_cost(graph, path)


def _cost(graph, path) -> float:
    """Cost of a returned path, treating "no path" as infinite.

    `path_cost` sums edges, so an empty path is 0.0 by construction. That is
    the right answer for a sum and the wrong one for a comparison against
    Dijkstra, which reports an unreachable goal as inf.
    """
    return inf if len(path) < 2 else path_cost(graph, path)


def _random_grid(rows: int, cols: int, rng: random.Random, blocked: float = 0.15):
    """Undirected 4-connected grid with random costs and some missing cells."""
    nodes = [
        (r, c)
        for r in range(rows)
        for c in range(cols)
        if rng.random() > blocked or (r, c) in ((0, 0), (rows - 1, cols - 1))
    ]
    present = set(nodes)
    graph = {n: {} for n in nodes}
    for r, c in nodes:
        for dr, dc in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            n = (r + dr, c + dc)
            if n in present:
                graph[(r, c)][n] = rng.uniform(1.0, 5.0)
    return graph, {n: (float(n[1]), float(n[0])) for n in nodes}


def test_matches_dijkstra_on_a_single_query():
    rng = random.Random(0)
    graph, positions = _random_grid(8, 8, rng)
    start, goal = (0, 0), (7, 7)

    planner = DStarLite(positions=positions)
    path, stats = planner.plan(graph, start, goal)

    assert not stats.reused, "the first call has nothing to reuse"
    assert _cost(graph, path) == pytest.approx(_dijkstra_cost(graph, start, goal))


def test_matches_dijkstra_after_edge_costs_change():
    """The case the incremental machinery exists for."""
    rng = random.Random(1)
    graph, positions = _random_grid(9, 9, rng)
    start, goal = (0, 0), (8, 8)
    planner = DStarLite(positions=positions)
    planner.plan(graph, start, goal)

    for _ in range(12):
        # Perturb a handful of edges, exactly as a moving obstacle would.
        for u in rng.sample(sorted(graph), 6):
            for v in graph[u]:
                graph[u][v] = rng.uniform(1.0, 20.0)
        path, stats = planner.plan(graph, start, goal)
        assert stats.reused, "same nodes and same goal: this must repair, not rebuild"
        assert _cost(graph, path) == pytest.approx(
            _dijkstra_cost(graph, start, goal)
        ), "repaired search disagreed with Dijkstra"


def test_matches_dijkstra_as_the_start_moves():
    """km bookkeeping: a moving start must not corrupt the queue order."""
    rng = random.Random(2)
    graph, positions = _random_grid(9, 9, rng)
    goal = (8, 8)
    planner = DStarLite(positions=positions)

    path, _ = planner.plan(graph, (0, 0), goal)
    assert path
    # Walk the robot along its own path, replanning from each node in turn.
    for step in path[:5]:
        for u in rng.sample(sorted(graph), 4):
            for v in graph[u]:
                graph[u][v] = rng.uniform(1.0, 20.0)
        new_path, stats = planner.plan(graph, step, goal)
        assert stats.reused
        assert _cost(graph, new_path) == pytest.approx(_dijkstra_cost(graph, step, goal))


def test_matches_dijkstra_as_vertices_are_added_and_removed():
    """PRM and the visibility graph do this every call: vertices come and go.

    A vertex set that differs must be repaired, not rebuilt, or nothing here
    ever reuses anything. It must also still be right.
    """
    rng = random.Random(3)
    graph, positions = _random_grid(7, 7, rng, blocked=0.0)
    start, goal = (0, 0), (6, 6)
    planner = DStarLite(positions=positions)
    planner.plan(graph, start, goal)

    removed: dict = {}
    for _ in range(10):
        # Drop a vertex, as an obstacle sweeping over a milestone would.
        victim = next(
            n for n in graph if n not in (start, goal) and n not in removed
        )
        removed[victim] = {
            u: graph[u].pop(victim) for u in list(graph) if victim in graph[u]
        }
        removed[victim][victim] = graph.pop(victim)

        path, stats = planner.plan(graph, start, goal)
        assert stats.reused and stats.nodes_removed == 1
        assert _cost(graph, path) == pytest.approx(_dijkstra_cost(graph, start, goal))

        # Put it back, as the obstacle moving on would.
        edges = removed.pop(victim)
        graph[victim] = edges.pop(victim)
        for u, cost in edges.items():
            graph[u][victim] = cost

        path, stats = planner.plan(graph, start, goal)
        assert stats.reused and stats.nodes_added == 1
        assert _cost(graph, path) == pytest.approx(_dijkstra_cost(graph, start, goal))


def test_matches_dijkstra_when_the_start_is_a_new_vertex_each_call():
    """The shape every planner here actually has.

    The robot's position is spliced into the roadmap as a vertex, so the start
    is a DIFFERENT vertex every frame rather than an existing one the robot
    walked to. That is the case `km` exists for, and the case a vertex-set
    equality check would refuse to reuse.
    """
    rng = random.Random(8)
    graph, positions = _random_grid(8, 8, rng, blocked=0.0)
    goal = (7, 7)
    planner = DStarLite(positions=positions)

    previous = None
    for step in range(12):
        # A fresh start vertex, edged into the grid's first column.
        label = ("start", step)
        positions[label] = (-1.0, float(step) * 0.25)
        graph[label] = {(r, 0): 3.0 + r for r in range(4)}
        for r in range(4):
            graph[(r, 0)][label] = 3.0 + r
        if previous is not None:
            for u in list(graph):
                graph[u].pop(previous, None)
            graph.pop(previous, None)

        path, stats = planner.plan(graph, label, goal)
        if step:
            assert stats.reused, "a moving start must not force a rebuild"
        assert _cost(graph, path) == pytest.approx(_dijkstra_cost(graph, label, goal))
        previous = label


def test_reinitialises_when_the_goal_moves():
    rng = random.Random(4)
    graph, positions = _random_grid(6, 6, rng)
    planner = DStarLite(positions=positions)
    planner.plan(graph, (0, 0), (5, 5))
    _path, stats = planner.plan(graph, (0, 0), (0, 5))
    assert not stats.reused


def test_reports_unreachable_rather_than_raising():
    graph = {"a": {"b": 1.0}, "b": {"a": 1.0}, "island": {}}
    planner = DStarLite()
    path, _ = planner.plan(graph, "a", "island")
    assert path == []
    assert _dijkstra_cost(graph, "a", "island") == inf


def test_zero_heuristic_still_correct():
    """Without positions the heuristic is 0, which is admissible: Dijkstra."""
    rng = random.Random(5)
    graph, _positions = _random_grid(7, 7, rng)
    planner = DStarLite()  # no positions supplied
    path, _ = planner.plan(graph, (0, 0), (6, 6))
    assert _cost(graph, path) == pytest.approx(_dijkstra_cost(graph, (0, 0), (6, 6)))


def test_euclidean_heuristic_is_admissible_for_these_costs():
    """Guard the default: if edge costs were ever below Euclidean distance the
    heuristic would overestimate and D* Lite would return suboptimal paths."""
    rng = random.Random(6)
    graph, positions = _random_grid(7, 7, rng)
    for u, succs in graph.items():
        for v, cost in succs.items():
            straight = hypot(positions[u][0] - positions[v][0], positions[u][1] - positions[v][1])
            assert cost >= straight - 1e-9, (u, v, cost, straight)


def test_repair_expands_fewer_vertices_than_a_rebuild():
    """The reason to do any of this."""
    rng = random.Random(7)
    graph, positions = _random_grid(14, 14, rng, blocked=0.05)
    start, goal = (0, 0), (13, 13)

    planner = DStarLite(positions=positions)
    _path, first = planner.plan(graph, start, goal)

    # One edge changes, the smallest possible perturbation.
    u = next(n for n in graph if graph[n])
    v = next(iter(graph[u]))
    graph[u][v] *= 3.0
    _path, repair = planner.plan(graph, start, goal)

    assert repair.reused
    assert repair.vertices_expanded < first.vertices_expanded, (
        f"repair expanded {repair.vertices_expanded}, full search "
        f"expanded {first.vertices_expanded}"
    )
