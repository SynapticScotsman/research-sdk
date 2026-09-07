"""The one search every planner backend calls, and the Dijkstra/D* Lite switch.

PRM, the visibility graph and Voronoi differ in how they turn a scene into a
graph. They must not differ in how they search it, or a difference in path
quality cannot be attributed to the mapping. All three therefore call
:func:`shortest_path` here rather than ``nx.dijkstra_path`` directly.

## Why node labels are rewritten before searching

Each backend labels its vertices its own way: the visibility graph uses
``"o3v5"``, PRM uses an index into a resampled array, Voronoi uses an index
into a freshly built tessellation. None of those labels mean the same *point*
from one call to the next. That does not affect Dijkstra, which starts from
nothing every time, but it is the whole ballgame for D* Lite, which only saves
work when this call's graph is recognisably last call's graph.

So this module relabels every vertex by its quantised position
(:data:`DEFAULT_QUANTUM_MM`) before handing the graph to the search. Two
consequences follow, and both are deliberate:

  * A vertex that moved less than the quantum keeps its label, so the search
    tree over it can be repaired instead of rebuilt.
  * A vertex that moved further gets a new label, the vertex set differs, and
    D* Lite reinitialises. That is the correct answer, not a failure: a
    different vertex set is a different search problem.

The quantum is therefore the knob that says how much geometric drift still
counts as "the same roadmap". Reuse rates are only meaningful quoted with it.

## Why the heuristic is computed on the grid, not on the true positions

D* Lite's ``km`` bookkeeping assumes the heuristic does not change between
calls. If ``h`` were computed from the exact vertex positions it would drift
every frame (the points move within their quantum) and the priority ordering
could silently go wrong, returning suboptimal paths with no error at all.
Computing ``h`` from the quantised label makes it a pure function of the label
and so constant by construction. The price is up to ``quantum * sqrt(2)`` of
overestimate, which :data:`_ADMISSIBILITY_SLACK` subtracts back off so the
heuristic stays admissible and the paths stay optimal.

## What this is expected to save

Measured on an eight-obstacle scene, roadmap construction is 98.0% of a
visibility-graph call (29.94 ms of 30.54 ms) and 92.8% of a Voronoi call
(7.67 ms of 8.26 ms). Search is the remaining 2.0% and 7.2%. No search
algorithm can save more of a planning call than that, so the tally below
reports reuse rate per backend rather than a headline speedup: how often
incremental search is even applicable is the answerable question here.
"""

from __future__ import annotations

import time
from collections.abc import Hashable, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from math import hypot, sqrt

import networkx as nx

from research_sdk.planners.dstar_lite import DStarLite, SearchStats

Node = Hashable
Point2D = tuple[float, float]

# Grid spacing for the relabelling described above. 5 mm is well under the
# ~20 mm/frame a 1.5 m/s SSL robot moves at 70 Hz, and orders of magnitude
# under the spacing of roadmap vertices, so distinct vertices never collide.
DEFAULT_QUANTUM_MM = 5.0

# A point sits at most quantum/2 from its grid label in each axis, so a
# distance measured between two labels differs from the true distance by at
# most quantum*sqrt(2). Subtracting that keeps the heuristic admissible.
_ADMISSIBILITY_SLACK = sqrt(2.0)

_DSTAR = "dstar"
_DIJKSTRA = "dijkstra"
_backend = _DSTAR

# Persistent search state, keyed by (backend name, caller's key). The Voronoi
# waypoint manager builds a fresh VoronoiDijkstraPlanner on every reroute, so
# this cannot live on the planner object and still survive between calls.
_SEARCHERS: dict[tuple[str, Hashable], DStarLite] = {}


@dataclass
class Tally:
    """Running totals for one backend. Read with :func:`tallies`."""

    calls: int = 0
    searched: int = 0
    """Calls that reached the incremental search. Smaller than `calls`
    whenever an endpoint was missing from the graph or the labels were
    ambiguous, and the reuse rate is only meaningful over these."""

    reused: int = 0
    vertices_expanded: int = 0
    edges_changed: int = 0
    nodes_total: int = 0
    nodes_added: int = 0
    nodes_removed: int = 0
    seconds: float = 0.0
    collisions: int = 0
    """Calls where two vertices shared a grid cell, so reuse was refused."""

    @property
    def reuse_rate(self) -> float:
        return self.reused / self.searched if self.searched else 0.0

    @property
    def mean_ms(self) -> float:
        return self.seconds * 1000.0 / self.calls if self.calls else 0.0

    @property
    def mean_nodes(self) -> float:
        return self.nodes_total / self.calls if self.calls else 0.0

    @property
    def changed_per_call(self) -> float:
        """Vertices added or removed per call, as a count.

        Report this beside `churn_rate`, never instead of it, and never the
        ratio alone. Churn is a ratio, so a configuration that adds stable
        vertices around an equally unstable core lowers it without stabilising
        anything. Measured on a real match: a fixed 900 mm lattice cut churn
        from 116% to 54% while the count of vertices actually changing went UP,
        from 47.9 to 64.2 per frame.
        """
        return (self.nodes_added + self.nodes_removed) / self.calls if self.calls else 0.0

    @property
    def churn_rate(self) -> float:
        """Share of the vertex set replaced per call.

        The number that decides whether repairing can beat rebuilding: at
        churn near 1.0 the repair has to touch every vertex anyway.
        """
        return (self.nodes_added + self.nodes_removed) / self.nodes_total if self.nodes_total else 0.0


_TALLIES: dict[str, Tally] = {}


def tallies() -> dict[str, Tally]:
    """Per-backend totals since the last :func:`reset_tallies`."""
    return dict(_TALLIES)


def reset_tallies() -> None:
    _TALLIES.clear()


def reset_searchers() -> None:
    """Forget every persistent search tree.

    Call between independent trials so one trial's roadmap cannot be reused
    by the next, which would flatter the reuse rate.
    """
    _SEARCHERS.clear()


def current_backend() -> str:
    return _backend


@contextmanager
def use_backend(name: str):
    """Run a block with ``"dstar"`` or ``"dijkstra"`` as the search.

    A context manager rather than a ``plan()`` argument because the switch has
    to reach three call sites buried inside three modules, and every A/B wants
    it flipped for a whole trial rather than per call.
    """
    global _backend
    if name not in (_DSTAR, _DIJKSTRA):
        raise ValueError(f"unknown search backend {name!r}")
    previous, _backend = _backend, name
    # Old trees were built under the other backend's bookkeeping; drop them so
    # a switch cannot half-reuse across the boundary.
    reset_searchers()
    try:
        yield
    finally:
        _backend = previous
        reset_searchers()


def _grid_heuristic(quantum: float):
    slack = quantum * _ADMISSIBILITY_SLACK

    def h(a: Node, b: Node) -> float:
        # Labels ARE grid coordinates, so this needs no position lookup and
        # cannot drift as the underlying points move.
        d = hypot((a[0] - b[0]) * quantum, (a[1] - b[1]) * quantum)
        return max(0.0, d - slack)

    return h


def _record(
    planner: str, stats: SearchStats, seconds: float, collided: bool,
    searched: bool = False,
) -> None:
    tally = _TALLIES.setdefault(planner, Tally())
    tally.calls += 1
    tally.searched += int(searched)
    tally.reused += int(stats.reused)
    tally.vertices_expanded += stats.vertices_expanded
    tally.edges_changed += stats.edges_changed
    tally.nodes_total += stats.nodes_total
    tally.nodes_added += stats.nodes_added
    tally.nodes_removed += stats.nodes_removed
    tally.seconds += seconds
    tally.collisions += int(collided)


def _dijkstra_path(neighbours, start, goal) -> list[Node]:
    graph = nx.Graph()
    graph.add_nodes_from(neighbours)
    for u, succs in neighbours.items():
        for v, cost in succs.items():
            graph.add_edge(u, v, weight=cost)
    try:
        return nx.dijkstra_path(graph, start, goal, weight="weight")
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return []


def shortest_path(
    neighbours: Mapping[Node, Mapping[Node, float]],
    positions: Mapping[Node, Point2D],
    start: Node,
    goal: Node,
    *,
    planner: str,
    key: Hashable = None,
    quantum_mm: float | None = None,
) -> tuple[list[Node], SearchStats]:
    """Shortest path from ``start`` to ``goal``, in the caller's own labels.

    ``key`` identifies which robot's search tree to repair; pass the robot key
    so two robots planning in the same frame do not force each other to
    reinitialise. Returns ``[]`` when the goal is unreachable, which is what
    ``nx.dijkstra_path`` raising ``NetworkXNoPath`` used to mean at each of
    the three call sites.
    """
    # Resolved here rather than as a default argument so a caller can move
    # DEFAULT_QUANTUM_MM and have it take effect; defaults bind at def time.
    quantum_mm = DEFAULT_QUANTUM_MM if quantum_mm is None else quantum_mm
    t0 = time.perf_counter()

    if start not in neighbours or goal not in neighbours:
        stats = SearchStats(False, 0, 0, len(neighbours))
        _record(planner, stats, time.perf_counter() - t0, False)
        return [], stats

    if _backend == _DIJKSTRA:
        # The A/B baseline. Deliberately skips the relabelling below: that
        # cost belongs to D* Lite, and charging it to both would hide it.
        path = _dijkstra_path(neighbours, start, goal)
        stats = SearchStats(False, 0, 0, len(neighbours))
        _record(planner, stats, time.perf_counter() - t0, False)
        return path, stats

    canon: dict[Node, tuple[int, int]] = {}
    seen: dict[tuple[int, int], Node] = {}
    collided = False
    for node in neighbours:
        pos = positions.get(node)
        if pos is None:
            # Without geometry the relabelling is impossible and the reuse
            # argument collapses with it. Refuse rather than guess.
            collided = True
            break
        cell = (round(pos[0] / quantum_mm), round(pos[1] / quantum_mm))
        if seen.setdefault(cell, node) != node:
            # Two distinct vertices in one cell would merge into one and
            # silently create a shortcut edge that the roadmap never had.
            collided = True
            break
        canon[node] = cell

    if collided:
        # Fall back to a fresh exact search. Dropping the tree first is what
        # keeps this safe: a tree indexed by the other labelling must never be
        # repaired against this call's graph.
        _SEARCHERS.pop((planner, key), None)
        path = _dijkstra_path(neighbours, start, goal)
        stats = SearchStats(False, 0, 0, len(neighbours))
        _record(planner, stats, time.perf_counter() - t0, True)
        return path, stats

    canon_graph = {
        canon[u]: {canon[v]: float(cost) for v, cost in succs.items()}
        for u, succs in neighbours.items()
    }
    back = {cell: node for node, cell in canon.items()}

    searcher = _SEARCHERS.get((planner, key))
    if searcher is None:
        searcher = DStarLite(heuristic=_grid_heuristic(quantum_mm))
        _SEARCHERS[(planner, key)] = searcher

    cells, stats = searcher.plan(canon_graph, canon[start], canon[goal])
    _record(planner, stats, time.perf_counter() - t0, False, searched=True)
    return [back[c] for c in cells], stats


def shortest_path_nx(
    graph: nx.Graph,
    positions: Mapping[Node, Point2D],
    start: Node,
    goal: Node,
    *,
    planner: str,
    key: Hashable = None,
    weight: str = "weight",
    quantum_mm: float | None = None,
) -> tuple[list[Node], SearchStats]:
    """:func:`shortest_path` for callers that already hold a ``networkx`` graph."""
    neighbours = {
        u: {v: float(data[weight]) for v, data in succs.items()}
        for u, succs in graph.adj.items()
    }
    return shortest_path(
        neighbours,
        positions,
        start,
        goal,
        planner=planner,
        key=key,
        quantum_mm=quantum_mm,
    )
