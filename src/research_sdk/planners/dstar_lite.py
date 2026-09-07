"""D* Lite: incremental heuristic search, shared by every planner backend.

Koenig, S. and Likhachev, M. "D* Lite". AAAI 2002, 476-483. The algorithm
below follows their Figure 3 (the optimised version) with the standard lazy
priority queue: `heapq` cannot remove an arbitrary entry, so stale entries are
pushed and skipped on pop instead.

## Why this exists

Every backend in this package (PRM, visibility graph, Voronoi) builds a
roadmap and then searches it. Holding the SEARCH constant across all three is
what makes the comparison about the mapping rather than about the search, so
all three call into this module.

## What it can and cannot save

D* Lite repairs a previous search instead of redoing it, which pays only when
the graph it is handed is substantially the same graph as last time. Two
measured facts about this domain bound that:

  * Search is a small fraction of a planning call. Measured on an
    eight-obstacle scene: roadmap construction is 98.0% of a visibility-graph
    call (29.94 ms of 30.54 ms) and 92.8% of a Voronoi call (7.67 ms of
    8.26 ms). Reusing the search therefore cannot save more than 2.0% and 7.2%
    of those calls respectively, however well it is implemented.
  * Reuse requires stable vertices. A visibility graph's vertices ARE the
    inflated obstacle corners, so any obstacle motion replaces them and there
    is no previous search to repair. A Voronoi roadmap on a fixed site
    backbone keeps 70.9% of its edges when all obstacles move 500 mm.

So this module is instrumented rather than merely used: every call reports
whether it repaired or reinitialised, and how many vertices it expanded. That
turns "we used D*" into a measurement of how often incremental search is
applicable per backend, which is the honest claim available here.
"""

from __future__ import annotations

import heapq
from collections.abc import Callable, Hashable, Mapping
from dataclasses import dataclass, field
from itertools import pairwise
from math import hypot, inf

Node = Hashable
Point2D = tuple[float, float]
Graph = Mapping[Node, Mapping[Node, float]]

# Priority keys are compared as pairs; this is the "infinite" key.
_INF_KEY = (inf, inf)


@dataclass(frozen=True, slots=True)
class SearchStats:
    """What this call actually did. The point of instrumenting the search."""

    reused: bool
    """True when the previous search tree was repaired rather than rebuilt."""

    vertices_expanded: int
    """Pops from the queue that changed a g-value. The work D* Lite does."""

    edges_changed: int
    """Directed edges whose cost differed from the previous call."""

    nodes_total: int

    nodes_added: int = 0
    """Vertices this call has that the previous one did not."""

    nodes_removed: int = 0
    """Vertices the previous call had that this one does not. Together with
    `nodes_added` and `nodes_total` this says how much of the roadmap actually
    survived, which is what decides whether repairing can beat rebuilding."""


@dataclass
class DStarLite:
    """Persistent D* Lite state for one robot.

    Kept per robot because the search is rooted at that robot's goal and
    tracks its start as it moves; sharing one instance across robots would
    force a reinitialise on every call and defeat the purpose.
    """

    heuristic: Callable[[Node, Node], float] | None = None
    """Admissible estimate between two nodes. Defaults to Euclidean distance
    over `positions`, which is admissible when edge costs are lengths."""

    positions: dict[Node, Point2D] = field(default_factory=dict)

    _g: dict[Node, float] = field(default_factory=dict, repr=False)
    _rhs: dict[Node, float] = field(default_factory=dict, repr=False)
    _queue: list = field(default_factory=list, repr=False)
    _queue_keys: dict[Node, tuple[float, float]] = field(default_factory=dict, repr=False)
    _km: float = 0.0
    _start: Node | None = None
    _goal: Node | None = None
    _last_start: Node | None = None
    _graph: dict[Node, dict[Node, float]] = field(default_factory=dict, repr=False)

    # ---------------------------------------------------------------- helpers

    def _h(self, a: Node, b: Node) -> float:
        if self.heuristic is not None:
            return self.heuristic(a, b)
        pa, pb = self.positions.get(a), self.positions.get(b)
        if pa is None or pb is None:
            # No geometry available: fall back to Dijkstra behaviour. Zero is
            # always admissible, it just stops the search being focused.
            return 0.0
        return hypot(pa[0] - pb[0], pa[1] - pb[1])

    def _key(self, s: Node) -> tuple[float, float]:
        k2 = min(self._g.get(s, inf), self._rhs.get(s, inf))
        if k2 == inf:
            return _INF_KEY
        return (k2 + self._h(self._start, s) + self._km, k2)

    def _push(self, s: Node) -> None:
        key = self._key(s)
        self._queue_keys[s] = key
        heapq.heappush(self._queue, (key, s))

    def _top_key(self) -> tuple[float, float]:
        self._drop_stale()
        return self._queue[0][0] if self._queue else _INF_KEY

    def _drop_stale(self) -> None:
        """Discard queue entries superseded by a later push, or removed."""
        while self._queue:
            key, s = self._queue[0]
            current = self._queue_keys.get(s)
            if current is None or current != key:
                heapq.heappop(self._queue)
                continue
            return

    def _update_vertex(self, u: Node) -> None:
        if u != self._goal:
            best = inf
            for succ, cost in self._graph.get(u, {}).items():
                best = min(best, cost + self._g.get(succ, inf))
            self._rhs[u] = best
        self._queue_keys.pop(u, None)
        if self._g.get(u, inf) != self._rhs.get(u, inf):
            self._push(u)

    def _initialize(self, graph: Graph, start: Node, goal: Node) -> None:
        self._graph = {u: dict(vs) for u, vs in graph.items()}
        self._queue = []
        self._queue_keys = {}
        self._km = 0.0
        self._g = dict.fromkeys(self._graph, inf)
        self._rhs = dict.fromkeys(self._graph, inf)
        self._start, self._goal, self._last_start = start, goal, start
        self._rhs[goal] = 0.0
        self._push(goal)

    # ------------------------------------------------------------ the search

    def _compute_shortest_path(self) -> int:
        expanded = 0
        # The second clause is what makes this terminate with a correct
        # start value rather than merely an empty queue.
        while self._queue and (
            self._top_key() < self._key(self._start)
            or self._rhs.get(self._start, inf) != self._g.get(self._start, inf)
        ):
            self._drop_stale()
            if not self._queue:
                break
            k_old, u = heapq.heappop(self._queue)
            if self._queue_keys.get(u) != k_old:
                continue
            del self._queue_keys[u]

            k_new = self._key(u)
            if k_old < k_new:
                # Key was raised while queued: reinsert at its true priority.
                self._push(u)
            elif self._g.get(u, inf) > self._rhs.get(u, inf):
                # Overconsistent: accept the improvement and propagate.
                self._g[u] = self._rhs[u]
                expanded += 1
                for pred in self._graph.get(u, {}):
                    self._update_vertex(pred)
            else:
                # Underconsistent: invalidate and let predecessors recompute.
                self._g[u] = inf
                expanded += 1
                for pred in self._graph.get(u, {}):
                    self._update_vertex(pred)
                self._update_vertex(u)
        return expanded

    def _changed_edges(self, graph: Graph) -> list[tuple[Node, Node, float]]:
        """Every directed edge whose cost differs, over the UNION of node sets.

        Iterating only `graph` would miss the outgoing edges of a vertex that
        has disappeared since the last call, leaving g-values that depend on a
        vertex the caller no longer has.
        """
        changed: list[tuple[Node, Node, float]] = []
        for u in graph.keys() | self._graph.keys():
            new = graph.get(u, {})
            old = self._graph.get(u, {})
            for v, cost in new.items():
                if old.get(v) != cost:
                    changed.append((u, v, cost))
            for v in old:
                if v not in new:
                    changed.append((u, v, inf))
        return changed

    # ------------------------------------------------------------------- api

    def plan(self, graph: Graph, start: Node, goal: Node) -> tuple[list[Node], SearchStats]:
        """Shortest path start -> goal, repairing the previous search if possible.

        Returns the node path (empty when unreachable) and what the call did.

        A changed vertex set does NOT force a rebuild. An added vertex enters
        locally consistent (g = rhs = inf, so it needs no queue entry until an
        edge gives it a finite rhs) and a removed vertex is its edges going to
        inf, so both reduce to the edge-cost changes Koenig and Likhachev's
        UpdateVertex already handles. This matters because the start IS a
        vertex of the roadmap in all three planners here: it is spliced in
        fresh every call, so the vertex set differs by at least one node every
        single frame. Rebuilding on that would mean never reusing anything,
        which the frozen-scene control in `scripts/measure_search_reuse.py`
        caught: 0% reuse on a scene where nothing but the robot moved.

        Only a changed GOAL forces a reinitialise, because the goal is the
        root the whole g-field is measured from.
        """
        can_reuse = self._goal is not None and goal == self._goal

        if not can_reuse:
            self._initialize(graph, start, goal)
            edges_changed = 0
            added = removed = 0
        else:
            changed = self._changed_edges(graph)
            edges_changed = len(changed)
            added_nodes = graph.keys() - self._graph.keys()
            removed_nodes = self._graph.keys() - graph.keys()
            added, removed = len(added_nodes), len(removed_nodes)

            # Moving the start would invalidate every key; km absorbs that so
            # the queue stays sorted. Koenig and Likhachev, Figure 3.
            if start != self._start:
                self._km += self._h(self._last_start, start)
                self._last_start = start
                self._start = start

            # New vertices start locally consistent so they need no queue
            # entry of their own; their edges below are what wakes them.
            for u in added_nodes:
                self._g[u] = inf
                self._rhs[u] = inf

            # Replace wholesale rather than applying the diff twice: the diff
            # is already O(E), and a stored graph that drifted out of step
            # with the caller's would be a silent wrong-answer bug.
            self._graph = {u: dict(vs) for u, vs in graph.items()}

            for u in removed_nodes:
                self._g.pop(u, None)
                self._rhs.pop(u, None)
                # Its queue entries become stale and are dropped on the next
                # pop; see `_drop_stale`.
                self._queue_keys.pop(u, None)

            touched = {u for u, _v, _c in changed if u in self._graph}
            touched |= {v for _u, v, _c in changed if v in self._graph}
            for u in touched:
                self._update_vertex(u)

        self._start = start
        expanded = self._compute_shortest_path()

        path = self._extract_path()
        return path, SearchStats(
            reused=can_reuse,
            vertices_expanded=expanded,
            edges_changed=edges_changed,
            nodes_total=len(self._graph),
            nodes_added=added,
            nodes_removed=removed,
        )

    def _extract_path(self) -> list[Node]:
        """Walk greedily from start to goal down the g-values.

        The search is rooted at the goal, so g(s) is cost-to-goal and the
        successor minimising c(s, s') + g(s') is the next step.
        """
        if self._start is None or self._goal is None:
            return []
        if self._g.get(self._start, inf) == inf:
            return []
        path = [self._start]
        current = self._start
        # A correct g-field is acyclic toward the goal; the bound only guards
        # against a malformed graph rather than expected behaviour.
        for _ in range(len(self._graph) + 1):
            if current == self._goal:
                return path
            best, best_cost = None, inf
            for succ, cost in self._graph.get(current, {}).items():
                candidate = cost + self._g.get(succ, inf)
                if candidate < best_cost:
                    best, best_cost = succ, candidate
            if best is None or best_cost == inf:
                return []
            path.append(best)
            current = best
        return []


def path_cost(graph: Graph, path: list[Node]) -> float:
    """Total edge cost along `path`, or inf if it uses a missing edge."""
    total = 0.0
    for u, v in pairwise(path):
        cost = graph.get(u, {}).get(v)
        if cost is None:
            return inf
        total += cost
    return total
