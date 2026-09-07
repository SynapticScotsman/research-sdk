#!/usr/bin/env python3
"""How often can D* Lite actually repair, per planner backend?

All three planners now search through ``planners/search.py``, so "we use D*"
is true for all of them. That sentence is worth nothing on its own: D* Lite
only saves work when this call's graph is recognisably last call's graph, and
whether that holds is a property of the mapping, not of the search. This
measures it.

Three frame sources, because a single number cannot separate "the wiring is
broken" from "this domain does not permit reuse":

  static   The same obstacles every frame; only the robot moves. The positive
           control. If a backend cannot reuse here it cannot reuse anywhere,
           and that is a bug rather than a finding.
  drift    Every obstacle translates a fixed distance per frame. Sweeps the
           quantum boundary in `search.py`, so it says how much motion a
           roadmap tolerates before it stops being the same roadmap.
  log      A recorded SSL match. The real answer, and the only one that goes
           in the paper.

Every run is an A/B: the identical frame sequence is replayed under D* Lite
and under Dijkstra, and the two path lengths are compared frame by frame. A
faster search that returns a different path is not a faster search.

Usage:
    python scripts/measure_search_reuse.py --source static --frames 200
    python scripts/measure_search_reuse.py --source drift --drift-mm 20
    python scripts/measure_search_reuse.py --source log --log MATCH.log.gz
    python scripts/measure_search_reuse.py --selftest
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from research_sdk.planners import search
from research_sdk.planners.common import (
    DEFAULT_ROBOT_RADIUS_MM,
    Obstacle,
    PlanRequest,
    path_length_mm,
)
from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.planners.PRM import prm_dijkstra
from research_sdk.planners.VisibilityGraph import visibility_graph
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene

Point = tuple[float, float]

# The probe drives one robot from corner to corner past the obstacles, which
# is the same start/goal `dynamic_scenario.py` uses so the two are comparable.
START_MM: Point = (-3500.0, -1500.0)
GOAL_MM: Point = (3500.0, 1500.0)

# A robot at 1.5 m/s covers 21 mm per 70 Hz vision frame. Planning every
# frame is what the UI does when the reroute gate is off.
ROBOT_SPEED_MM_S = 1500.0
VISION_HZ = 70.0

STATIC_OBSTACLES: tuple[Point, ...] = (
    (-1000.0, 400.0), (500.0, -700.0), (1500.0, 900.0), (-2000.0, -1200.0),
    (2500.0, 300.0), (0.0, 1500.0), (-500.0, -1800.0), (3000.0, -900.0),
)

PLANNERS = ("visibility_graph", "prm", "voronoi")

# Set by --voronoi-placement. "density_grid" is the planner's own default and
# what every published number in this study used; "grid" pins the virtual sites
# to a fixed lattice, which is the configuration the incremental-repair argument
# actually describes.
VORONOI_PLACEMENT = "density_grid"

# Only read when VORONOI_PLACEMENT is "grid". None leaves the generator's own
# 240 mm default, which tiles the pitch with ~960 vertices and takes 4.5 s to
# build. 1200 mm is the coarsest lattice measured to resolve 420 mm gaps and
# the finest that builds inside a 50 ms control tick.
VORONOI_SPACING = None


@dataclass
class Frame:
    """One planning query: where the robot is and where everyone else is."""

    start: Point
    obstacles: tuple[Obstacle, ...]


@dataclass
class Run:
    """What one backend did over one frame sequence."""

    lengths: dict[str, list[float]] = field(default_factory=dict)
    tallies: dict[str, search.Tally] = field(default_factory=dict)
    expanded_reused: dict[str, list[int]] = field(default_factory=dict)
    expanded_fresh: dict[str, list[int]] = field(default_factory=dict)


# ------------------------------------------------------------ frame sources


def _obs(points, radius: float = DEFAULT_ROBOT_RADIUS_MM) -> tuple[Obstacle, ...]:
    return tuple(
        Obstacle(pos_mm=(float(x), float(y)), radius_mm=radius, robot_id=i, isYellow=False)
        for i, (x, y) in enumerate(points)
    )


def _shuttle(count: int) -> list[Point]:
    """Robot positions for `count` frames, shuttling between start and goal.

    It shuttles rather than parking on arrival because a robot sitting exactly
    on its goal makes the two the same roadmap vertex, and the search then
    refuses to run at all. That artefact cost 65 of 421 frames on the first
    match replay before this: a property of the probe, not of any planner.
    """
    step = ROBOT_SPEED_MM_S / VISION_HZ
    here, target = START_MM, GOAL_MM
    out = []
    for _ in range(count):
        out.append(here)
        dx, dy = target[0] - here[0], target[1] - here[1]
        d = math.hypot(dx, dy)
        if d <= step * 2.0:
            target = START_MM if target == GOAL_MM else GOAL_MM
            continue
        here = (here[0] + dx / d * step, here[1] + dy / d * step)
    return out


def frames_static(count: int) -> list[Frame]:
    """Obstacles frozen, robot advancing. The positive control."""
    return [Frame(p, _obs(STATIC_OBSTACLES)) for p in _shuttle(count)]


def frames_drift(count: int, drift_mm: float) -> list[Frame]:
    """Every obstacle translating by the same vector each frame.

    A uniform translation is the cleanest possible perturbation: the roadmap's
    shape is preserved exactly, only its position changes, so any drop in
    reuse is attributable to vertex identity alone rather than to the roadmap
    genuinely becoming a different roadmap.
    """
    return [
        Frame(p, _obs((x + drift_mm * i, y + drift_mm * i * 0.5)
                      for x, y in STATIC_OBSTACLES))
        for i, p in enumerate(_shuttle(count))
    ]


def frames_from_log(path: str, seconds: float, skip_seconds: float) -> list[Frame]:
    """Real match motion, replayed through the same vision pipeline the UI uses."""
    from measure_graph_churn import capture_from_logfile

    captures = capture_from_logfile(path, seconds=seconds, skip_seconds=skip_seconds)
    return [
        Frame(p, tuple(cap.obstacles))
        for cap, p in zip(captures, _shuttle(len(captures)))
    ]


# ------------------------------------------------------------- the three calls


def _scene(obstacles: tuple[Obstacle, ...]) -> PlanningScene:
    return PlanningScene(
        timestamp=0.0,
        obstacles=tuple(
            PlanningObstacle(
                robot_id=o.robot_id, isYellow=o.isYellow,
                pos_mm=o.pos_mm, radius_mm=o.radius_mm,
            )
            for o in obstacles
        ),
        field=FieldDimensions(),
    )


def plan_all(frame: Frame, key) -> dict[str, float]:
    """Plan one frame with all three backends; return path length or nan.

    ``skip_direct_path=True`` throughout: without it a clear line of sight
    returns before the search runs at all, and a reuse rate measured over
    calls that never searched means nothing.
    """
    request = PlanRequest(start_mm=frame.start, goal_mm=GOAL_MM, obstacles=frame.obstacles)
    out: dict[str, float] = {}

    vg = visibility_graph.plan(request, skip_direct_path=True, search_key=key)
    out["visibility_graph"] = vg.path_length_mm if vg.success else math.nan

    prm = prm_dijkstra.plan(request, skip_direct_path=True, search_key=key)
    out["prm"] = prm.path_length_mm if prm.success else math.nan

    result = VoronoiDijkstraPlanner(
        placement_mode=VORONOI_PLACEMENT, grid_spacing_mm=VORONOI_SPACING
    ).plan(
        _scene(frame.obstacles), frame.start, GOAL_MM,
        skip_direct_path=True, search_key=key,
    )
    # Voronoi returns waypoints excluding the start, so splice it back on
    # before measuring, or its paths read shorter than the other two.
    out["voronoi"] = (
        path_length_mm((frame.start, *result.waypoints_mm))
        if result.waypoints_mm
        else math.nan
    )
    return out


def run(frames: list[Frame], backend: str) -> Run:
    """Replay every frame under one search backend."""
    result = Run()
    with search.use_backend(backend):
        search.reset_tallies()
        for name in PLANNERS:
            result.lengths[name] = []
            result.expanded_reused[name] = []
            result.expanded_fresh[name] = []
        before = {name: search.Tally() for name in PLANNERS}
        for frame in frames:
            lengths = plan_all(frame, key=("probe", 0))
            now = search.tallies()
            for name in PLANNERS:
                result.lengths[name].append(lengths[name])
                tally = now.get(name)
                if tally is None:
                    continue
                # Difference against the previous frame isolates this frame's
                # calls; the tally itself is cumulative.
                searched = tally.searched - before[name].searched
                reused = tally.reused - before[name].reused
                expanded = tally.vertices_expanded - before[name].vertices_expanded
                if searched:
                    bucket = result.expanded_reused if reused else result.expanded_fresh
                    bucket[name].append(expanded)
                before[name] = search.Tally(
                    searched=tally.searched, reused=tally.reused,
                    vertices_expanded=tally.vertices_expanded,
                )
        result.tallies = search.tallies()
    return result


# ------------------------------------------------------------------ reporting


def _mean(values) -> float:
    return statistics.fmean(values) if values else math.nan


def report(frames: list[Frame], label: str, quantum_mm: float, repeats: int = 1) -> dict:
    # Alternate the two backends rather than running each once. Sub-millisecond
    # search times on a loaded machine drift by more than the effect being
    # measured: a single pass put D* Lite 24% slower on the visibility graph
    # and, on the next run of the same code, 8% faster. Alternating and taking
    # the median puts both backends under the same drift.
    dstar_ms: dict[str, list[float]] = {n: [] for n in PLANNERS}
    dijkstra_ms: dict[str, list[float]] = {n: [] for n in PLANNERS}
    for _ in range(repeats):
        dstar = run(frames, "dstar")
        dijkstra = run(frames, "dijkstra")
        for name in PLANNERS:
            dstar_ms[name].append(dstar.tallies.get(name, search.Tally()).mean_ms)
            dijkstra_ms[name].append(dijkstra.tallies.get(name, search.Tally()).mean_ms)

    print(f"\n{label}: {len(frames)} frames, quantum {quantum_mm:.0f} mm, "
          f"{repeats} repeat(s), voronoi sites {VORONOI_PLACEMENT}"
          f"{'' if VORONOI_SPACING is None else f' @ {VORONOI_SPACING:.0f} mm'}\n")
    header = (
        f"{'planner':<18}{'searches':>9}{'reuse':>8}{'nodes':>8}{'churn':>8}{'chg/fr':>8}"
        f"{'exp/reuse':>11}{'exp/fresh':>11}{'D* ms':>9}{'Dij ms':>9}"
        f"{'collide':>9}{'len agree':>11}{'max d mm':>10}"
    )
    print(header)
    print("-" * len(header))

    rows = {}
    for name in PLANNERS:
        d = dstar.tallies.get(name, search.Tally())

        pairs = [
            (a, b)
            for a, b in zip(dstar.lengths[name], dijkstra.lengths[name])
            if not (math.isnan(a) or math.isnan(b))
        ]
        deltas = [abs(a - b) for a, b in pairs]
        agree = sum(1 for x in deltas if x <= 1e-6)
        max_delta = max(deltas) if deltas else 0.0

        rows[name] = {
            "searches": d.searched,
            "calls": d.calls,
            "reuse_rate": d.reuse_rate,
            "mean_nodes": d.mean_nodes,
            "churn_rate": d.churn_rate,
            "changed_per_call": d.changed_per_call,
            "expanded_when_reused": _mean(dstar.expanded_reused[name]),
            "expanded_when_fresh": _mean(dstar.expanded_fresh[name]),
            "dstar_ms": statistics.median(dstar_ms[name]),
            "dijkstra_ms": statistics.median(dijkstra_ms[name]),
            "dstar_ms_all": dstar_ms[name],
            "dijkstra_ms_all": dijkstra_ms[name],
            "compared": len(pairs),
            "agreeing": agree,
            "max_delta_mm": max_delta,
            "collisions": d.collisions,
        }
        print(
            f"{name:<18}{d.searched:>9}{d.reuse_rate * 100:>7.1f}%{d.mean_nodes:>8.1f}"
            f"{d.churn_rate * 100:>7.1f}%{d.changed_per_call:>8.1f}"
            f"{_mean(dstar.expanded_reused[name]):>11.1f}"
            f"{_mean(dstar.expanded_fresh[name]):>11.1f}"
            f"{statistics.median(dstar_ms[name]):>9.3f}"
            f"{statistics.median(dijkstra_ms[name]):>9.3f}{d.collisions:>9}"
            f"{agree:>6}/{len(pairs):<4}{max_delta:>10.4f}"
        )

    print(
        "\nsearches = calls that reached the search. Fewer than the frame count\n"
        "           when an endpoint was not in the graph at all.\n"
        "reuse    = share of those searches that repaired the previous tree.\n"
        "churn    = share of the vertex set replaced per call; chg/fr is the\n"
        "           same quantity as a count. Read them together. A ratio falls\n"
        "           when its denominator grows, which is not stabilisation: a\n"
        "           900 mm fixed lattice cuts churn from 116% to 54% while the\n"
        "           count of vertices actually changing rises, 47.9 to 64.2.\n"
        "exp/*    = vertices expanded, split by whether that call repaired.\n"
        "collide  = calls refused reuse because two roadmap vertices landed in\n"
        "           the same quantum cell; those fall back to plain Dijkstra.\n"
        "len agree= frames where D* Lite and Dijkstra returned the same path\n"
        "           length to within 1e-6 mm. Anything less than all of them\n"
        "           is a correctness bug, not a tuning issue."
    )
    return rows


# ------------------------------------------------------------------- selftest


def selftest() -> int:
    """Pin the two properties the measurement is worthless without."""
    failures = []

    frames = frames_static(12)
    rows = report(frames, "SELFTEST static control", search.DEFAULT_QUANTUM_MM, repeats=2)

    for name, row in rows.items():
        if row["compared"] and row["agreeing"] != row["compared"]:
            failures.append(
                f"{name}: D* Lite and Dijkstra disagreed on "
                f"{row['compared'] - row['agreeing']}/{row['compared']} frames "
                f"(max {row['max_delta_mm']:.4f} mm)"
            )
        if row["searches"] == 0:
            failures.append(f"{name}: no search calls, so nothing was measured")

    # The control exists to prove reuse is reachable at all. A frozen scene
    # with only the robot moving must let at least one backend repair.
    if rows and max(r["reuse_rate"] for r in rows.values()) == 0.0:
        failures.append(
            "no backend reused anything on a frozen scene: the wiring is "
            "broken, not the domain"
        )

    print()
    for line in failures:
        print(f"FAIL {line}")
    if not failures:
        print("selftest OK")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", choices=("static", "drift", "log"), default="static")
    parser.add_argument("--frames", type=int, default=200)
    parser.add_argument("--drift-mm", type=float, default=20.0,
                        help="Per-frame obstacle translation for --source drift")
    parser.add_argument("--log", help="SSL match log (.log or .log.gz) for --source log")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--skip", type=float, default=180.0,
                        help="Seconds of log to skip before capturing")
    parser.add_argument("--quantum-mm", type=float, default=search.DEFAULT_QUANTUM_MM)
    parser.add_argument("--voronoi-placement", default="density_grid",
                        choices=("density_grid", "grid", "random"),
                        help="Where Voronoi puts its virtual sites. Only 'grid' "
                             "gives the fixed backbone the reuse argument assumes.")
    parser.add_argument("--voronoi-spacing", type=float, default=None,
                        help="Lattice spacing in mm for --voronoi-placement grid. "
                             "The generator defaults to 240 mm, which is unusable.")
    parser.add_argument("--repeats", type=int, default=1,
                        help="Alternating A/B passes; the timing columns are medians")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    global VORONOI_PLACEMENT, VORONOI_SPACING
    VORONOI_PLACEMENT = args.voronoi_placement
    VORONOI_SPACING = args.voronoi_spacing
    search.DEFAULT_QUANTUM_MM = args.quantum_mm

    if args.selftest:
        return selftest()

    if args.source == "static":
        frames = frames_static(args.frames)
        label = "Static control (obstacles frozen, robot moving)"
    elif args.source == "drift":
        frames = frames_drift(args.frames, args.drift_mm)
        label = f"Uniform drift, {args.drift_mm:.0f} mm per frame"
    else:
        if not args.log:
            parser.error("--source log needs --log PATH")
        frames = frames_from_log(args.log, args.seconds, args.skip)
        label = f"Real match, {Path(args.log).name}"

    report(frames, label, args.quantum_mm, repeats=args.repeats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
