#!/usr/bin/env python3
"""Paired planner comparison stratified by obstacle density -- the paper table.

The existing ``benchmark_planners.py`` times all three planners over the two
saved scenario files. Both of those are 3 robots / 9 obstacles, so its output
is a single difficulty point: it answers "which is faster here" and cannot
answer "which degrades as the field fills up", which is the question a planner
comparison is actually for.

This sweeps obstacle count and reports, per density:

  success        fraction of instances solved            (anchor: 1.00 = always)
  length ratio   path length / straight-line distance    (anchor: 1.00 = optimal
                 unobstructed path; a planner cannot do better, so this is a
                 true lower bound, not a normalisation)
  time           per-call planning latency, ms
  clearance      closest approach of the path to any obstacle SURFACE, mm
                 (anchor: 0 = grazing; negative = the path passes through an
                 obstacle. Required margin is robot_radius + clearance_mm.)

Every planner sees the SAME instance at each (density, seed), so differences
are paired -- the comparison is not confounded by one planner drawing easier
random layouts than another.

Start and goal are fixed at the two ends of the long axis so the length-ratio
denominator is constant across every instance; only the obstacle field is
randomised.

Usage:
    python scripts/benchmark_planner_sweep.py
    python scripts/benchmark_planner_sweep.py --instances 50
    python scripts/benchmark_planner_sweep.py --selftest
"""

from __future__ import annotations

import argparse
import random
import sys
from itertools import pairwise
from math import dist, hypot
from pathlib import Path
from statistics import median

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from demo_planners import PLANNER_NAMES, run_all

from research_sdk.planners.common import (
    DEFAULT_ROBOT_RADIUS_MM,
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    Obstacle,
    PlanRequest,
)

START_MM = (-4000.0, 0.0)
GOAL_MM = (4000.0, 0.0)
STRAIGHT_LINE_MM = dist(START_MM, GOAL_MM)

DENSITIES = (0, 3, 6, 9, 12, 15)

# The shipping constraint. ExecutionController drives control on a 50 ms Qt
# timer (ui/execution/page.py:380, setInterval(50)), so a planning call has to
# finish inside that tick -- NOT the 16 ms figure the VisibilityGraph module
# docstring cites (visibility_graph.py:33), which is the SSL vision frame
# budget, a different thing. ResearchRuntime.plan() loops robots serially, so
# the whole team's planning shares one tick.
CONTROL_TICK_MS = 50.0
TEAM_ROBOTS = 6  # Division B team size

# Obstacles are kept this far from start and goal so no instance is unsolvable
# by construction -- an obstacle sitting on the goal would score as a planner
# failure when it is really a broken instance.
ENDPOINT_KEEPOUT_MM = 400.0


def random_obstacles(count: int, rng: random.Random) -> tuple[Obstacle, ...]:
    """`count` non-overlapping robot-sized obstacles, clear of start and goal."""
    placed: list[Obstacle] = []
    # Rejection sampling. Attempt cap keeps a too-dense request from spinning
    # forever; at the densities here it is never reached (asserted in selftest).
    for _ in range(count * 200):
        if len(placed) == count:
            break
        x = rng.uniform(-FIELD_LENGTH_MM / 2 + 500, FIELD_LENGTH_MM / 2 - 500)
        y = rng.uniform(-FIELD_WIDTH_MM / 2 + 500, FIELD_WIDTH_MM / 2 - 500)
        if min(dist((x, y), START_MM), dist((x, y), GOAL_MM)) < ENDPOINT_KEEPOUT_MM:
            continue
        if any(dist((x, y), o.pos_mm) < 3 * DEFAULT_ROBOT_RADIUS_MM for o in placed):
            continue
        placed.append(Obstacle(pos_mm=(x, y), radius_mm=DEFAULT_ROBOT_RADIUS_MM, robot_id=len(placed)))
    return tuple(placed)


def min_clearance_mm(waypoints, obstacles, step_mm: float = 20.0) -> float:
    """Closest approach of the polyline to any obstacle surface.

    Samples along each segment rather than testing endpoints only: a segment
    can pass straight through an obstacle while both of its endpoints sit
    comfortably outside it, and endpoint-only testing scores that as clear.
    """
    if len(waypoints) < 2 or not obstacles:
        return float("inf")
    worst = float("inf")
    for (x0, y0), (x1, y1) in pairwise(waypoints):
        seg = hypot(x1 - x0, y1 - y0)
        steps = max(int(seg / step_mm), 1)
        for i in range(steps + 1):
            t = i / steps
            px, py = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            for o in obstacles:
                worst = min(worst, dist((px, py), o.pos_mm) - o.radius_mm)
    return worst


def _percentile(values: list[float], pct: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * pct))]


def straight_line_blocked(obstacles, request: PlanRequest) -> bool:
    """Does any obstacle sit on the direct start->goal segment?

    This splits the instances into two populations that must not share a
    length-ratio column. On an unblocked instance the optimal path IS the
    straight line, so any exact planner scores 1.000 and the column measures
    nothing but "was this instance trivial". Measured over uniform-field
    sampling only 4-13 of 20 instances are blocked at these densities, so a
    median pooled over both populations is dominated by the trivial half.
    """
    (x0, y0), (x1, _) = request.start_mm, request.goal_mm
    for o in obstacles:
        px, py = o.pos_mm
        # Segment is axis-aligned here (y0 == y1), so the perpendicular
        # distance is just |py - y0| within the x span.
        #
        # The band must equal what the planners actually inflate obstacles by,
        # or this mislabels instances. That is obstacle radius + total_clearance
        # = 90 + 120 = 210 mm from centre, matching `inflate_radius` in
        # visibility_graph.py. Before commit 5b56ab3 the planners inflated by
        # only radius + clearance (120 mm from centre, measured as a 30 mm
        # surface floor), and this threshold had to be 120 to match; that
        # commit fixed the under-inflation, so the radius term comes back.
        # Keep this in step with the planners -- the selftest pins it to
        # request.total_clearance_mm rather than a literal for that reason.
        band = o.radius_mm + request.total_clearance_mm
        if min(x0, x1) <= px <= max(x0, x1) and abs(py - y0) < band:
            return True
    return False


def sweep(instances: int, densities=DENSITIES) -> dict:
    """{(planner, density): {n, successes, ratios, times, clearances}}

    ``ratios`` and ``clearances`` collect BLOCKED instances only (see
    ``straight_line_blocked``); ``times`` collects every call.
    """
    out: dict = {}

    # Warm-up: the first planning call of the process pays one-time import and
    # networkx setup cost. Left in, it landed entirely in the lowest-density
    # bucket and showed up as a p95 of 92ms against a 1.3ms median.
    warm = PlanRequest(start_mm=START_MM, goal_mm=GOAL_MM, obstacles=random_obstacles(6, random.Random(0)))
    run_all(warm, seed=0)

    for density in densities:
        for i in range(instances):
            rng = random.Random(hash((density, i)) & 0xFFFFFFFF)
            obstacles = random_obstacles(density, rng)
            request = PlanRequest(start_mm=START_MM, goal_mm=GOAL_MM, obstacles=obstacles)
            blocked = straight_line_blocked(obstacles, request)
            # Same instance to all three; PRM's seed varies with the instance,
            # not the planner, so its sampling is paired too.
            for plan in run_all(request, seed=i):
                bucket = out.setdefault(
                    (plan.name, density),
                    {"n": 0, "blocked": 0, "successes": 0, "ratios": [], "times": [], "clearances": []},
                )
                bucket["n"] += 1
                bucket["times"].append(plan.planning_time_ms)
                if plan.success and plan.path_length_mm > 0:
                    bucket["successes"] += 1
                    if blocked:
                        bucket["blocked"] += 1
                        bucket["ratios"].append(plan.path_length_mm / STRAIGHT_LINE_MM)
                        bucket["clearances"].append(min_clearance_mm(plan.waypoints_mm, obstacles))
    return out


def report(results: dict, instances: int, densities=DENSITIES) -> None:
    # What the planners actually inflate to, from centre: see straight_line_blocked.
    required = DEFAULT_ROBOT_RADIUS_MM + PlanRequest(start_mm=START_MM, goal_mm=GOAL_MM).total_clearance_mm
    print(
        f"\nPaired planner comparison. n = {instances} random instances per density,\n"
        f"same instance given to all three planners. Field {FIELD_LENGTH_MM:.0f}x{FIELD_WIDTH_MM:.0f} mm,\n"
        f"start {START_MM} -> goal {GOAL_MM}, straight-line distance {STRAIGHT_LINE_MM:.0f} mm.\n"
        f"All three run with skip_direct_path=True, so every call measures full\n"
        f"map/graph/roadmap construction, never the trivial clear-field shortcut.\n"
    )
    header = (
        f"{'planner':<26}{'solved':>8}{'blocked n':>11}{'length ratio':>14}"
        f"{'time med ms':>13}{'time p95 ms':>13}{'6-robot ms':>12}{'fits/tick':>11}{'min clear mm':>14}"
    )
    for density in densities:
        any_bucket = next((results[(n, density)] for n in PLANNER_NAMES if (n, density) in results), None)
        blocked_n = any_bucket["blocked"] if any_bucket else 0
        print(f"\n  obstacles = {density}   ({blocked_n}/{instances} instances block the straight line)")
        print("  " + header)
        print("  " + "-" * len(header))
        for name in PLANNER_NAMES:
            b = results.get((name, density))
            if not b:
                continue
            solved = f"{b['successes']}/{b['n']}"
            ratio = f"{median(b['ratios']):.4f}" if b["ratios"] else "--"
            clear = f"{min(b['clearances']):.0f}" if b["clearances"] else "--"
            med = median(b["times"])
            # ResearchRuntime.plan() walks robots in a plain for-loop, so a
            # team's planning cost is N * per-call, and it has to land inside
            # ONE control tick. That, not raw speed, is the shipping constraint.
            team_ms = med * TEAM_ROBOTS
            fits = int(CONTROL_TICK_MS // med) if med > 0 else 999
            print(
                f"  {name:<26}{solved:>8}{b['blocked']:>11}{ratio:>14}"
                f"{med:>13.2f}{_percentile(b['times'], 0.95):>13.2f}"
                f"{team_ms:>12.1f}{fits:>11}{clear:>14}"
            )
    print(
        f"\n  length ratio: median over BLOCKED instances only ('blocked n' column) -- those\n"
        f"                where an obstacle sits on the direct segment. 1.0000 = straight line.\n"
        f"                The visibility graph is the shortest-path reference BY CONSTRUCTION,\n"
        f"                so its row is the optimum the other two pay a detour against. It sits\n"
        f"                near 1.0000 because a grazing obstacle costs only a few mm of lateral\n"
        f"                detour over an 8 m run -- read the 4th decimal, not the 1st.\n"
        f"                Unblocked instances are excluded because every exact planner scores\n"
        f"                1.0000 on them, which would report instance triviality, not planner\n"
        f"                quality. At obstacles=0 the column is empty by construction.\n"
        f"  min clear mm: worst closest-approach to an obstacle SURFACE across blocked\n"
        f"                instances, sampled every 20 mm along the path. The planners inflate\n"
        f"                obstacles to {required:.0f} mm from centre (robot radius + clearance), which is\n"
        f"                {required - DEFAULT_ROBOT_RADIUS_MM:.0f} mm from the surface -- so {required - DEFAULT_ROBOT_RADIUS_MM:.0f} is the designed floor, not a warning.\n"
        f"                Below 0 the path cuts through an obstacle.\n"
        f"  time: per-call latency over all {instances} calls per density, solved or not,\n"
        f"        after a discarded warm-up call.\n"
    )


# Categorical slots 1-3 of the validated default palette. Order is the
# CVD-safety mechanism, not cosmetic: this triple passes the all-pairs gate in
# both modes (worst CVD dE 9.2). Aqua sits at 2.74:1 on the light surface, below
# the 3:1 bar, so the relief rule applies -- every series carries a direct label
# and the table above is always printed alongside.
SERIES_COLOURS = {
    "PRM+Dijkstra": "#2a78d6",              # slot 1, blue
    "VisibilityGraph+Dijkstra": "#eb6834",  # slot 2, orange
    "Voronoi+Dijkstra": "#1baf7a",          # slot 3, aqua
}
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e3e2df"


def plot(results: dict, instances: int, path: str, densities=DENSITIES) -> str:
    """Two stacked panels sharing the density axis.

    Deliberately NOT one chart with twin y-axes: latency (ms) and length ratio
    (dimensionless) have unrelated scales, and a dual axis lets the reader infer
    crossings that are an artefact of where the two scales were pinned.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_t, ax_r) = plt.subplots(
        2, 1, figsize=(9, 8.5), sharex=True, gridspec_kw={"hspace": 0.18}
    )
    fig.patch.set_facecolor(SURFACE)

    for ax in (ax_t, ax_r):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
        ax.tick_params(colors=INK_2, labelsize=9)

    # --- panel A: per-call latency, against the tick budget ---
    ax_t.axhline(CONTROL_TICK_MS, color=INK_2, linewidth=1.4, linestyle=(0, (5, 4)), zorder=1)
    ax_t.text(
        densities[-1], CONTROL_TICK_MS * 1.06,
        f"{CONTROL_TICK_MS:.0f} ms control tick (one robot's whole share)",
        color=INK_2, fontsize=8.5, ha="right", va="bottom",
    )
    budget_6 = CONTROL_TICK_MS / TEAM_ROBOTS
    ax_t.axhline(budget_6, color=INK_2, linewidth=1.4, linestyle=(0, (2, 3)), zorder=1)
    ax_t.text(
        densities[-1], budget_6 * 1.08,
        f"{budget_6:.1f} ms = tick / {TEAM_ROBOTS} robots planning serially",
        color=INK_2, fontsize=8.5, ha="right", va="bottom",
    )

    for name in PLANNER_NAMES:
        xs = [d for d in densities if (name, d) in results]
        ys = [median(results[(name, d)]["times"]) for d in xs]
        ax_t.plot(xs, ys, color=SERIES_COLOURS[name], linewidth=2.0,
                  marker="o", markersize=5, zorder=3)
        ax_t.annotate(
            name.replace("+Dijkstra", ""), (xs[-1], ys[-1]),
            xytext=(6, 0), textcoords="offset points",
            color=SERIES_COLOURS[name], fontsize=9.5, va="center", fontweight="bold",
        )

    ax_t.set_yscale("log")
    ax_t.set_ylabel("median planning time per call, ms (log)", color=INK, fontsize=9.5)
    ax_t.set_title(
        "Planner cost against the control-tick budget",
        color=INK, fontsize=13, fontweight="bold", loc="left", pad=10,
    )

    # --- panel B: path length against the straight-line optimum ---
    ax_r.axhline(1.0, color=INK_2, linewidth=1.4, linestyle=(0, (5, 4)), zorder=1)
    ax_r.text(
        densities[-1], 1.002, "1.000 = straight line, the unobstructed optimum",
        color=INK_2, fontsize=8.5, ha="right", va="bottom",
    )
    for name in PLANNER_NAMES:
        xs = [d for d in densities if (name, d) in results and results[(name, d)]["ratios"]]
        if not xs:
            continue
        ys = [median(results[(name, d)]["ratios"]) for d in xs]
        ax_r.plot(xs, ys, color=SERIES_COLOURS[name], linewidth=2.0,
                  marker="o", markersize=5, zorder=3)
        ax_r.annotate(
            name.replace("+Dijkstra", ""), (xs[-1], ys[-1]),
            xytext=(6, 0), textcoords="offset points",
            color=SERIES_COLOURS[name], fontsize=9.5, va="center", fontweight="bold",
        )

    ax_r.set_ylabel("path length / straight-line distance", color=INK, fontsize=9.5)
    ax_r.set_xlabel("obstacles on the field", color=INK, fontsize=9.5)
    ax_r.set_xticks(list(densities))
    ax_r.set_title(
        "Path quality on instances where the straight line is blocked",
        color=INK, fontsize=13, fontweight="bold", loc="left", pad=10,
    )

    fig.text(
        0.005, 0.005,
        f"n = {instances} paired random instances per density, same instances to all three planners. "
        f"skip_direct_path=True throughout.",
        color=INK_2, fontsize=8,
    )
    fig.savefig(path, dpi=170, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
    return path


def selftest() -> None:
    """Fails if the measurement code is wrong, not if a planner is slow."""
    rng = random.Random(0)

    obs = random_obstacles(12, rng)
    assert len(obs) == 12, f"rejection sampling gave up: {len(obs)}/12"
    for o in obs:
        assert min(dist(o.pos_mm, START_MM), dist(o.pos_mm, GOAL_MM)) >= ENDPOINT_KEEPOUT_MM
    for i, a in enumerate(obs):
        for b in obs[i + 1:]:
            assert dist(a.pos_mm, b.pos_mm) >= 3 * DEFAULT_ROBOT_RADIUS_MM

    # A straight line through an obstacle centred on it must read as fully
    # negative clearance: -radius. Endpoint-only testing would return a large
    # positive number here, which is the bug this check exists to catch.
    blocker = (Obstacle(pos_mm=(0.0, 0.0), radius_mm=90.0),)
    through = min_clearance_mm(((-1000.0, 0.0), (1000.0, 0.0)), blocker)
    assert abs(through - -90.0) < 1e-6, f"path through obstacle scored {through:.3f}, expected -90"

    # A path detouring 500mm around it clears the surface by 500 - 90 = 410.
    around = min_clearance_mm(((-1000.0, 500.0), (1000.0, 500.0)), blocker)
    assert abs(around - 410.0) < 1e-6, f"detour scored {around:.3f}, expected 410"

    assert min_clearance_mm(((0.0, 0.0), (1.0, 1.0)), ()) == float("inf")
    assert _percentile([1.0, 2.0, 3.0, 4.0], 0.0) == 1.0
    assert abs(STRAIGHT_LINE_MM - 8000.0) < 1e-9

    # Blocked/unblocked split: an obstacle on the segment blocks, one parked
    # off to the side does not, and one beyond the goal's x does not either.
    req = PlanRequest(start_mm=START_MM, goal_mm=GOAL_MM)
    assert straight_line_blocked((Obstacle(pos_mm=(0.0, 0.0)),), req)
    assert not straight_line_blocked((Obstacle(pos_mm=(0.0, 2500.0)),), req)
    assert not straight_line_blocked((Obstacle(pos_mm=(4500.0, 0.0)),), req)
    # Just outside the inflated radius does not block; just inside does.
    # Pinned to the planners' own inflation so a change there fails here
    # rather than silently mislabelling instances.
    edge = DEFAULT_ROBOT_RADIUS_MM + req.total_clearance_mm
    assert not straight_line_blocked((Obstacle(pos_mm=(0.0, edge + 1)),), req)
    assert straight_line_blocked((Obstacle(pos_mm=(0.0, edge - 1)),), req)

    # Every instance is paired: the same density+index must rebuild bit-identically.
    a = random_obstacles(6, random.Random(hash((6, 3)) & 0xFFFFFFFF))
    b = random_obstacles(6, random.Random(hash((6, 3)) & 0xFFFFFFFF))
    assert [o.pos_mm for o in a] == [o.pos_mm for o in b], "instances are not reproducible"

    print("selftest OK")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instances", type=int, default=20, help="Random instances per density")
    parser.add_argument("--selftest", action="store_true", help="Check the measurement code, then exit")
    parser.add_argument("--plot", metavar="PATH", help="Also write the two-panel figure here")
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return

    results = sweep(args.instances)
    report(results, args.instances)
    if args.plot:
        print(f"  figure: {plot(results, args.instances, args.plot)}\n")


if __name__ == "__main__":
    main()
