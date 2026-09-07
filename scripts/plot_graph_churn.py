#!/usr/bin/env python3
"""Draw what `measure_graph_churn.py` measures, on a real match frame.

The churn table says "0.6% of edges flipped". This renders the thing that
sentence is about, so the number can be checked against a picture:

  left  -- one frame of a real match: robots, the Minkowski-inflated obstacles
           the planner actually avoids, every edge of the visibility graph it
           builds, and the path it chooses.
  right -- the same query a chosen interval later, with only the edges whose
           visibility CHANGED drawn, plus an arrow per robot showing how far it
           moved. Everything else is identical, so what is drawn is the churn.

Usage:
    python scripts/plot_graph_churn.py --log LOG --skip 180 --gap-ms 500 --out churn.png
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from measure_graph_churn import capture_from_logfile

from research_sdk.planners.common import (
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    PlanRequest,
    StepRecorder,
)
from research_sdk.planners.VisibilityGraph import visibility_graph

# Validated categorical slots plus neutral ink. Team colours are the league's
# own semantics, not a free choice, so yellow and blue robots keep their names.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#d8d7d3"
EDGE = "#c9c8c4"          # a visibility edge, deliberately recessive
FLIPPED = "#eb6834"       # the edges that changed: the whole point of the figure
PATH = "#1baf7a"
YELLOW_TEAM = "#eda100"
BLUE_TEAM = "#2a78d6"
INFLATE = "#b9b8b4"


def graph_snapshot(request: PlanRequest):
    """(label -> point, {edge: visible}, path) for one planning query."""
    recorder = StepRecorder()
    result = visibility_graph.plan(request, record=recorder, skip_direct_path=True)

    polygons = next((s["polygons"] for s in recorder.steps if s["kind"] == "obstacles"), None)
    if polygons is None:
        return {}, {}, ()

    label_of: dict[tuple[int, int], str] = {}
    points: dict[str, tuple[float, float]] = {}
    for poly_idx, poly in enumerate(polygons):
        obs = request.obstacles[poly_idx] if poly_idx < len(request.obstacles) else None
        who = f"{'y' if obs.isYellow else 'b'}{obs.robot_id}" if obs else f"p{poly_idx}"
        for vert_idx, (vx, vy) in enumerate(poly):
            label = f"{who}v{vert_idx}"
            label_of[(round(vx), round(vy))] = label
            points[label] = (vx, vy)
    for name, pt in (("start", request.start_mm), ("goal", request.goal_mm)):
        label_of[(round(pt[0]), round(pt[1]))] = name
        points[name] = tuple(pt)

    edges: dict[frozenset[str], bool] = {}
    for step in recorder.steps:
        if step["kind"] != "edge_test":
            continue
        a = label_of.get((round(step["a"][0]), round(step["a"][1])))
        b = label_of.get((round(step["b"][0]), round(step["b"][1])))
        if a and b and a != b:
            edges[frozenset((a, b))] = bool(step["accepted"])
    return points, edges, (result.waypoints_mm if result.success else ())


def draw_pitch(ax) -> None:
    from matplotlib.patches import Circle, Rectangle

    half_x, half_y = FIELD_LENGTH_MM / 2, FIELD_WIDTH_MM / 2
    ax.add_patch(
        Rectangle((-half_x, -half_y), 2 * half_x, 2 * half_y,
                  fill=False, edgecolor=GRID, linewidth=1.2, zorder=1)
    )
    ax.plot([0, 0], [-half_y, half_y], color=GRID, linewidth=1.0, zorder=1)
    ax.add_patch(Circle((0, 0), 500, fill=False, edgecolor=GRID, linewidth=1.0, zorder=1))
    ax.set_xlim(-half_x - 400, half_x + 400)
    ax.set_ylim(-half_y - 400, half_y + 400)
    ax.set_aspect("equal")
    ax.axis("off")


def draw_robots(ax, obstacles, inflate_mm: float, *, hollow: bool = False) -> None:
    from matplotlib.patches import Circle

    for o in obstacles:
        colour = YELLOW_TEAM if o.isYellow else BLUE_TEAM
        ax.add_patch(
            Circle(o.pos_mm, inflate_mm, fill=False, edgecolor=INFLATE,
                   linewidth=0.7, linestyle=(0, (3, 3)), zorder=2)
        )
        ax.add_patch(
            Circle(o.pos_mm, o.radius_mm, facecolor="none" if hollow else colour,
                   edgecolor=colour, linewidth=1.4, zorder=4)
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True)
    parser.add_argument("--skip", type=float, default=180.0)
    parser.add_argument("--gap-ms", type=float, default=500.0)
    parser.add_argument("--start", type=float, nargs=2, default=[-3500.0, -2000.0])
    parser.add_argument("--goal", type=float, nargs=2, default=[3500.0, 2000.0])
    parser.add_argument("--out", default="graph_churn.png")
    args = parser.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    captures = capture_from_logfile(args.log, seconds=8.0, skip_seconds=args.skip)
    if len(captures) < 10:
        print("not enough frames")
        return

    # Pick the pair separated by roughly --gap-ms, and prefer a stretch where
    # something actually moves: a stoppage makes a truthful but dull picture.
    target = args.gap_ms / 1000.0
    best = None
    for i in range(len(captures) - 1):
        j = i
        while j < len(captures) - 1 and captures[j].t_s - captures[i].t_s < target:
            j += 1
        if captures[j].t_s - captures[i].t_s < target * 0.8:
            continue
        prev = {(o.isYellow, o.robot_id): o.pos_mm for o in captures[i].obstacles}
        motion = sum(
            math.dist(o.pos_mm, prev[(o.isYellow, o.robot_id)])
            for o in captures[j].obstacles
            if (o.isYellow, o.robot_id) in prev
        )
        if best is None or motion > best[0]:
            best = (motion, i, j)
    if best is None:
        print("no usable frame pair")
        return
    motion, i, j = best
    a, b = captures[i], captures[j]

    req_a = PlanRequest(start_mm=tuple(args.start), goal_mm=tuple(args.goal), obstacles=a.obstacles)
    req_b = PlanRequest(start_mm=tuple(args.start), goal_mm=tuple(args.goal), obstacles=b.obstacles)
    inflate = req_a.obstacles[0].radius_mm + req_a.total_clearance_mm if req_a.obstacles else 210.0

    pts_a, edges_a, path_a = graph_snapshot(req_a)
    pts_b, edges_b, _ = graph_snapshot(req_b)
    if not edges_a or not edges_b:
        print("an endpoint was blocked in one of the frames; try another --skip")
        return

    shared = edges_a.keys() & edges_b.keys()
    flipped = [e for e in shared if edges_a[e] != edges_b[e]]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6.2))
    fig.patch.set_facecolor(SURFACE)
    for ax in (ax1, ax2):
        ax.set_facecolor(SURFACE)
        draw_pitch(ax)

    # ---- left: the whole graph the planner builds ----
    visible = [e for e, ok in edges_a.items() if ok]
    for e in visible:
        (p, q) = tuple(e)
        (x0, y0), (x1, y1) = pts_a[p], pts_a[q]
        ax1.plot([x0, x1], [y0, y1], color=EDGE, linewidth=0.35, zorder=3)
    draw_robots(ax1, a.obstacles, inflate)
    if path_a:
        ax1.plot([p[0] for p in path_a], [p[1] for p in path_a],
                 color=PATH, linewidth=2.6, zorder=6, solid_capstyle="round")
    for pt, marker in ((args.start, "o"), (args.goal, "*")):
        ax1.scatter(*pt, s=110 if marker == "o" else 220, marker=marker,
                    color=INK, zorder=7)
    ax1.set_title(
        f"One frame: {len(a.obstacles)} robots, {len(visible):,} visible edges of "
        f"{len(edges_a):,} tested",
        color=INK, fontsize=12, fontweight="bold", loc="left", pad=12,
    )

    # ---- right: only what changed ----
    draw_robots(ax2, a.obstacles, inflate, hollow=True)
    draw_robots(ax2, b.obstacles, inflate)
    prev = {(o.isYellow, o.robot_id): o.pos_mm for o in a.obstacles}
    for o in b.obstacles:
        k = (o.isYellow, o.robot_id)
        if k in prev and math.dist(o.pos_mm, prev[k]) > 5:
            ax2.annotate(
                "", xy=o.pos_mm, xytext=prev[k],
                arrowprops={"arrowstyle": "->", "color": INK_2, "linewidth": 1.1},
                zorder=5,
            )
    for e in flipped:
        (p, q) = tuple(e)
        pts = pts_b if p in pts_b and q in pts_b else pts_a
        if p not in pts or q not in pts:
            continue
        (x0, y0), (x1, y1) = pts[p], pts[q]
        ax2.plot([x0, x1], [y0, y1], color=FLIPPED, linewidth=1.5, zorder=6)
    pct = 100.0 * len(flipped) / len(shared) if shared else 0.0
    ax2.set_title(
        f"{(b.t_s - a.t_s) * 1000:.0f} ms later: {len(flipped):,} of {len(shared):,} "
        f"shared edges changed ({pct:.2f}%)",
        color=INK, fontsize=12, fontweight="bold", loc="left", pad=12,
    )

    fig.text(
        0.5, 0.03,
        f"{Path(args.log).name}   ·   grey = visibility edge   ·   orange = visibility changed   ·   "
        f"green = chosen path   ·   dashed = {inflate:.0f} mm inflation the planner avoids",
        color=INK_2, fontsize=9, ha="center",
    )
    fig.savefig(args.out, dpi=160, facecolor=SURFACE, bbox_inches="tight")
    print(f"wrote {args.out}")
    print(f"  frame pair {i} -> {j}, {(b.t_s - a.t_s) * 1000:.0f} ms apart")
    print(f"  total robot motion in that gap: {motion:.0f} mm across {len(b.obstacles)} robots")
    print(f"  edges: {len(edges_a):,} tested, {len(visible):,} visible, "
          f"{len(flipped):,} flipped ({pct:.2f}% of {len(shared):,} shared)")


if __name__ == "__main__":
    main()
