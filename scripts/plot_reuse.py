#!/usr/bin/env python3
"""Draw why incremental reuse cannot pay on an obstacle-vertex graph.

The reuse table says the oracle ceiling is 98.2% on a real match while a sound
implementation can prove only 0.4% reusable. That gap is the whole finding and
a table hides it. This colours every edge of the graph by which of the three
cases it falls into:

  grey    the visibility answer is unchanged AND an implementation can prove it
          without retesting. This is the only real saving.
  orange  the answer is unchanged, but no sound test can establish that,
          because an endpoint moved or a moved obstacle swept the segment.
          The saving exists and is unreachable.
  red     the answer actually changed. Must be retested by anyone.

A picture that is almost entirely orange means the opportunity is real and
unreachable, which is exactly the argument for not implementing D*/LPA* here.

Usage:
    python scripts/plot_reuse.py --log LOG --skip 180 --out reuse.png
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from dynamic_scenario import GOAL_MM as PATROL_GOAL
from dynamic_scenario import SCENARIOS as PATROL_SCENARIOS
from dynamic_scenario import START_MM as PATROL_START
from measure_reuse import STILL_MM, _segment_swept_by
from plot_graph_churn import graph_snapshot

from research_sdk.planners.common import Obstacle, PlanRequest

SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#d8d7d3"
REUSABLE = "#c9c8c4"   # recessive: this is the part you get to skip
STRANDED = "#eb6834"   # unchanged but unprovable: the wasted opportunity
CHANGED = "#e34948"    # genuinely different
MOVED_OBS = "#2a78d6"
STILL_OBS = "#9a9994"


def classify(before, after, start_mm, goal_mm):
    """Per-edge class plus the endpoint coordinates needed to draw it."""
    req_a = PlanRequest(start_mm=start_mm, goal_mm=goal_mm, obstacles=before)
    req_b = PlanRequest(start_mm=start_mm, goal_mm=goal_mm, obstacles=after)
    pts_a, edges_a, _ = graph_snapshot(req_a)
    _, edges_b, _ = graph_snapshot(req_b)
    if not edges_a or not edges_b:
        return None, None, None

    old = {o.robot_id: o for o in before}
    new = {o.robot_id: o for o in after}
    moved = {
        rid for rid in old.keys() & new.keys()
        if math.dist(old[rid].pos_mm, new[rid].pos_mm) > STILL_MM
    }
    inflation = req_a.total_clearance_mm

    def owner(label: str):
        if label in ("start", "goal"):
            return None
        return int(label.rsplit("v", 1)[0][1:])

    centres = {o.robot_id: o.pos_mm for o in after}
    classed = []
    for e in edges_a.keys() & edges_b.keys():
        p, q = tuple(e)
        if p not in pts_a or q not in pts_a:
            continue
        if edges_a[e] != edges_b[e]:
            klass = "changed"
        else:
            op, oq = owner(p), owner(q)
            a = centres.get(op, start_mm) if op is not None else start_mm
            b = centres.get(oq, goal_mm) if oq is not None else goal_mm
            # An endpoint moved, or a moved obstacle swept the segment: either
            # way no sound test can carry the old answer over.
            endpoint_moved = (op in moved) or (oq in moved)
            swept = any(
                _segment_swept_by(a, b, old[r].pos_mm, new[r].pos_mm,
                                  old[r].radius_mm + inflation)
                for r in moved
            )
            klass = "stranded" if (endpoint_moved or swept) else "reusable"
        classed.append((pts_a[p], pts_a[q], klass))
    return classed, moved, (before, after)


def draw(ax, classed, moved, frames, title):
    from matplotlib.patches import Circle, Rectangle

    before, after = frames
    ax.set_facecolor(SURFACE)
    ax.add_patch(Rectangle((-4500, -3000), 9000, 6000, fill=False, edgecolor=GRID, linewidth=1.2))
    ax.plot([0, 0], [-3000, 3000], color=GRID, linewidth=1.0)
    ax.add_patch(Circle((0, 0), 500, fill=False, edgecolor=GRID, linewidth=1.0))

    # Draw reusable first so the two interesting classes sit on top.
    order = {"reusable": 0, "stranded": 1, "changed": 2}
    colour = {"reusable": REUSABLE, "stranded": STRANDED, "changed": CHANGED}
    width = {"reusable": 0.3, "stranded": 0.4, "changed": 1.3}
    for a, b, klass in sorted(classed, key=lambda c: order[c[2]]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=colour[klass],
                linewidth=width[klass], zorder=3 + order[klass])

    for o in after:
        is_moved = o.robot_id in moved
        ax.add_patch(Circle(o.pos_mm, o.radius_mm,
                            color=MOVED_OBS if is_moved else STILL_OBS, zorder=8))
    prev = {o.robot_id: o.pos_mm for o in before}
    for o in after:
        if o.robot_id in moved and math.dist(o.pos_mm, prev[o.robot_id]) > 20:
            ax.annotate("", xy=o.pos_mm, xytext=prev[o.robot_id],
                        arrowprops={"arrowstyle": "->", "color": INK_2, "linewidth": 1.1},
                        zorder=9)

    counts = {k: sum(1 for _, _, c in classed if c == k) for k in ("reusable", "stranded", "changed")}
    total = max(len(classed), 1)
    ax.set_xlim(-4800, 4800)
    ax.set_ylim(-3300, 3300)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(
        f"{title}\n{len(moved)} of {len(after)} obstacles moved   ·   "
        f"reusable {counts['reusable'] / total * 100:.1f}%   ·   "
        f"unchanged but unprovable {counts['stranded'] / total * 100:.1f}%   ·   "
        f"changed {counts['changed'] / total * 100:.1f}%",
        color=INK, fontsize=10.5, fontweight="bold", loc="left", pad=10,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True)
    parser.add_argument("--skip", type=float, default=180.0)
    parser.add_argument("--gap-ms", type=float, default=500.0)
    parser.add_argument("--speed", type=float, default=1.0, help="Patrol obstacle speed, m/s")
    parser.add_argument("--out", default="reuse.png")
    args = parser.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from measure_graph_churn import capture_from_logfile

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6.4))
    fig.patch.set_facecolor(SURFACE)

    # --- patrol: every obstacle moving ---
    loops = PATROL_SCENARIOS["scenario_1 (7 obstacles)"]
    rng = random.Random(0)
    phases = [rng.uniform(0, lp.length()) for lp in loops]
    gap_s = args.gap_ms / 1000.0

    def patrol_at(t):
        return tuple(
            Obstacle(pos_mm=lp.position(phases[i] + args.speed * 1000.0 * t),
                     radius_mm=90.0, robot_id=i, isYellow=False)
            for i, lp in enumerate(loops)
        )

    classed, moved, frames = classify(patrol_at(0.0), patrol_at(gap_s),
                                      PATROL_START, PATROL_GOAL)
    if classed:
        draw(ax1, classed, moved, frames,
             f"Synthetic patrol, {args.speed:.1f} m/s, {args.gap_ms:.0f} ms apart")

    # --- real match ---
    captures = capture_from_logfile(args.log, seconds=8.0, skip_seconds=args.skip)
    stride = max(1, round(gap_s / ((captures[-1].t_s - captures[0].t_s) / len(captures))))
    best = None
    for i in range(len(captures) - stride):
        prev = {(o.isYellow, o.robot_id): o.pos_mm for o in captures[i].obstacles}
        motion = sum(math.dist(o.pos_mm, prev[(o.isYellow, o.robot_id)])
                     for o in captures[i + stride].obstacles
                     if (o.isYellow, o.robot_id) in prev)
        if best is None or motion > best[0]:
            best = (motion, i)
    i = best[1]
    classed, moved, frames = classify(
        captures[i].obstacles, captures[i + stride].obstacles,
        (-3500.0, -2000.0), (3500.0, 2000.0),
    )
    if classed:
        draw(ax2, classed, moved, frames,
             f"Real match, {(captures[i + stride].t_s - captures[i].t_s) * 1000:.0f} ms apart")

    handles = [
        Line2D([0], [0], color=REUSABLE, lw=2.5, label="reusable: unchanged and provable"),
        Line2D([0], [0], color=STRANDED, lw=2.5, label="unchanged but unprovable"),
        Line2D([0], [0], color=CHANGED, lw=2.5, label="visibility actually changed"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor=MOVED_OBS,
               markersize=9, label="obstacle that moved"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor=STILL_OBS,
               markersize=9, label="obstacle that did not"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=5, frameon=False,
               fontsize=9, labelcolor=INK_2, bbox_to_anchor=(0.5, 0.0))
    fig.suptitle(
        "Why incremental reuse cannot pay: the surviving answers are unprovable",
        color=INK, fontsize=13, fontweight="bold", x=0.01, ha="left",
    )
    fig.tight_layout(rect=(0, 0.06, 1, 0.94))
    fig.savefig(args.out, dpi=160, facecolor=SURFACE, bbox_inches="tight")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
