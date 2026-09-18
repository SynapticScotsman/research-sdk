#!/usr/bin/env python3
"""Pick a traverse whose endpoints are clear but whose corridor meets traffic.

Two requirements pull against each other. The endpoints must never sit inside an
inflated obstacle, or no path exists and the run measures failure latency
(see docs/grsim-scenario-validity.md, fault 1). The corridor should cross where
the recorded robots actually are, or the planner is never exercised and the run
measures nothing.

Optimising only the first gives the y=1500 lane, whose endpoints are clear
because the recorded robots do not go there: closest approach 465 to 905 mm
against a 210 mm inflation radius, and a trigger that never fires.

This scores candidate traverses on both, and allows the start and goal to sit at
different lanes so the route can cut across the field rather than run parallel
to the traffic.

Usage:
    python scripts/choose_traverse.py --log LOG --replay-team both --obstacles 11
    python scripts/choose_traverse.py --selftest
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

INFLATION_MM = 210.0
# Counted as an encounter when an obstacle centre comes this close to the
# straight-line corridor. The geometric trigger fires at 90 mm from the PATH,
# and a planned path deviates from the straight line, so this is deliberately
# looser: it asks whether traffic is near the route at all.
NEAR_MM = 400.0

XS = (3200.0, 2800.0, 2400.0)
YS = (0.0, 750.0, 1500.0, -750.0, -1500.0)


def point_to_segment(p, a, b) -> float:
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def endpoint_clearance(frames, point):
    """Smallest distance from `point` to any obstacle centre, over all frames."""
    worst = float("inf")
    for obstacles in frames:
        for o in obstacles:
            worst = min(worst, math.dist(point, o))
    return worst


def corridor_traffic(frames, start, goal, near_mm: float = NEAR_MM):
    """How often an obstacle sits near the straight line, and how close it gets."""
    hits = 0
    closest = float("inf")
    for obstacles in frames:
        near = False
        for o in obstacles:
            d = point_to_segment(o, start, goal)
            closest = min(closest, d)
            if d <= near_mm:
                near = True
        hits += near
    return {"frames_with_traffic": hits / max(len(frames), 1), "closest_mm": closest}


def selftest() -> int:
    frames = [[(0.0, 0.0)]]
    assert endpoint_clearance(frames, (300.0, 0.0)) == 300.0
    # An obstacle sitting on the line is distance 0 from it.
    t = corridor_traffic([[(0.0, 0.0)]], (-100.0, 0.0), (100.0, 0.0))
    assert t["closest_mm"] == 0.0 and t["frames_with_traffic"] == 1.0, t
    # An obstacle far to the side is not traffic.
    t = corridor_traffic([[(0.0, 5000.0)]], (-100.0, 0.0), (100.0, 0.0))
    assert t["frames_with_traffic"] == 0.0, t
    # Distance is to the SEGMENT, not the infinite line: a point beyond the end
    # measures from the endpoint.
    assert point_to_segment((200.0, 0.0), (-100.0, 0.0), (100.0, 0.0)) == 100.0
    # Half the frames have traffic.
    t = corridor_traffic([[(0.0, 0.0)], [(0.0, 5000.0)]], (-100.0, 0.0), (100.0, 0.0))
    assert t["frames_with_traffic"] == 0.5, t
    print("choose_traverse selftest OK")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--log")
    p.add_argument("--log-skip", type=float, default=180.0)
    p.add_argument("--log-seconds", type=float, default=30.0)
    p.add_argument("--obstacles", type=int, default=11)
    p.add_argument("--replay-team", choices=("yellow", "blue", "both"), default="both")
    p.add_argument("--min-clearance-mm", type=float, default=400.0,
                   help="Reject an endpoint that ever comes closer than this. Above "
                        "the 210 mm inflation radius, to leave room to manoeuvre.")
    p.add_argument("--selftest", action="store_true")
    args = p.parse_args()

    if args.selftest:
        return selftest()
    if not args.log:
        p.error("--log is required unless --selftest")

    from check_endpoint_clearance import replay_frames
    frames, _ = replay_frames(args.log, args.log_seconds, args.log_skip,
                              args.obstacles, args.replay_team)
    print(f"{len(frames)} frames, {args.obstacles} replayed {args.replay_team} robots, "
          f"clip {args.log_skip:.0f}-{args.log_skip + args.log_seconds:.0f} s")
    print(f"endpoints must stay {args.min_clearance_mm:.0f} mm clear; corridor traffic "
          f"counts obstacles within {NEAR_MM:.0f} mm of the straight line\n")

    rows = []
    for x in XS:
        for y0 in YS:
            for y1 in YS:
                start, goal = (-x, y0), (x, y1)
                cs = endpoint_clearance(frames, start)
                cg = endpoint_clearance(frames, goal)
                if min(cs, cg) < args.min_clearance_mm:
                    continue
                t = corridor_traffic(frames, start, goal)
                rows.append((t["frames_with_traffic"], min(cs, cg), t["closest_mm"],
                             x, y0, y1))

    if not rows:
        print("No traverse satisfies the clearance floor. Lower --min-clearance-mm.")
        return 1

    rows.sort(reverse=True)
    head = (f"{'start':>16}{'goal':>16}{'endpoint clear':>17}"
            f"{'frames w/ traffic':>20}{'closest to line':>18}")
    print(head)
    print("-" * len(head))
    for frac, clear, closest, x, y0, y1 in rows[:12]:
        print(f"{f'(-{x:.0f}, {y0:.0f})':>16}{f'({x:.0f}, {y1:.0f})':>16}"
              f"{clear:>15.0f} mm{frac * 100:>19.0f}%{closest:>15.0f} mm")

    frac, clear, closest, x, y0, y1 = rows[0]
    if y0 == y1:
        print(f"\nBest: --traverse-x {x:.0f} --lane-centre-mm {y0:.0f}")
    else:
        print(f"\nBest: start y={y0:.0f} and goal y={y1:.0f} differ, which "
              f"--lane-centre-mm cannot express; it sets one lane for both ends.")
    print(f"  traffic within {NEAR_MM:.0f} mm of the route in {frac * 100:.0f}% of "
          f"frames, nearest approach to the line {closest:.0f} mm,")
    print(f"  worst endpoint clearance {clear:.0f} mm.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
