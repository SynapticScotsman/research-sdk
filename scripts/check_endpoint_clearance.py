#!/usr/bin/env python3
"""Is the traverse the driver is asked to run actually reachable in this clip?

WHY THIS EXISTS. `drive_grsim.py` sends one robot back and forth between
(-3200, 0) and (3200, 0). Those points sit in the two goal mouths, which is
exactly where a recorded match parks its goalkeepers. Replaying the blue team of
2024-07-19_TurtleRabbit-vs-NAMeC from t=180 s, a keeper sits 32 to 89 mm from
(-3200, 0) for most of the clip. The planners inflate obstacles by 210 mm
(90 mm obstacle + 90 mm robot + 30 mm clearance), so the goal is inside an
inflated obstacle and NO path exists. Every planner correctly returned failure.

What that cost, measured on paired-blue-90-fullbuild/prm_run02.json: 247 of 253
planning calls failed, 240 of them in one unbroken streak, because a failed plan
leaves `robot.path` empty and `not robot.path` re-triggers a plan on the very
next tick. The robot stood still for roughly 12 s of a 30 s run, and
`mean_plan_ms` averaged 247 cheap failure returns (0.15 ms) against 6 real
roadmap builds. Both earlier blue sets have the same defect: ~300 failed calls
per run and 2 laps, against 0 failures and 5 laps for the parked yellow team.

So this is a SCENARIO fault, not a planner fault and not a bug in the search.
A comparison run on blocked endpoints measures how fast each planner can say
"no path", which is not what RQ1 asks.

Clearance here is a necessary condition, not a sufficient one: an endpoint clear
of every obstacle centre by more than the inflation radius is reachable from
nearby, but the corridor to it can still close. It rules out the impossible
case, which is the one that silently ate half of every blue run.

Usage:
    python scripts/check_endpoint_clearance.py --log LOG [--replay-team blue]
    python scripts/check_endpoint_clearance.py --selftest
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# 90 mm obstacle + 90 mm robot + 30 mm clearance. A point closer than this to an
# obstacle CENTRE lies inside the inflated obstacle and no roadmap can reach it.
INFLATION_MM = 210.0

# Endpoints the driver may be asked to use, x first then lane y.
DEFAULT_XS = (3200.0, 2800.0, 2400.0, 2000.0)
DEFAULT_YS = (0.0, 1000.0, -1000.0, 1500.0, -1500.0)


def clearance_series(frames, point):
    """Distance from `point` to the nearest obstacle centre, one per frame."""
    px, py = point
    out = []
    for obstacles in frames:
        if not obstacles:
            continue
        out.append(min(math.hypot(px - ox, py - oy) for ox, oy in obstacles))
    return out


def score(frames, point, inflation_mm: float = INFLATION_MM):
    """Worst-case and typical clearance, plus the fraction of frames blocked."""
    series = clearance_series(frames, point)
    if not series:
        return {"min_mm": float("inf"), "blocked_frac": 0.0, "frames": 0}
    blocked = sum(1 for d in series if d < inflation_mm)
    series_sorted = sorted(series)
    return {
        "min_mm": series_sorted[0],
        "p05_mm": series_sorted[max(0, len(series) // 20)],
        "blocked_frac": blocked / len(series),
        "frames": len(series),
    }


def replay_frames(log_path, seconds, skip_seconds, count, team):
    """Obstacle positions per frame, exactly the set drive_grsim would place.

    Reuses LogOpponents so the identity selection, the count and the hold
    behaviour match the driver rather than approximating them. Its `place`
    method talks to a socket, so we step `frame_at` directly and apply the same
    hold rule, which is the part that changes which positions appear.
    """
    from drive_grsim import LogOpponents

    opp = LogOpponents(log_path, seconds=seconds, skip_seconds=skip_seconds,
                       count=count, team=team)
    return frames_from_opponents(opp), opp


def frames_from_opponents(opp):
    """The positions `opp.place` would send, without a socket.

    Applies the same hold rule: a robot missing from a frame keeps its last
    position rather than shifting slots, which is what the driver does.
    """
    frames = []
    last: dict = {}
    for cap in opp.captures:
        present = {(bool(o.isYellow), int(o.robot_id)): o.pos_mm
                   for o in cap.obstacles}
        positions = []
        for key in opp.keys:
            pos = present.get(key) or last.get(key)
            if pos is None:
                continue
            last[key] = pos
            positions.append(pos)
        frames.append(positions)
    return frames


def selftest() -> int:
    # An obstacle parked on the point blocks it in every frame.
    frames = [[(0.0, 0.0)]] * 10
    s = score(frames, (0.0, 0.0))
    assert s["blocked_frac"] == 1.0, s
    assert s["min_mm"] == 0.0, s

    # Just inside the inflation radius is still blocked; just outside is clear.
    assert score([[(0.0, 0.0)]], (209.0, 0.0))["blocked_frac"] == 1.0
    assert score([[(0.0, 0.0)]], (211.0, 0.0))["blocked_frac"] == 0.0

    # Blocked for part of the clip only: 3 of 10 frames.
    frames = [[(0.0, 0.0)]] * 3 + [[(5000.0, 0.0)]] * 7
    s = score(frames, (0.0, 0.0))
    assert abs(s["blocked_frac"] - 0.3) < 1e-9, s

    # Nearest of several obstacles wins.
    s = score([[(1000.0, 0.0), (300.0, 0.0), (2000.0, 0.0)]], (0.0, 0.0))
    assert s["min_mm"] == 300.0, s

    # An empty frame carries no information and must not count as clearance 0.
    s = score([[], [(5000.0, 0.0)]], (0.0, 0.0))
    assert s["frames"] == 1 and s["blocked_frac"] == 0.0, s

    print("check_endpoint_clearance selftest OK")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--log")
    p.add_argument("--log-skip", type=float, default=180.0)
    p.add_argument("--log-seconds", type=float, default=30.0)
    p.add_argument("--obstacles", type=int, default=6)
    p.add_argument("--replay-team", choices=("yellow", "blue", "both"), default="blue")
    p.add_argument("--inflation-mm", type=float, default=INFLATION_MM)
    p.add_argument("--selftest", action="store_true")
    args = p.parse_args()

    if args.selftest:
        return selftest()
    if not args.log:
        p.error("--log is required unless --selftest")

    frames, opp = replay_frames(args.log, args.log_seconds, args.log_skip,
                                args.obstacles, args.replay_team)
    print(f"{len(frames)} frames, {args.obstacles} replayed {args.replay_team} "
          f"robots, clip {args.log_skip:.0f}-{args.log_skip + args.log_seconds:.0f} s")
    print(f"blocked means nearest obstacle centre within {args.inflation_mm:.0f} mm, "
          f"so the point is inside an inflated obstacle and unreachable\n")

    head = f"{'endpoint':>18}{'min mm':>10}{'p05 mm':>10}{'blocked':>10}"
    print(head)
    print("-" * len(head))
    rows = []
    for x in DEFAULT_XS:
        for y in DEFAULT_YS:
            for sx in (-x, x):
                s = score(frames, (sx, y), args.inflation_mm)
                rows.append(((sx, y), s))
                flag = "  <-- BLOCKED" if s["blocked_frac"] > 0 else ""
                print(f"{f'({sx:.0f}, {y:.0f})':>18}{s['min_mm']:>10.0f}"
                      f"{s['p05_mm']:>10.0f}{s['blocked_frac'] * 100:>9.1f}%{flag}")

    print("\nA traverse needs BOTH endpoints clear. Best symmetric pairs:")
    pairs = []
    for x in DEFAULT_XS:
        for y in DEFAULT_YS:
            a = score(frames, (-x, y), args.inflation_mm)
            b = score(frames, (x, y), args.inflation_mm)
            pairs.append((min(a["min_mm"], b["min_mm"]),
                          max(a["blocked_frac"], b["blocked_frac"]), x, y))
    for worst, blocked, x, y in sorted(pairs, key=lambda r: (r[1], -r[0]))[:5]:
        print(f"  x=+/-{x:.0f} y={y:>7.0f}   worst clearance {worst:6.0f} mm   "
              f"blocked {blocked * 100:.1f}% of frames")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
