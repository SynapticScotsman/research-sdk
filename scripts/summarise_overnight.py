#!/usr/bin/env python3
"""Summarise the two overnight jobs.

Job 1 is a paired three-planner comparison in grSim on one recorded clip: 15
runs each, planners alternating, everything else held. Job 2 is path stability
against obstacle speed, offline, both scenarios at six speeds.

Job 2 exists to test a relationship the committed result could not: reversals
per run separated arriving conditions from failing ones across TRIGGERS at a
single obstacle speed of 1.0 m/s. Whether that separation survives a change of
speed was never measured, so this checks it over every condition at once.

Usage:
    python scripts/summarise_overnight.py
"""

from __future__ import annotations

import json
import re
import statistics
from pathlib import Path

PLANNERTEST = Path(__file__).resolve().parent.parent
EXPERIMENT = PLANNERTEST.parent / "research-sdk"
import sys
PAIRED = PLANNERTEST / "results" / "grsim" / (
    sys.argv[1] if len(sys.argv) > 1 else "paired")
STABILITY = EXPERIMENT / "results" / "stability-speed"


OPENING_S = 5.0


def _failure_timing(robot):
    """(failures in the opening window, failures after it).

    Failures confined to the opening mean the robot started somewhere it could
    not plan out of, which is a placement fault rather than a hard scenario.
    """
    events = robot.get("diagnostics", {}).get("plan_events") or []
    if not events:
        return 0, 0
    t0 = events[0]["t_capture_s"]
    early = late = 0
    for e in events:
        if e.get("status") != "failed":
            continue
        if e["t_capture_s"] - t0 <= OPENING_S:
            early += 1
        else:
            late += 1
    return early, late


def _fail_streak(robot):
    """Longest unbroken run of failed planning calls.

    A goal inside an inflated obstacle cannot be replanned around, so every tick
    fails until the obstacle moves: 240 in a row on the blocked-endpoint set. A
    corridor that closes behind a passing robot recovers within a few calls.
    """
    events = robot.get("diagnostics", {}).get("plan_events") or []
    best = run = 0
    for e in events:
        run = run + 1 if e.get("status") == "failed" else 0
        best = max(best, run)
    return best


def _built_ms(robot):
    """Mean milliseconds over calls that actually returned a path.

    Falls back to the stored mean when a run predates the per-call diagnostics.
    """
    events = robot.get("diagnostics", {}).get("plan_events") or []
    built = [e["planning_ms"] for e in events if e.get("status") == "path"]
    if built:
        return sum(built) / len(built)
    return robot["mean_plan_ms"]


def _steady_ms(robot):
    """Mean milliseconds over successful calls after the first one.

    Every driver run is a fresh process, and numpy's default_rng costs 25 to
    39 ms on its first use in a process (measured 19/09/2026; 0.02 ms after).
    PRM is the only backend that draws random numbers, so its first build in
    every run carried that cost: 33 to 43 ms against 4.5 to 5.5 ms for each
    later build, the slowest call being the first in 60 of 60 runs across four
    sets, while Voronoi and the visibility graph showed no first-call effect.
    That doubled PRM's ms/build. drive_grsim.py now warms the generator before
    any timed call; for sets recorded before that, this column is the number
    to read. nan when a run built only one path.
    """
    events = robot.get("diagnostics", {}).get("plan_events") or []
    built = [e["planning_ms"] for e in events if e.get("status") == "path"]
    if len(built) > 1:
        return sum(built[1:]) / (len(built) - 1)
    return float("nan")


def _spread(values):
    if not values:
        return "n/a"
    if len(values) == 1:
        return f"{values[0]:.2f}"
    return f"{statistics.median(values):.2f} [{min(values):.2f}, {max(values):.2f}]"


def job1() -> None:
    runs: dict[str, list[dict]] = {}
    for f in sorted(PAIRED.glob("*.json")):
        d = json.loads(f.read_text(encoding="utf-8"))
        runs.setdefault(d["meta"]["planner"], []).append(d["robots"][0])
    if not runs:
        print("job 1: no results\n")
        return

    any_run = json.loads(next(PAIRED.glob("*.json")).read_text(encoding="utf-8"))
    m, g = any_run["meta"], any_run["provenance"]["grsim"]
    print(f"JOB 1  grSim, recorded clip {m['log_skip_s']:.0f}-"
          f"{m['log_skip_s'] + m['log_seconds']:.0f} s, {m['duration_s']:.0f} s per run, "
          f"noise {float(g['noise_x_mm']):.0f} mm, delay {g['sending_delay_ms']} ms")
    print(f"       trigger {m['trigger']} at "
          f"{m.get('trigger_centre_threshold_mm', 'default')} mm centre separation, "
          f"{m['robots']} controlled robot, {m['obstacles']} replayed\n")

    head = (f"{'planner':<12}{'runs':>5}{'crossings':>24}{'replans':>24}"
            f"{'direct':>24}{'failed':>24}{'worst streak':>24}{'ms/build':>24}{'ms/build ex.1st':>24}"
            f"{'ms/call':>24}{'closest mm':>24}")
    print(head)
    print("-" * len(head))
    for planner in ("voronoi", "visibility", "prm"):
        rs = runs.get(planner, [])
        if not rs:
            continue
        print(f"{planner:<12}{len(rs):>5}"
              f"{_spread([r['laps'] for r in rs]):>24}"
              f"{_spread([r['replans'] for r in rs]):>24}"
              f"{_spread([r['direct'] for r in rs]):>24}"
              f"{_spread([r.get('failed_plans', 0) for r in rs]):>24}"
              f"{_spread([_fail_streak(r) for r in rs]):>24}"
              f"{_spread([_built_ms(r) for r in rs]):>24}"
              f"{_spread([v for v in (_steady_ms(r) for r in rs) if v == v]):>24}"
              f"{_spread([r['mean_plan_ms'] for r in rs]):>24}"
              f"{_spread([r['closest_mm'] for r in rs]):>24}")
    print("\nmedian [min, max] over runs. Closest separation is from VISION positions,")
    print("so it is not a physics contact count and not a collision total.")
    print("ms/build averages only calls that returned a path; ms/call is the stored\n"
          "mean over every call, which a failure at about 0.15 ms drags downward.\n"
          "ms/build ex.1st leaves out each run's first build: in a fresh process that call\n"
          "pays numpy's one-off default_rng initialisation (25-39 ms), which only PRM uses.\n"
          "For sets recorded before drive_grsim.py warmed the generator, read that column.")

    # A failed call returns in roughly 0.15 ms, so a large failure share pulls
    # mean_plan_ms away from the roadmap build it is meant to measure. A goal
    # inside an inflated obstacle produced 247 failures in 253 calls once and
    # this table showed nothing unusual.
    worst = 0.0
    for rs in runs.values():
        for r in rs:
            calls = r['replans'] + r['direct'] + r.get('failed_plans', 0)
            if calls:
                worst = max(worst, r.get('failed_plans', 0) / calls)
    if worst > 0.05:
        streak = max(_fail_streak(r) for rs in runs.values() for r in rs)
        early = sum(_failure_timing(r)[0] for rs in runs.values() for r in rs)
        late = sum(_failure_timing(r)[1] for rs in runs.values() for r in rs)
        if early and not late:
            print(f"\nSCENARIO FAULT: all {early} failed calls happen in the first "
                  f"{OPENING_S:.0f} s and none after.")
            print("The robot is starting somewhere it cannot plan out of. Check that")
            print("place_teams spawns it in the same lane its target uses, and that the")
            print("start is clear:")
            print("  python scripts/check_endpoint_clearance.py --log LOG "
                  "--replay-team both")
        if early > 3 * max(late, 1):
            print(f"\nNOTE: failures skew to the opening: {early} in the first "
                  f"{OPENING_S:.0f} s against {late} after.")
        print(f"\nNOTE: up to {worst * 100:.0f}% of planning calls FAILED in some "
              f"run; longest unbroken failure streak {streak}.")
        if streak >= 50:
            print("A streak that long means the GOAL ITSELF is unreachable, not that a\n"
                  "corridor closed. The run is invalid, and ms/call is not a planning time:")
            print("  python scripts/check_endpoint_clearance.py --log LOG --replay-team blue")
        else:
            print("Short streaks are a corridor closing and reopening, which is a result\n"
                  "rather than a fault. Read ms/build, not ms/call.")
    print()


ROW = re.compile(
    r"^(\S+)\s+(PRM|VisibilityGraph|Voronoi)\s+(\d+)/(\d+)\s+"
    r"([\d.]+)\s+([\d.]+)\s+([\d.]+)%\s+([\d.]+)\s+(\d+)\s+(\d+)"
)


def job2() -> None:
    rows = []
    for f in sorted(STABILITY.glob("*.txt")):
        scenario, speed = f.stem.rsplit("_speed", 1)
        for line in f.read_text(encoding="utf-8").splitlines():
            m = ROW.match(line.strip())
            if m:
                rows.append({
                    "scenario": scenario, "speed": float(speed),
                    "trigger": m.group(1), "planner": m.group(2),
                    "arrived": int(m.group(3)), "of": int(m.group(4)),
                    "replans": float(m.group(5)), "rev_rate": float(m.group(7)),
                    "rev_per_run": float(m.group(8)), "shift": int(m.group(9)),
                })
    if not rows:
        print("job 2: no parsable rows")
        return

    full = [r for r in rows if r["arrived"] == r["of"]]
    part = [r for r in rows if r["arrived"] < r["of"]]
    speeds = sorted({r["speed"] for r in rows})
    print(f"JOB 2  path stability against obstacle speed, offline, "
          f"{len(rows)} conditions over {len(speeds)} speeds "
          f"({speeds[0]:.1f}-{speeds[-1]:.1f} m/s), both scenarios, n=200\n")

    print(f"       conditions arriving in every run: {len(full)}")
    print(f"       conditions with any failure:      {len(part)}")
    if full and part:
        hi = max(r["rev_per_run"] for r in full)
        lo = min(r["rev_per_run"] for r in part)
        print(f"\n       highest reversals/run among always-arriving: {hi:.1f}")
        print(f"       lowest  reversals/run among failing:         {lo:.1f}")
        print(f"       -> {'separated, gap ' + format(lo - hi, '.1f') if lo > hi else 'OVERLAP: reversals/run no longer separates them'}")
        if lo <= hi:
            bad = [r for r in full if r["rev_per_run"] >= lo]
            print("       arriving conditions above the failing floor:")
            for r in sorted(bad, key=lambda r: -r["rev_per_run"])[:6]:
                print(f"         {r['scenario']} {r['speed']:.1f} m/s {r['planner']:<16}"
                      f"{r['trigger']:<26} {r['rev_per_run']:.1f}")

    print("\n       failing conditions, worst first:")
    for r in sorted(part, key=lambda r: r["arrived"])[:10]:
        print(f"         {r['scenario']} {r['speed']:.1f} m/s {r['planner']:<16}"
              f"{r['trigger']:<26} {r['arrived']:>3}/{r['of']} arrived, "
              f"{r['rev_per_run']:.1f} rev/run")


if __name__ == "__main__":
    job1()
    if len(sys.argv) <= 2:
        job2()
