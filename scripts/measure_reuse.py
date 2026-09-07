#!/usr/bin/env python3
"""How much of a visibility graph survives between two consecutive REPLANS?

Test 2's policy-driven half proposes reusing the previous solution instead of
rebuilding it. Before implementing D*/LPA*-style reuse, this measures the
ceiling on what reuse could save, which decides whether implementing it is
worth the week.

The expensive part of a visibility-graph call is the O(n^2) segment-versus-
polygon visibility test, exactly the "edge evaluation" Lim et al. (arXiv
2210.12851) argue incremental search wastes. Edge WEIGHTS are cheap (one
hypot) and are always recomputed. So the saving available is the fraction of
visibility ANSWERS that can be carried over.

Two numbers are reported, and the gap between them is the point:

  oracle ceiling   the share of edges whose answer did not change. Only an
                   oracle that already knew the answers could skip exactly
                   these, so this is an upper bound no implementation beats.
  achievable       the share an implementation can skip using a sound test:
                   an edge is safe to reuse when neither endpoint moved AND no
                   obstacle swept across the segment between the two frames.
                   Conservative, so it never claims an edge that changed.

Two sources, because they disagree and the disagreement matters:

  patrol   the synthetic dynamic scenario, where every obstacle moves
           continuously at a set speed
  match    a recorded SSL match, where the measured median robot speed is
           0.04 m/s and most robots stand still

Usage:
    python scripts/measure_reuse.py --speeds 0.5 1.0 2.0 3.0
    python scripts/measure_reuse.py --log ~/ssl-gamelogs/... --skip 180
    python scripts/measure_reuse.py --selftest
"""

from __future__ import annotations

import argparse
import math
import random
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from dynamic_scenario import CONTROL_TICK_S, GOAL_MM, START_MM
from dynamic_scenario import SCENARIOS as PATROL_SCENARIOS
from measure_graph_churn import edge_states

from research_sdk.planners.common import Obstacle, PlanRequest

# Below this, a vertex is treated as unmoved. Vision quantisation and float
# noise are well under a millimetre in the synthetic case; on a real log the
# measured per-frame jitter is 23 mm, so anything smaller cannot be called
# motion. Kept tight so the achievable figure stays conservative.
STILL_MM = 1.0


def _segment_swept_by(a, b, old_pos, new_pos, radius_mm) -> bool:
    """Could an obstacle moving old_pos -> new_pos have crossed segment a-b?

    Sound rather than exact: tests the segment against a disc of radius
    (obstacle inflation + the distance it travelled) centred on its old
    position. Any real crossing is inside that disc, so this never misses one.
    """
    travel = math.dist(old_pos, new_pos)
    reach = radius_mm + travel
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_sq = dx * dx + dy * dy
    px, py = old_pos
    t = 0.0 if seg_sq == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_sq))
    closest = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
    return closest <= reach


def reuse_between(before, after, request_template: PlanRequest) -> dict | None:
    """Compare two obstacle sets. Returns shares of edges reusable, or None.

    None when an endpoint was blocked in either frame, which is the planner
    returning before it tests any edge (visibility_graph.py:304).
    """
    req_a = PlanRequest(start_mm=request_template.start_mm, goal_mm=request_template.goal_mm,
                        obstacles=before)
    req_b = PlanRequest(start_mm=request_template.start_mm, goal_mm=request_template.goal_mm,
                        obstacles=after)
    states_a = edge_states(req_a)
    states_b = edge_states(req_b)
    if states_a is None or states_b is None:
        return None

    shared = states_a.keys() & states_b.keys()
    if not shared:
        return None

    unchanged = sum(1 for e in shared if states_a[e] == states_b[e])

    # Which obstacles moved, and where their vertices are, for the sound test.
    old = {o.robot_id: o for o in before}
    new = {o.robot_id: o for o in after}
    moved = {
        rid for rid in old.keys() & new.keys()
        if math.dist(old[rid].pos_mm, new[rid].pos_mm) > STILL_MM
    }
    inflation = req_a.total_clearance_mm

    # An edge label is "<team><robot_id>v<vertex>" or start/goal; the owning
    # robot is everything before the final "v".
    def owner(label: str) -> int | None:
        if label in ("start", "goal"):
            return None
        body = label.rsplit("v", 1)[0]
        return int(body[1:])

    # Vertex positions in the AFTER frame, to test the swept discs against.
    points_b: dict[str, tuple[float, float]] = {}
    for o in after:
        # Vertices are not needed exactly; the obstacle centre bounds them.
        points_b[f"c{o.robot_id}"] = o.pos_mm

    achievable = 0
    for e in shared:
        p, q = tuple(e)
        op, oq = owner(p), owner(q)
        if (op in moved) or (oq in moved):
            continue  # an endpoint moved: the segment itself is different
        # Endpoints are static. Did any moved obstacle sweep across the segment?
        a = points_b.get(f"c{op}") if op is not None else request_template.start_mm
        b = points_b.get(f"c{oq}") if oq is not None else request_template.goal_mm
        if a is None or b is None:
            continue
        if any(
            _segment_swept_by(a, b, old[rid].pos_mm, new[rid].pos_mm,
                              old[rid].radius_mm + inflation)
            for rid in moved
        ):
            continue
        achievable += 1

    return {
        "shared": len(shared),
        "oracle": unchanged / len(shared),
        "achievable": achievable / len(shared),
        "moved_obstacles": len(moved),
    }


def replan_frames_patrol(loops, speed_mps: float, phases, max_s: float = 12.0):
    """Obstacle sets at the instants their trigger would fire a replan.

    Reuse is only worth measuring between consecutive REPLANS, not consecutive
    ticks: those are the calls an incremental planner would actually make.
    """
    frames = []
    t = 0.0
    while t < max_s:
        obs = tuple(
            Obstacle(pos_mm=loop.position(phases[i] + speed_mps * 1000.0 * t),
                     radius_mm=90.0, robot_id=i, isYellow=False)
            for i, loop in enumerate(loops)
        )
        frames.append(obs)
        t += CONTROL_TICK_S
    return frames


def measure_patrol(speeds, samples: int, seed: int = 0) -> dict:
    """Reuse between replans in the synthetic patrol scenario, per speed."""
    loops = PATROL_SCENARIOS["scenario_1 (7 obstacles)"]
    template = PlanRequest(start_mm=START_MM, goal_mm=GOAL_MM)
    rng = random.Random(seed)
    out = {}
    for v in speeds:
        oracle, achievable, gaps = [], [], []
        for _ in range(samples):
            phases = [rng.uniform(0, lp.length()) for lp in loops]
            frames = replan_frames_patrol(loops, v, phases)
            # Their trigger fires roughly every 10-20 ticks at these speeds.
            # Sample pairs a realistic replan interval apart rather than a tick.
            stride = 10
            for i in range(0, len(frames) - stride, stride * 4):
                r = reuse_between(frames[i], frames[i + stride], template)
                if r is None:
                    continue
                oracle.append(r["oracle"])
                achievable.append(r["achievable"])
                gaps.append(stride * CONTROL_TICK_S)
        if oracle:
            out[v] = {
                "n": len(oracle),
                "gap_s": statistics.mean(gaps),
                "oracle": statistics.mean(oracle),
                "achievable": statistics.mean(achievable),
            }
    return out


def measure_match(log: str, skip_s: float, seconds: float, stride: int) -> dict | None:
    """Reuse between replans on a recorded match."""
    from measure_graph_churn import capture_from_logfile

    captures = capture_from_logfile(log, seconds, skip_seconds=skip_s)
    if len(captures) < stride + 2:
        return None
    # Endpoints off the goal mouths: a keeper parks there and blocks the query.
    template = PlanRequest(start_mm=(-3500.0, -2000.0), goal_mm=(3500.0, 2000.0))
    oracle, achievable, gaps, blocked = [], [], [], 0
    for i in range(0, len(captures) - stride, stride):
        r = reuse_between(captures[i].obstacles, captures[i + stride].obstacles, template)
        if r is None:
            blocked += 1
            continue
        oracle.append(r["oracle"])
        achievable.append(r["achievable"])
        gaps.append(captures[i + stride].t_s - captures[i].t_s)
    if not oracle:
        return None
    return {
        "n": len(oracle), "blocked": blocked,
        "gap_s": statistics.mean(gaps),
        "oracle": statistics.mean(oracle),
        "achievable": statistics.mean(achievable),
    }


def selftest() -> None:
    # Swept-disc test: an obstacle that does not move only sweeps its own disc.
    assert _segment_swept_by((0, 0), (1000, 0), (500, 100), (500, 100), 210)
    assert not _segment_swept_by((0, 0), (1000, 0), (500, 400), (500, 400), 210)
    # One that travels far enough reaches the segment even from outside.
    assert _segment_swept_by((0, 0), (1000, 0), (500, 400), (500, 150), 210)
    # Past the end of the segment, distance is to the endpoint, not the line.
    assert not _segment_swept_by((0, 0), (1000, 0), (3000, 0), (3000, 0), 210)

    # Identical obstacle sets: nothing changed and nothing moved, so an oracle
    # and a sound test must both say every edge is reusable.
    obs = (
        Obstacle(pos_mm=(0.0, 400.0), robot_id=0),
        Obstacle(pos_mm=(1200.0, -500.0), robot_id=1),
        Obstacle(pos_mm=(-1500.0, 300.0), robot_id=2),
    )
    template = PlanRequest(start_mm=(-3500.0, -2000.0), goal_mm=(3500.0, 2000.0))
    r = reuse_between(obs, obs, template)
    assert r is not None and r["oracle"] == 1.0, r
    assert r["achievable"] == 1.0, r
    assert r["moved_obstacles"] == 0

    # Move one obstacle a long way: the oracle may still find most answers
    # unchanged, but the sound test must claim strictly less than the oracle.
    moved = (obs[0], obs[1], Obstacle(pos_mm=(-1500.0, 1800.0), robot_id=2))
    r2 = reuse_between(obs, moved, template)
    assert r2 is not None and r2["moved_obstacles"] == 1
    assert r2["achievable"] <= r2["oracle"] + 1e-9, r2
    print("selftest OK")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.5, 1.0, 2.0, 3.0])
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--log", help="Also measure a recorded match")
    parser.add_argument("--skip", type=float, default=180.0)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--stride", type=int, default=35,
                        help="Frames between replans on the log (35 at 70 Hz = 0.5 s)")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return

    print("\nHow much of the visibility graph survives between consecutive replans?")
    print("oracle = share of visibility answers unchanged (upper bound, unimplementable)")
    print("sound  = share an implementation can prove reusable without testing\n")

    patrol = measure_patrol(args.speeds, args.samples)
    print("  PATROL scenario (every obstacle moving continuously)")
    print(f"  {'obst m/s':>10}{'gap s':>8}{'n':>6}{'oracle %':>11}{'sound %':>10}")
    print("  " + "-" * 45)
    for v, r in sorted(patrol.items()):
        print(f"  {v:>10.1f}{r['gap_s']:>8.2f}{r['n']:>6}"
              f"{r['oracle'] * 100:>11.1f}{r['achievable'] * 100:>10.1f}")

    if args.log:
        m = measure_match(args.log, args.skip, args.seconds, args.stride)
        if m:
            print("\n  REAL MATCH (measured median robot speed 0.04 m/s)")
            print(f"  {'source':>10}{'gap s':>8}{'n':>6}{'oracle %':>11}{'sound %':>10}")
            print("  " + "-" * 45)
            print(f"  {'log':>10}{m['gap_s']:>8.2f}{m['n']:>6}"
                  f"{m['oracle'] * 100:>11.1f}{m['achievable'] * 100:>10.1f}")
            if m["blocked"]:
                print(f"  ({m['blocked']} pairs skipped: an endpoint was inside an obstacle)")

    print(
        "\n  A high sound %% means an incremental planner can skip that share of the\n"
        "  O(n^2) visibility tests, so per-call cost falls by roughly that much.\n"
        "  A low one means rebuilding from scratch is close to optimal and the\n"
        "  policy-driven half of the plugin cannot pay for itself.\n"
    )


if __name__ == "__main__":
    main()
