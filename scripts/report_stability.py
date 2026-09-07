#!/usr/bin/env python3
"""Does the robot keep changing its mind, and does it turn around when it does?

Runs the dynamic scenario across every replanning trigger and every planner,
and reports the two path-stability numbers from path_stability.py alongside the
arrival rate they are meant to explain.

The claim under test is the draft's own explanation of why the Voronoi roadmap
fails when told to replan on a timer: "each replan from a slightly advanced
position produces a different route, and the robot oscillates between them
without converging." If that is right, the Voronoi rows under a timer show a
high reversal rate and a large corridor shift, and the same rows under an
event trigger do not.

Usage:
    python scripts/report_stability.py --samples 40 --workers 8
    python scripts/report_stability.py --scenario "scenario_2 (5 obstacles)"
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from dynamic_scenario import SCENARIOS, sweep_triggers


def pooled(runs, field: str) -> float:
    """Mean over every replan, not mean of per-run means.

    A run with 500 replans and a run with 3 must not carry equal weight when
    the question is what a replan typically does.
    """
    weight = sum(r.replans_compared for r in runs)
    if not weight:
        return float("nan")
    return sum(getattr(r, field) * r.replans_compared for r in runs) / weight


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", default="scenario_1 (7 obstacles)",
                        choices=sorted(SCENARIOS))
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--obstacle-speed", type=float, default=1.0)
    parser.add_argument("--robot-speed", type=float, default=1.5)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()

    results = sweep_triggers(
        args.scenario, samples=args.samples,
        obstacle_speed=args.obstacle_speed, robot_speed=args.robot_speed,
        workers=args.workers,
    )

    print(f"\n{args.scenario} - {args.samples} samples, obstacles at "
          f"{args.obstacle_speed:.1f} m/s, robot at {args.robot_speed:.1f} m/s\n")
    header = (
        f"{'trigger':<26}{'planner':<14}{'arrived':>9}{'replans':>9}"
        f"{'turn deg':>10}{'reversals':>11}{'rev/run':>9}{'shift mm':>10}{'p95 mm':>9}"
    )
    print(header)
    print("-" * len(header))

    last_trigger = None
    for (trigger, planner), runs in results.items():
        if last_trigger is not None and trigger != last_trigger:
            print()
        last_trigger = trigger
        arrived = sum(r.arrived for r in runs)
        replans = sum(r.recalculated for r in runs) / len(runs)
        compared = sum(r.replans_compared for r in runs)
        reversals = sum(r.reversals for r in runs)
        rate = reversals / compared if compared else float("nan")
        print(
            f"{trigger:<26}{planner.replace('+Dijkstra', ''):<14}"
            f"{arrived:>4}/{len(runs):<4}{replans:>9.1f}"
            f"{pooled(runs, 'mean_heading_deg'):>10.1f}"
            f"{rate * 100:>10.1f}%"
            f"{reversals / len(runs):>9.1f}"
            f"{pooled(runs, 'mean_shift_mm'):>10.0f}"
            f"{pooled(runs, 'p95_shift_mm'):>9.0f}"
        )

    print(
        "\nturn deg  = angle between the direction the old plan pointed and the\n"
        "            direction the replacement points, from where the robot is.\n"
        "            0 means the replan did not change the next move.\n"
        "reversals = share of replans turning more than 90 degrees, so the new\n"
        "            plan sends the robot back the way it came.\n"
        "rev/run   = reversals per run. This is the one that tracks arrival:\n"
        "            the rate alone does not, because a planner that reverses\n"
        "            half the time but replans three times per run reverses\n"
        "            once or twice in total and still gets there.\n"
        "shift mm  = mean distance between the old route and the new one over\n"
        "            their whole length. 0 means the route did not move.\n"
        "Both read 0 for a robot that never changes its mind."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
