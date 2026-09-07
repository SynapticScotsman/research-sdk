#!/usr/bin/env python3
"""Can a coarse fixed lattice of Voronoi sites still find the gaps robots use?

The incremental-repair argument wants Voronoi's virtual sites pinned to a fixed
lattice, because only then does the roadmap keep its shape when obstacles move.
Measured separately, that lattice has to be about 1200 mm coarse to build inside
a 50 ms control tick. Robots pass through gaps of 420 mm. So the lattice would be
three times coarser than the features it has to represent, and this asks whether
it still represents them.

## The test

A wall of robots spans the pitch with exactly one gap in it, so a path from one
side to the other either goes through that gap or does not exist. Sweep the gap
width, and sweep where the gap sits, because a lattice has a period and a gap
that lands on a site is not the same problem as one that lands between sites.
Measuring one gap position would measure one lucky alignment.

## Ground truth needs no planner

A robot centre must stay 210 mm from an obstacle centre: 90 mm for the obstacle,
90 mm for the robot, 30 mm of clearance. So a gap between two obstacles is
passable exactly when their centres are 420 mm apart or more. That is geometry,
not opinion, and it gives every trial a right answer to be scored against.

  miss   the gap is passable and the roadmap found no path. The failure this
         exists to look for.
  fault  the roadmap returned a path through a gap too narrow to fit, or a path
         that passes closer than 210 mm to an obstacle centre. Worse than a miss.

Usage:
    python scripts/measure_gap_resolution.py
    python scripts/measure_gap_resolution.py --positions 12 --step-mm 40
    python scripts/measure_gap_resolution.py --selftest
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))

from research_sdk.planners.common import (
    DEFAULT_ROBOT_RADIUS_MM,
    Obstacle,
    PlanRequest,
)
from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.planners.VisibilityGraph import visibility_graph
from research_sdk.world.map.geometry import distance_2_segment
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene

Point = tuple[float, float]

FIELD_HALF_Y = 3000.0
# The wall runs well past the touchlines. Stopping it at the field edge let
# the visibility graph route around the end and score a 300 mm gap as solved,
# which the selftest caught before any result was reported.
WALL_HALF_Y = 4600.0
CLEARANCE_MM = 210.0          # robot 90 + obstacle 90 + margin 30
PASSABLE_MM = 2 * CLEARANCE_MM  # 420: centre-to-centre for a gap to admit a robot

# Consecutive wall robots sit closer than PASSABLE_MM so the wall has no second
# way through. 400 mm leaves 20 mm of margin against that.
WALL_PITCH_MM = 400.0

START_X, GOAL_X = -3000.0, 3000.0

# Lattice spacings worth testing. 1200 mm is the coarsest that builds inside a
# 50 ms tick (11.3 ms on an eight-obstacle scene); 900 mm costs 27.5 ms; 600 mm
# costs 108.5 ms and is already over budget, included as an upper reference.
LATTICES_MM = (1200.0, 900.0, 600.0)


@dataclass(frozen=True)
class Trial:
    mode: str
    gap_mm: float
    gap_y: float
    passable: bool
    solved: bool
    min_clearance_mm: float

    @property
    def miss(self) -> bool:
        return self.passable and not self.solved

    @property
    def fault(self) -> bool:
        """Returned a path it should not have, or one that grazes an obstacle."""
        if not self.solved:
            return False
        return (not self.passable) or self.min_clearance_mm < CLEARANCE_MM - 1e-6


def wall_with_gap(gap_mm: float, gap_y: float) -> tuple[Obstacle, ...]:
    """A line of robots at x = 0 with one gap of `gap_mm` centred on `gap_y`."""
    ys: list[float] = []
    low, high = gap_y - gap_mm / 2.0, gap_y + gap_mm / 2.0
    y = low
    while y > -WALL_HALF_Y:
        ys.append(y)
        y -= WALL_PITCH_MM
    y = high
    while y < WALL_HALF_Y:
        ys.append(y)
        y += WALL_PITCH_MM
    return tuple(
        Obstacle(pos_mm=(0.0, yy), radius_mm=DEFAULT_ROBOT_RADIUS_MM, robot_id=i, isYellow=False)
        for i, yy in enumerate(sorted(ys))
    )


def crosses_in_gap(path: tuple[Point, ...], gap_mm: float, gap_y: float) -> bool:
    """True when every crossing of the wall line x = 0 lies inside the gap.

    Without this a path that goes round the end of the wall counts as a
    solution, which is how a 300 mm gap first scored as passable.
    """
    if len(path) < 2:
        return False
    crossed = False
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        if (ax <= 0.0 <= bx) or (bx <= 0.0 <= ax):
            if ax == bx:
                y = ay
            else:
                y = ay + (by - ay) * (0.0 - ax) / (bx - ax)
            if abs(y - gap_y) > gap_mm / 2.0 + 1e-6:
                return False
            crossed = True
    return crossed


def min_clearance(path: tuple[Point, ...], obstacles: tuple[Obstacle, ...]) -> float:
    """Closest approach of the path to any obstacle CENTRE, in mm."""
    if len(path) < 2:
        return float("inf")
    best = float("inf")
    for a, b in zip(path, path[1:]):
        for o in obstacles:
            best = min(best, distance_2_segment(o.pos_mm, a, b))
    return best


def _scene(obstacles: tuple[Obstacle, ...]) -> PlanningScene:
    return PlanningScene(
        timestamp=0.0,
        obstacles=tuple(
            PlanningObstacle(robot_id=o.robot_id, isYellow=o.isYellow,
                             pos_mm=o.pos_mm, radius_mm=o.radius_mm)
            for o in obstacles
        ),
        field=FieldDimensions(),
    )


def run_trial(mode: str, gap_mm: float, gap_y: float) -> Trial:
    obstacles = wall_with_gap(gap_mm, gap_y)
    start, goal = (START_X, gap_y), (GOAL_X, gap_y)
    passable = gap_mm >= PASSABLE_MM - 1e-6

    if mode == "visibility":
        result = visibility_graph.plan(
            PlanRequest(start_mm=start, goal_mm=goal, obstacles=obstacles),
            skip_direct_path=True,
        )
        path = result.waypoints_mm if result.success else ()
    else:
        if mode.startswith("grid"):
            planner = VoronoiDijkstraPlanner(
                placement_mode="grid", grid_spacing_mm=float(mode.split("@")[1])
            )
        else:
            planner = VoronoiDijkstraPlanner(placement_mode="density_grid")
        result = planner.plan(_scene(obstacles), start, goal, skip_direct_path=True)
        path = (start, *result.waypoints_mm) if result.waypoints_mm else ()

    path = tuple(path)
    return Trial(
        mode=mode, gap_mm=gap_mm, gap_y=gap_y, passable=passable,
        solved=crosses_in_gap(path, gap_mm, gap_y),
        min_clearance_mm=min_clearance(path, obstacles),
    )


def selftest() -> int:
    """Pin the instrument before trusting anything it reports."""
    failures = []

    # The wall must actually be a wall: no unintended second gap.
    obs = wall_with_gap(600.0, 0.0)
    ys = sorted(o.pos_mm[1] for o in obs)
    gaps = [b - a for a, b in zip(ys, ys[1:])]
    wide = [g for g in gaps if g >= PASSABLE_MM - 1e-6]
    if len(wide) != 1 or abs(wide[0] - 600.0) > 1e-6:
        failures.append(f"expected exactly one 600 mm gap, found {wide}")
    if ys[0] > -FIELD_HALF_Y or ys[-1] < FIELD_HALF_Y:
        failures.append(f"wall does not reach the field edges: {ys[0]:.0f} to {ys[-1]:.0f}")

    # The gap must sit where it was asked to sit.
    obs = wall_with_gap(500.0, 900.0)
    ys = sorted(o.pos_mm[1] for o in obs)
    pair = [(a, b) for a, b in zip(ys, ys[1:]) if b - a >= PASSABLE_MM - 1e-6]
    if len(pair) != 1 or abs((pair[0][0] + pair[0][1]) / 2 - 900.0) > 1e-6:
        failures.append(f"gap not centred on 900: {pair}")

    # Exact geometry must solve a passable gap and refuse an impassable one.
    # If the reference backend cannot do this the test measures nothing.
    ok = run_trial("visibility", 600.0, 0.0)
    if not ok.solved:
        failures.append("visibility graph failed a 600 mm gap, which is passable")
    if ok.solved and ok.min_clearance_mm < CLEARANCE_MM - 1e-6:
        failures.append(f"visibility path grazed at {ok.min_clearance_mm:.1f} mm")
    blocked = run_trial("visibility", 300.0, 0.0)
    if blocked.solved:
        failures.append("visibility graph found a path through a 300 mm gap")

    for line in failures:
        print(f"FAIL {line}")
    if not failures:
        print("gap resolution selftest OK")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--min-gap", type=float, default=300.0)
    parser.add_argument("--max-gap", type=float, default=1200.0)
    parser.add_argument("--step-mm", type=float, default=60.0)
    parser.add_argument("--positions", type=int, default=6,
                        help="Gap positions swept across one 1200 mm lattice period")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    widths = []
    w = args.min_gap
    while w <= args.max_gap + 1e-6:
        widths.append(w)
        w += args.step_mm
    positions = [1200.0 * i / args.positions for i in range(args.positions)]
    modes = ["visibility", "density_grid"] + [f"grid@{s:.0f}" for s in LATTICES_MM]

    print(f"\nWall across the pitch, one gap. {len(widths)} widths x "
          f"{len(positions)} positions per mode.")
    print(f"A gap is passable at {PASSABLE_MM:.0f} mm centre to centre and above.\n")

    trials: dict[str, list[Trial]] = {m: [] for m in modes}
    for mode in modes:
        for gap in widths:
            for y in positions:
                try:
                    trials[mode].append(run_trial(mode, gap, y))
                except Exception as exc:  # noqa: BLE001
                    print(f"  {mode} {gap:.0f}mm @{y:.0f}: {type(exc).__name__}: {exc}")

    header = (f"{'roadmap':<16}{'solved':>10}{'misses':>9}{'faults':>8}"
              f"{'first solid':>13}{'min clear mm':>14}")
    print(header)
    print("-" * len(header))
    for mode in modes:
        ts = trials[mode]
        passable = [t for t in ts if t.passable]
        solved = sum(t.solved for t in passable)
        misses = sum(t.miss for t in ts)
        faults = sum(t.fault for t in ts)
        # Narrowest gap solved at EVERY position tested: the width below which
        # the roadmap's answer depends on where the gap happens to fall.
        solid = None
        for gap in widths:
            at = [t for t in ts if abs(t.gap_mm - gap) < 1e-6]
            if at and all(t.solved for t in at) and at[0].passable:
                solid = gap
                break
        clears = [t.min_clearance_mm for t in ts if t.solved and math.isfinite(t.min_clearance_mm)]
        print(
            f"{mode:<16}{solved:>4}/{len(passable):<5}{misses:>9}{faults:>8}"
            f"{(f'{solid:.0f} mm' if solid else 'never'):>13}"
            f"{(min(clears) if clears else float('nan')):>14.1f}"
        )

    print("\nsolved      = passable gaps for which a path was returned.")
    print("misses      = passable gaps the roadmap could not find a way through.")
    print("faults      = paths returned through a gap too narrow, or passing")
    print(f"              closer than {CLEARANCE_MM:.0f} mm to an obstacle centre.")
    print("first solid = narrowest gap solved at every position tested. Below it")
    print("              the answer depends on where the gap falls in the lattice.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
