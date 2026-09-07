"""How much of the UI thread does planning eat, per planner?

Written to diagnose a report that the execution page was unusable. It measures
time inside plan() per vision frame for six robots, as a share of the 14.3 ms
frame budget; above 100% the Qt thread cannot keep up and every repaint queues
behind planning.

Obstacles MOVE here, which is the whole point. An earlier version used a static
scene, the reroute gate short-circuited every call, and it reported ~0.03 ms and
no problem at all. The UI feeds a new vision frame every ~14 ms with obstacles in
new positions, and that is when the gate decides a reroute is needed and the full
graph rebuild lands on the UI thread.

Measured at 7060bf4: 182% for PRM and 105% for the visibility graph with the gate
on. At e0f0232, after the OrientedRectangle geometry change and a replan rate cap,
the same probe reads 24% and 32%.
"""
import statistics
import sys
from pathlib import Path
from math import cos, sin
from time import perf_counter

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from research_sdk.planners.common import DEFAULT_ROBOT_RADIUS_MM
from research_sdk.planners.Dijkstra.waypoint_manager import PlannerInput
from research_sdk.ui.session import discover_planners
from research_sdk.world.scene import PlanningObstacle, PlanningScene

VISION_HZ = 70.0
REPLAN_HZ = 20.0  # the 50 ms execution timer
ROBOTS = 6
FRAMES = 120
SPEED_MM_S = 1500.0  # a realistic SSL robot speed

BASE = [(-1000, 400), (500, -700), (1500, 900), (-2000, -1200),
        (2500, 300), (0, 1500), (-500, -1800), (3000, -900)]


def scene_at(frame: int) -> PlanningScene:
    t = frame / VISION_HZ
    obstacles = tuple(
        PlanningObstacle(
            robot_id=i, isYellow=False,
            pos_mm=(x + SPEED_MM_S * t * cos(i), y + SPEED_MM_S * t * sin(i * 1.7)),
            radius_mm=DEFAULT_ROBOT_RADIUS_MM,
        )
        for i, (x, y) in enumerate(BASE)
    )
    return PlanningScene(timestamp=frame / VISION_HZ, obstacles=obstacles)


start = (-3500.0, -1500.0, 0.0)
target = (3500.0, 1500.0, 0.0)

print(f"{ROBOTS} robots, {FRAMES} vision frames at {VISION_HZ:.0f} Hz, "
      f"obstacles moving at {SPEED_MM_S / 1000:.1f} m/s\n")
print(f"{'planner':<26}{'gate':>6}{'median ms':>11}{'p95 ms':>9}{'worst ms':>10}"
      f"{'reroutes':>10}{'@70Hz':>9}{'@20Hz':>10}")
print("-" * 81)

for cls in (v for v in discover_planners().values() if isinstance(v, type)):
    for gate in (True, False):
        try:
            planner = cls(use_reroute_gate=gate)
        except TypeError:
            continue

        times, reroutes = [], 0
        try:
            for f in range(FRAMES):
                sc = scene_at(f)
                for r in range(ROBOTS):
                    t0 = perf_counter()
                    result = planner.plan(PlannerInput(
                        robot_id=r, is_yellow=True, current_pose=start,
                        target_pose=target, scene=sc,
                    ))
                    times.append((perf_counter() - t0) * 1000.0)
                    if getattr(result, "did_reroute", False):
                        reroutes += 1
        except Exception as exc:  # noqa: BLE001
            print(f"{cls.__name__:<26}{str(gate):>6}  failed: {type(exc).__name__}: {exc}")
            continue

        times.sort()
        med = statistics.median(times)
        p95 = times[int(len(times) * 0.95)]
        # Per FRAME the UI thread pays ROBOTS calls; the frame budget is 1/70 s.
        per_frame_ms = sum(times) / FRAMES
        load = per_frame_ms / (1000.0 / VISION_HZ) * 100.0
        # Same work, driven at 20 Hz instead of 70 Hz.
        load_50ms = per_frame_ms / (1000.0 / VISION_HZ) * 100.0 * (REPLAN_HZ / VISION_HZ)
        print(f"{cls.__name__:<26}{str(gate):>6}{med:>11.3f}{p95:>9.2f}{times[-1]:>10.2f}"
              f"{reroutes:>10}{load:>8.0f}%{load_50ms:>10.0f}%")

print("\nUI load = time inside plan() per vision frame, as a share of the 14.3 ms")
print("frame budget, for 6 robots. Above 100% the UI thread cannot keep up and")
print("every repaint queues behind planning.")
