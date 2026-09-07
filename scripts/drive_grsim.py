#!/usr/bin/env python3
"""Drive grSim from the planners, headless, so you can watch them work.

Places a blue team on one touchline and sends it across the pitch and back,
planning every control tick with whichever backend you pick. The yellow team
patrols across the middle as moving obstacles, so the blue robots have to keep
replanning around traffic rather than driving a straight line once.

Everything the study measures is printed live while it runs: replans, how often
a replan reverses the robot, planning time, and closest approach to another
robot. Watching the pitch shows you the behaviour; the numbers say whether it is
the behaviour the paper claims.

Run grSim first. Both this and grSim must be on the same side of the WSL
boundary, because WSL2 NAT does not carry multicast to Windows.

Usage:
    python scripts/drive_grsim.py --planner voronoi
    python scripts/drive_grsim.py --planner visibility --robots 3 --duration 60
    python scripts/drive_grsim.py --planner prm --trigger periodic
    python scripts/drive_grsim.py --selftest
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from path_stability import Stability

from research_sdk.network.command_dispatcher import RobotCommandDispatcher
from research_sdk.network.grSimPacketFactory import grSimPacketFactory
from research_sdk.network.robot_command import RobotCommand
from research_sdk.network.ssl_sockets import grSimSender, grSimVision
from research_sdk.planners.common import (
    DEFAULT_ROBOT_RADIUS_MM,
    Obstacle,
    PlanRequest,
)
from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.planners.PRM import prm_dijkstra
from research_sdk.planners.VisibilityGraph import visibility_graph
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene

Point = tuple[float, float]

CONTROL_TICK_S = 0.05        # ui/execution/page.py's own execution timer

# grSim does NOT hold a velocity command between packets. Measured on a robot
# commanded at a steady 1.2 m/s for 3 s: sending once per 50 ms control tick
# moved it 22 mm, sending at 50 Hz moved it 618 mm, and neither approached the
# 3600 mm the command asks for. The robot accelerates while commands arrive and
# coasts down between them. So planning runs at the control tick and delivery
# runs far faster, through the SDK's own RobotCommandDispatcher, which repeats
# the latest command and zeroes it after command_ttl_s of silence.
COMMAND_HZ = 100.0
ARRIVED_MM = 150.0           # close enough to call it, and to turn around
CONTACT_MM = 2 * DEFAULT_ROBOT_RADIUS_MM   # centre distance at which robots touch
INVALIDATION_MM = 90.0       # the source study's replan trigger threshold

# Keep well inside the touchlines: a robot commanded past them piles into the
# wall and stops, which looks like a planner failure and is not one.
START_X, GOAL_X = -3200.0, 3200.0
LANE_SPACING_MM = 900.0

# Yellow robots cross the middle at this speed, reversing at the edges. Moving
# obstacles are the whole point; against a static scene every planner looks the
# same and no trigger ever fires twice.
OBSTACLE_SPEED_MPS = 0.7
OBSTACLE_TURN_Y_MM = 1900.0


def _prm(request, key):
    return prm_dijkstra.plan(request, skip_direct_path=False, search_key=key)


def _visibility(request, key):
    return visibility_graph.plan(request, skip_direct_path=False, search_key=key)


PLANNERS = {
    "prm": _prm,
    "visibility": _visibility,
    # Voronoi takes a scene rather than a PlanRequest, so it is adapted in
    # `plan_for` rather than here.
    "voronoi": None,
}


@dataclass
class Robot:
    """One blue robot under our control."""

    robot_id: int
    target: Point
    path: tuple[Point, ...] = ()
    stability: Stability = field(default_factory=Stability)
    replans: int = 0
    plan_ms_total: float = 0.0
    laps: int = 0
    closest_mm: float = float("inf")


def rotate_into_robot_frame(ex: float, ey: float, theta: float) -> tuple[float, float]:
    """World-frame error to the robot's own axes.

    grSim reads veltangent and velnormal in the robot's frame, so a world-frame
    velocity sent unrotated drives every robot in a different direction
    depending on which way it happens to be facing.
    """
    c, s = math.cos(theta), math.sin(theta)
    return c * ex + s * ey, -s * ex + c * ey


def path_blocked(path, obstacles, threshold_mm: float = INVALIDATION_MM) -> bool:
    """The source study's trigger: any obstacle within 90 mm of the path."""
    if len(path) < 2:
        return False
    for o in obstacles:
        for a, b in zip(path, path[1:]):
            if _point_to_segment(o.pos_mm, a, b) <= o.radius_mm + threshold_mm:
                return True
    return False


def _point_to_segment(p: Point, a: Point, b: Point) -> float:
    px, py = p
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    seg = dx * dx + dy * dy
    t = 0.0 if seg == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class Vision:
    """Latest pose per robot, kept current by draining every queued frame.

    The drain is not an optimisation. grSim emits four camera streams at ~60 Hz
    each and a 50 ms tick leaves a dozen frames waiting; reading one of them
    gives a pose from the previous tick, which reads as a robot that will not
    move. That mistake has cost this project two debugging sessions.
    """

    def __init__(self) -> None:
        self.sock = grSimVision(threading.Event())
        self.sock.is_running.set()
        self.blue: dict[int, tuple[float, float, float]] = {}
        self.yellow: dict[int, tuple[float, float, float]] = {}

    def update(self) -> int:
        self.sock.sock.setblocking(False)
        frames = 0
        try:
            while True:
                packet = self.sock.listen()
                if packet is None or not packet.HasField("detection"):
                    continue
                frames += 1
                for r in packet.detection.robots_blue:
                    self.blue[r.robot_id] = (r.x, r.y, r.orientation)
                for r in packet.detection.robots_yellow:
                    self.yellow[r.robot_id] = (r.x, r.y, r.orientation)
        except (BlockingIOError, OSError):
            pass
        finally:
            self.sock.sock.setblocking(True)
        return frames

    def wait_for_robots(self, blue: int, yellow: int, timeout_s: float = 5.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.update()
            if len(self.blue) >= blue and len(self.yellow) >= yellow:
                return True
            time.sleep(0.05)
        return False


def obstacles_for(vision: Vision, skip_blue: int) -> tuple[Obstacle, ...]:
    """Every other robot on the pitch, both teams."""
    out = []
    for rid, (x, y, _t) in vision.blue.items():
        if rid == skip_blue:
            continue
        out.append(Obstacle(pos_mm=(x, y), radius_mm=DEFAULT_ROBOT_RADIUS_MM,
                            robot_id=rid, isYellow=False))
    for rid, (x, y, _t) in vision.yellow.items():
        out.append(Obstacle(pos_mm=(x, y), radius_mm=DEFAULT_ROBOT_RADIUS_MM,
                            robot_id=rid, isYellow=True))
    return tuple(out)


def plan_for(planner: str, start: Point, goal: Point,
             obstacles: tuple[Obstacle, ...], key) -> tuple[tuple[Point, ...], float]:
    """One planning call. Returns the waypoints and how long it took, in ms."""
    t0 = time.perf_counter()
    if planner == "voronoi":
        scene = PlanningScene(
            timestamp=0.0,
            obstacles=tuple(
                PlanningObstacle(robot_id=o.robot_id, isYellow=o.isYellow,
                                 pos_mm=o.pos_mm, radius_mm=o.radius_mm)
                for o in obstacles
            ),
            field=FieldDimensions(),
        )
        result = VoronoiDijkstraPlanner().plan(scene, start, goal, search_key=key)
        waypoints = tuple(result.waypoints_mm)
    else:
        request = PlanRequest(start_mm=start, goal_mm=goal, obstacles=obstacles)
        result = PLANNERS[planner](request, key)
        # The visibility graph and PRM include the start as their first
        # waypoint; Voronoi does not. Drop it so every backend hands back the
        # same thing: where to go next.
        waypoints = tuple(result.waypoints_mm)[1:] if result.success else ()
    return waypoints, (time.perf_counter() - t0) * 1000.0


def place_teams(sender: grSimSender, blue: int, yellow: int) -> None:
    """Line the blue team up on one touchline, the yellow team across the middle."""
    robots = []
    for i in range(blue):
        robots.append({
            "x": START_X / 1000.0,
            "y": (i - (blue - 1) / 2.0) * LANE_SPACING_MM / 1000.0,
            "orientation": 0.0, "robot_id": i, "isYellow": False,
        })
    for i in range(yellow):
        robots.append({
            "x": (-1200.0 + i * 1200.0) / 1000.0,
            "y": (OBSTACLE_TURN_Y_MM if i % 2 else -OBSTACLE_TURN_Y_MM) / 1000.0,
            "orientation": 0.0, "robot_id": i, "isYellow": True,
        })
    sender.send_packet(grSimPacketFactory.scenario_replacement_command(robots))


def drive_obstacles(dispatcher: RobotCommandDispatcher, vision: Vision, yellow: int) -> None:
    """Yellow robots shuttle up and down, turning at the edges."""
    for rid in range(yellow):
        pose = vision.yellow.get(rid)
        if pose is None:
            continue
        _x, y, theta = pose
        direction = -1.0 if y > OBSTACLE_TURN_Y_MM else (1.0 if y < -OBSTACLE_TURN_Y_MM else None)
        if direction is None:
            direction = 1.0 if (rid % 2 == 0) else -1.0
            direction = direction if abs(y) < OBSTACLE_TURN_Y_MM else -direction
        vx, vy = rotate_into_robot_frame(0.0, direction * OBSTACLE_SPEED_MPS, theta)
        dispatcher.publish(
            RobotCommand(robot_id=rid, vx=vx, vy=vy, w=0.0, isYellow=True)
        )


def selftest() -> int:
    """Pin the parts that do not need a running simulator."""
    failures = []

    # A robot facing +y must be told to drive along its own -x to move world +x.
    vx, vy = rotate_into_robot_frame(1.0, 0.0, math.pi / 2)
    if abs(vx) > 1e-9 or abs(vy + 1.0) > 1e-9:
        failures.append(f"frame rotation wrong at 90 deg: got ({vx:.3f}, {vy:.3f})")
    vx, vy = rotate_into_robot_frame(1.0, 0.0, 0.0)
    if abs(vx - 1.0) > 1e-9 or abs(vy) > 1e-9:
        failures.append(f"frame rotation wrong at 0 deg: got ({vx:.3f}, {vy:.3f})")

    # The trigger has to fire on an obstacle sitting on the path and not on one
    # far away, or every run replans either always or never.
    path = ((0.0, 0.0), (2000.0, 0.0))
    near = (Obstacle(pos_mm=(1000.0, 150.0), radius_mm=90.0, robot_id=1, isYellow=True),)
    far = (Obstacle(pos_mm=(1000.0, 900.0), radius_mm=90.0, robot_id=1, isYellow=True),)
    if not path_blocked(path, near):
        failures.append("trigger missed an obstacle 150 mm off the path")
    if path_blocked(path, far):
        failures.append("trigger fired on an obstacle 900 mm off the path")

    # Every backend must return waypoints that do not start on the robot, so the
    # controller does not chase the point it is already standing on.
    obstacles = (Obstacle(pos_mm=(0.0, 0.0), radius_mm=90.0, robot_id=9, isYellow=True),)
    for name in PLANNERS:
        waypoints, ms = plan_for(name, (-2000.0, 0.0), (2000.0, 0.0), obstacles, key=("test", 0))
        if waypoints and math.dist(waypoints[0], (-2000.0, 0.0)) < 1.0:
            failures.append(f"{name}: first waypoint is the robot's own position")
        if ms <= 0.0:
            failures.append(f"{name}: planning time not measured")

    for line in failures:
        print(f"FAIL {line}")
    if not failures:
        print("drive_grsim selftest OK")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--planner", choices=sorted(PLANNERS), default="voronoi")
    parser.add_argument("--robots", type=int, default=3, help="Blue robots to drive")
    parser.add_argument("--obstacles", type=int, default=4, help="Yellow robots to patrol")
    parser.add_argument("--speed", type=float, default=1.2, help="Blue robot speed, m/s")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--trigger", choices=("geometric", "periodic", "always"),
                        default="geometric")
    parser.add_argument("--period-ms", type=float, default=500.0,
                        help="Replan interval for --trigger periodic")
    parser.add_argument("--no-place", action="store_true",
                        help="Leave the robots where they are")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    vision = Vision()
    sender = grSimSender()
    dispatcher = RobotCommandDispatcher(sender.send_robot_command, send_hz=COMMAND_HZ)
    dispatcher.start()

    if not args.no_place:
        place_teams(sender, args.robots, args.obstacles)
        time.sleep(0.5)

    if not vision.wait_for_robots(args.robots, args.obstacles):
        print(f"Only saw {len(vision.blue)} blue and {len(vision.yellow)} yellow robots.")
        print("Is grSim running, and is this process on the same side of WSL as it?")
        return 1

    lanes = [(i - (args.robots - 1) / 2.0) * LANE_SPACING_MM for i in range(args.robots)]
    robots = [Robot(robot_id=i, target=(GOAL_X, lanes[i])) for i in range(args.robots)]

    print(f"\n{args.planner} planner, {args.robots} robots at {args.speed:.1f} m/s, "
          f"{args.obstacles} moving obstacles, {args.trigger} trigger")
    print(f"Running {args.duration:.0f} s. Watch the pitch; the numbers are below.\n")

    started = time.time()
    last_report = 0.0
    ticks = 0

    try:
        while time.time() - started < args.duration:
            tick_start = time.perf_counter()
            vision.update()
            ticks += 1
            drive_obstacles(dispatcher, vision, args.obstacles)

            for robot in robots:
                pose = vision.blue.get(robot.robot_id)
                if pose is None:
                    continue
                x, y, theta = pose
                here = (x, y)

                if math.dist(here, robot.target) <= ARRIVED_MM:
                    robot.laps += 1
                    robot.target = (START_X if robot.target[0] > 0 else GOAL_X,
                                    robot.target[1])
                    robot.path = ()

                obstacles = obstacles_for(vision, robot.robot_id)
                robot.closest_mm = min(
                    robot.closest_mm,
                    min((math.dist(here, o.pos_mm) for o in obstacles), default=float("inf")),
                )

                if args.trigger == "always":
                    due = True
                elif args.trigger == "periodic":
                    due = (ticks * CONTROL_TICK_S * 1000.0) % args.period_ms < (
                        CONTROL_TICK_S * 1000.0)
                else:
                    due = path_blocked((here, *robot.path), obstacles)

                if not robot.path or due:
                    previous = (here, *robot.path) if robot.path else ()
                    new_path, ms = plan_for(args.planner, here, robot.target,
                                            obstacles, key=("blue", robot.robot_id))
                    robot.stability.observe(previous, (here, *new_path) if new_path else ())
                    robot.path = new_path
                    robot.replans += 1
                    robot.plan_ms_total += ms

                # Drive at the next waypoint, or straight at the target when the
                # planner returned nothing to follow.
                aim = robot.path[0] if robot.path else robot.target
                if robot.path and math.dist(here, aim) < ARRIVED_MM:
                    robot.path = robot.path[1:]
                    aim = robot.path[0] if robot.path else robot.target

                ex, ey = aim[0] - x, aim[1] - y
                dist = math.hypot(ex, ey)
                if dist > 1.0:
                    scale = args.speed * min(1.0, dist / 400.0) / dist
                    vx, vy = rotate_into_robot_frame(ex * scale, ey * scale, theta)
                else:
                    vx = vy = 0.0
                dispatcher.publish(
                    RobotCommand(robot_id=robot.robot_id, vx=vx, vy=vy, w=0.0, isYellow=False)
                )

            now = time.time() - started
            if now - last_report >= 2.0:
                last_report = now
                parts = []
                for r in robots:
                    s = r.stability
                    parts.append(
                        f"#{r.robot_id} laps {r.laps} replans {r.replans:3d} "
                        f"rev {s.reversals:3d} {r.plan_ms_total / max(r.replans, 1):5.1f}ms"
                    )
                closest = min(r.closest_mm for r in robots)
                print(f"  t={now:5.1f}s  " + "  ".join(parts) +
                      f"   closest {closest:.0f}mm")

            elapsed = time.perf_counter() - tick_start
            if elapsed < CONTROL_TICK_S:
                time.sleep(CONTROL_TICK_S - elapsed)

    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        for r in robots:
            dispatcher.publish(RobotCommand(robot_id=r.robot_id, isYellow=False))
        for i in range(args.obstacles):
            dispatcher.publish(RobotCommand(robot_id=i, isYellow=True))
        time.sleep(0.3)
        dispatcher.stop()

    print(f"\n{'robot':<8}{'laps':>6}{'replans':>9}{'reversals':>11}"
          f"{'mean ms':>9}{'closest mm':>12}")
    print("-" * 55)
    for r in robots:
        print(f"#{r.robot_id:<7}{r.laps:>6}{r.replans:>9}{r.stability.reversals:>11}"
              f"{r.plan_ms_total / max(r.replans, 1):>9.2f}{r.closest_mm:>12.0f}")
    print(f"\nClosest approach below {CONTACT_MM:.0f} mm means the robots touched.")
    print("Reversals are replans that turned the robot more than 90 degrees from")
    print("the direction it was already going; see scripts/path_stability.py.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
