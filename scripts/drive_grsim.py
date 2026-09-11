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
    # Calls where the planner returned no waypoints because the goal was
    # directly visible. These build no roadmap and must not be counted as
    # replans: doing so reported ~550 replans in 30 s, one per control tick,
    # which is a count of clear sightlines rather than of planning work.
    direct: int = 0
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


class LogOpponents:
    """Yellow robots replaying a recorded Division B match, frame by frame.

    Placed rather than driven. These are a recorded obstacle field, not
    simulated agents: teleporting each frame reproduces the recorded
    trajectories exactly, where a velocity controller chasing them would track
    with its own error and stop being the thing that was recorded. It also
    sidesteps the synthetic-motion problem that defeated two earlier attempts,
    both of which produced jitter in place rather than travel.

    Frames come from the same reader the offline churn and reuse measurements
    use, so the closed-loop tier and the log tier see the same motion.
    """

    def __init__(self, log_path: str, seconds: float, skip_seconds: float,
                 count: int, team: str = "yellow") -> None:
        from measure_graph_churn import capture_from_logfile

        self.captures = capture_from_logfile(
            log_path, seconds=seconds, skip_seconds=skip_seconds
        )
        if not self.captures:
            raise SystemExit(f"no frames read from {log_path}")
        span = self.captures[-1].t_s - self.captures[0].t_s
        self.hz = len(self.captures) / span if span > 0 else 60.0

        # Choose WHICH recorded robots to replay, by identity, once. Vision
        # drops robots intermittently: in a 30 s clip the frame carries 8, 9, 10
        # or 11 obstacles and each robot is present in 80 to 100 per cent of
        # frames. An earlier version took `obstacles[:count]` and numbered them
        # by list position, so a robot vanishing for one frame shifted every
        # later slot down by one and the grSim robot in that slot jumped to a
        # different real robot. Measured on this clip: apparent slot speeds of
        # 41 and 57 m/s, against 0.96 to 3.46 m/s for the same robots tracked by
        # identity. This is the mistake `measure_graph_churn.py` documents for
        # edge labels, in a second place.
        want_yellow = team != "blue"
        seen: dict[tuple[bool, int], int] = {}
        for cap in self.captures:
            for o in cap.obstacles:
                key = (bool(o.isYellow), int(o.robot_id))
                if key[0] is want_yellow:
                    seen[key] = seen.get(key, 0) + 1
        # Most consistently tracked first: a robot present in half the frames
        # spends the other half held at a stale position.
        self.keys = sorted(seen, key=lambda k: (-seen[k], k[1]))[:count]
        self.coverage = {k: seen[k] / len(self.captures) for k in self.keys}
        if not self.keys:
            raise SystemExit(f"no {team} robots found in the clip")

        self.holds = 0          # frames where a robot was missing and was held
        self.placements = 0
        self._last: dict[tuple[bool, int], tuple[float, float]] = {}

    def frame_at(self, elapsed_s: float):
        """The recorded frame at `elapsed_s` in, looping when the clip ends.

        Resampled at the clip's MEAN rate, so this is not a timestamp-accurate
        replay: real gaps in the recording are evened out, and the clip restarts
        with a positional discontinuity once `elapsed_s` exceeds its length.
        """
        i = int(elapsed_s * self.hz) % len(self.captures)
        return self.captures[i].obstacles

    def place(self, sender: grSimSender, elapsed_s: float) -> None:
        present = {
            (bool(o.isYellow), int(o.robot_id)): o.pos_mm
            for o in self.frame_at(elapsed_s)
        }
        robots = []
        for slot, key in enumerate(self.keys):
            pos = present.get(key)
            if pos is None:
                # Hold the last known position rather than shifting slots. A
                # held obstacle is stale, not teleported, and the count is
                # reported so a run with many holds can be discarded.
                pos = self._last.get(key)
                if pos is None:
                    continue
                self.holds += 1
            else:
                self._last[key] = pos
            robots.append({"x": pos[0] / 1000.0, "y": pos[1] / 1000.0,
                           "orientation": 0.0, "robot_id": slot, "isYellow": True})
        if robots:
            self.placements += 1
            sender.send_packet(grSimPacketFactory.scenario_replacement_command(robots))


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


def run_provenance() -> dict:
    """Code revision and the grSim realism condition, recorded with the result.

    A results file that does not say which revision produced it, or whether the
    simulator was reporting exact positions, cannot be compared against another
    one later. Both are cheap to read and easy to forget.
    """
    import re
    import subprocess

    out: dict = {}
    try:
        out["git_revision"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=str(HERE.parent),
            capture_output=True, text=True, timeout=10,
        ).stdout.strip() or None
        out["git_dirty"] = bool(subprocess.run(
            ["git", "status", "--porcelain"], cwd=str(HERE.parent),
            capture_output=True, text=True, timeout=10,
        ).stdout.strip())
    except Exception:  # noqa: BLE001 - provenance must never fail a run
        out["git_revision"] = None

    if not out.get("git_revision"):
        # git did not answer, so its "clean" verdict is absence of output rather
        # than evidence. Say unknown instead of claiming a clean tree.
        out["git_dirty"] = None
        # This checkout is a git worktree whose .git file records the main
        # repository by WINDOWS path. Run from WSL, `git rev-parse` fails with
        # "not a git repository: /mnt/c/.../C:/Users/...", because git joins the
        # recorded absolute path onto the current directory. Read HEAD directly
        # instead, translating a drive letter to its /mnt mount point.
        try:
            dot_git = HERE.parent / ".git"
            head_dir = dot_git
            if dot_git.is_file():
                recorded = dot_git.read_text(encoding="utf-8").split("gitdir:", 1)[1].strip()
                if len(recorded) > 2 and recorded[1] == ":":
                    recorded = f"/mnt/{recorded[0].lower()}{recorded[2:]}"
                head_dir = Path(recorded.replace("\\", "/"))
            head = (head_dir / "HEAD").read_text(encoding="utf-8").strip()
            if head.startswith("ref:"):
                ref = head.split(None, 1)[1]
                common = head_dir.parent.parent if head_dir.name.startswith("research") else head_dir
                for candidate in (head_dir / ref, common / ref):
                    if candidate.exists():
                        head = candidate.read_text(encoding="utf-8").strip()
                        break
            out["git_revision"] = head if len(head) == 40 else None
            out["git_revision_source"] = "HEAD file, git command unavailable here"
        except Exception:  # noqa: BLE001
            out["git_revision"] = None

    cfg = Path.home() / ".grsim.xml"
    grsim: dict = {"config": str(cfg)}
    if cfg.exists():
        xml = cfg.read_text(encoding="utf-8", errors="replace")
        for key, label in (("Noise", "noise_enabled"),
                           ("Deviation for x values", "noise_x_mm"),
                           ("Deviation for y values", "noise_y_mm"),
                           ("Sending delay (milliseconds)", "sending_delay_ms")):
            m = re.search(r'<Var name="%s"[^>]*>\s*([^<\s]+)' % re.escape(key), xml)
            grsim[label] = m.group(1) if m else None
    out["grsim"] = grsim
    out["command"] = " ".join(sys.argv)
    return out


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
    parser.add_argument("--opponents", choices=("patrol", "log"), default="patrol",
                        help="patrol shuttles yellow robots up and down; log "
                             "replays recorded Division B match trajectories")
    parser.add_argument("--log", help="Match log for --opponents log")
    parser.add_argument("--log-seconds", type=float, default=60.0)
    parser.add_argument("--log-skip", type=float, default=180.0)
    parser.add_argument("--replay-team", choices=("yellow", "blue"), default="yellow",
                        help="Which recorded team to replay into grSim's yellow "
                             "slots. Selection is by robot identity, not list "
                             "position; see LogOpponents.")
    parser.add_argument("--out-json", help="Write the per-robot results here")
    parser.add_argument("--no-place", action="store_true",
                        help="Leave the robots where they are")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    opponents = None
    if args.opponents == "log":
        if not args.log:
            parser.error("--opponents log needs --log PATH")
        opponents = LogOpponents(args.log, args.log_seconds, args.log_skip,
                                 args.obstacles, team=args.replay_team)
        print(f"replaying {len(opponents.captures)} frames at "
              f"{opponents.hz:.0f} Hz mean from {Path(args.log).name}, "
              f"clip {args.log_skip:.0f}-{args.log_skip + args.log_seconds:.0f} s")
        print("  recorded robots: " + ", ".join(
            f"{'Y' if y else 'B'}{r} into slot {i} ({opponents.coverage[(y, r)] * 100:.0f}% of frames)"
            for i, (y, r) in enumerate(opponents.keys)))

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
            if opponents is None:
                drive_obstacles(dispatcher, vision, args.obstacles)
            else:
                opponents.place(sender, time.time() - started)

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
                    if new_path:
                        robot.replans += 1
                    else:
                        robot.direct += 1
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

    print(f"\n{'robot':<8}{'laps':>6}{'replans':>9}{'direct':>8}{'reversals':>11}"
          f"{'mean ms':>9}{'closest mm':>12}")
    print("-" * 63)
    for r in robots:
        print(f"#{r.robot_id:<7}{r.laps:>6}{r.replans:>9}{r.direct:>8}"
              f"{r.stability.reversals:>11}"
              f"{r.plan_ms_total / max(r.replans + r.direct, 1):>9.2f}"
              f"{r.closest_mm:>12.0f}")
    if args.out_json:
        import json
        Path(args.out_json).write_text(json.dumps({
            "provenance": run_provenance(),
            "meta": {"planner": args.planner, "robots": args.robots,
                     "obstacles": args.obstacles, "speed_mps": args.speed,
                     "trigger": args.trigger, "duration_s": args.duration,
                     "opponents": args.opponents, "log": args.log,
                     "replay_team": args.replay_team,
                     "log_skip_s": args.log_skip, "log_seconds": args.log_seconds,
                     "replay": None if opponents is None else {
                         "frames": len(opponents.captures),
                         "mean_hz": opponents.hz,
                         "robots": [
                             {"slot": i, "recorded": ("yellow" if y else "blue"),
                              "robot_id": r, "coverage": opponents.coverage[(y, r)]}
                             for i, (y, r) in enumerate(opponents.keys)],
                         "placements": opponents.placements,
                         "held_positions": opponents.holds,
                     }},
            "robots": [
                {"robot_id": r.robot_id, "laps": r.laps, "replans": r.replans, "direct": r.direct,
                 "reversals": r.stability.reversals,
                 "replans_compared": r.stability.replans_compared,
                 "mean_heading_deg": r.stability.mean_heading_deg,
                 "mean_shift_mm": r.stability.mean_shift_mm,
                 "mean_plan_ms": r.plan_ms_total / max(r.replans + r.direct, 1),
                 "closest_mm": r.closest_mm}
                for r in robots],
        }, indent=1), encoding="utf-8")
        print(f"wrote {args.out_json}")

    print(f"\nClosest approach below {CONTACT_MM:.0f} mm means the robots touched.")
    print("Reversals are replans that turned the robot more than 90 degrees from")
    print("the direction it was already going; see scripts/path_stability.py.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
