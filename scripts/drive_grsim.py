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
import hashlib
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOADED_SOURCE_HASHES = {name: hashlib.sha256((HERE/name).read_bytes()).hexdigest()
                       for name in ('drive_grsim.py','closed_loop_diagnostics.py')}
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from path_stability import Stability
from closed_loop_diagnostics import Diagnostics

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
TRACK_EVERY = 5              # sample trajectories every 5th tick, 4 Hz
CONTACT_MM = 2 * DEFAULT_ROBOT_RADIUS_MM   # centre distance at which robots touch
INVALIDATION_MM = 90.0       # the source study's replan trigger threshold

# Keep well inside the touchlines: a robot commanded past them piles into the
# wall and stops, which looks like a planner failure and is not one.
# Set by --traverse-x. The default keeps the historical 6400 mm traverse.
# Shorter can be better: at +/-2400, y=750 the corridor carries recorded traffic
# in 100 per cent of frames with 708 mm of endpoint clearance, where the full
# +/-3200 at y=1500 runs through empty space (closest approach 465 to 905 mm)
# and never fires the trigger. See scripts/choose_traverse.py.
START_X, GOAL_X = -3200.0, 3200.0
LANE_SPACING_MM = 900.0

# Yellow robots cross the middle at this speed, reversing at the edges. Moving
# obstacles are the whole point; against a static scene every planner looks the
# same and no trigger ever fires twice.
OBSTACLE_SPEED_MPS = 0.7
OBSTACLE_TURN_Y_MM = 1900.0


# Set by --force-full-build. Leaving the shortcut on is correct for production
# and wrong for comparing per-call cost against the offline tables, which
# disable it so that every reported call builds a roadmap.
FORCE_FULL_BUILD = False


def _prm(request, key):
    return prm_dijkstra.plan(request, skip_direct_path=FORCE_FULL_BUILD,
                             search_key=key)


def _visibility(request, key):
    return visibility_graph.plan(request, skip_direct_path=FORCE_FULL_BUILD,
                                 search_key=key)


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
    # Explicit direct-line returns, normalised to a goal waypoint by plan_for.
    direct: int = 0
    plan_ms_total: float = 0.0
    laps: int = 0
    closest_mm: float = float("inf")
    # Sampled, not every tick: 30 s at 20 Hz is 600 points per robot and the
    # figure needs shape, not resolution. Written to --out-json so a run can be
    # drawn afterwards without rerunning it.
    track: list = field(default_factory=list)
    failed: int = 0
    diagnostics: Diagnostics = field(default_factory=Diagnostics)


def rotate_into_robot_frame(ex: float, ey: float, theta: float) -> tuple[float, float]:
    """World-frame error to the robot's own axes.

    grSim reads veltangent and velnormal in the robot's frame, so a world-frame
    velocity sent unrotated drives every robot in a different direction
    depending on which way it happens to be facing.
    """
    c, s = math.cos(theta), math.sin(theta)
    return c * ex + s * ey, -s * ex + c * ey


def follow_waypoints(here, theta, path, speed_mps, tolerance_mm):
    """Pursue only a supplied waypoint. Empty/exhausted paths command zero."""
    if path and math.dist(here,path[0]) < tolerance_mm:
        path = path[1:]
    if not path:
        return (), 0.0, 0.0
    ex,ey = path[0][0]-here[0], path[0][1]-here[1]
    distance = math.hypot(ex,ey)
    if distance <= 1.0:
        return path, 0.0, 0.0
    scale = speed_mps * min(1.0,distance/400.0) / distance
    vx,vy = rotate_into_robot_frame(ex*scale,ey*scale,theta)
    return path, vx, vy


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

    def __init__(self, *, pixel_truth: bool = False) -> None:
        self.sock = grSimVision(threading.Event())
        self.sock.is_running.set()
        self.blue: dict[int, tuple[float, float, float]] = {}
        self.yellow: dict[int, tuple[float, float, float]] = {}
        self.pixel_truth = pixel_truth
        self.pending_frames = {}
        self.capture_s = None
        self.observed_snapshot = {}
        self.truth_snapshot = {}

    def update(self) -> int:
        self.sock.sock.setblocking(False)
        frames = 0
        try:
            while True:
                packet = self.sock.listen()
                if packet is None or not packet.HasField("detection"):
                    continue
                frames += 1
                detection = packet.detection
                frame = self.pending_frames.setdefault(detection.t_capture, dict(cameras=set(), observed={}, truth={}))
                frame['cameras'].add(detection.camera_id)
                for yellow, robots in ((False,detection.robots_blue),(True,detection.robots_yellow)):
                    for r in robots:
                        key = (yellow,r.robot_id)
                        frame['observed'][key] = (r.x,r.y)
                        if self.pixel_truth and r.HasField('pixel_x') and r.HasField('pixel_y'):
                            frame['truth'][key] = (r.pixel_x,r.pixel_y)
                for r in packet.detection.robots_blue:
                    self.blue[r.robot_id] = (r.x, r.y, r.orientation)
                for r in packet.detection.robots_yellow:
                    self.yellow[r.robot_id] = (r.x, r.y, r.orientation)
        except (BlockingIOError, OSError):
            pass
        finally:
            self.sock.sock.setblocking(True)
        # The installed grSim emits four camera packets at one sim timestamp.
        # Never combine truth from different timestamps to manufacture proximity.
        complete = [t for t,f in self.pending_frames.items() if len(f['cameras']) == 4]
        if complete:
            t = max(complete)
            self.capture_s = t
            self.observed_snapshot = self.pending_frames[t]['observed']
            self.truth_snapshot = self.pending_frames[t]['truth']
            self.pending_frames = {stamp:f for stamp,f in self.pending_frames.items() if stamp > t}
        if len(self.pending_frames) > 64:
            self.pending_frames = dict(sorted(self.pending_frames.items())[-64:])
        return frames

    def wait_for_robots(self, blue: int, yellow: int, timeout_s: float = 5.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.update()
            if len(self.blue) >= blue and len(self.yellow) >= yellow:
                return True
            time.sleep(0.05)
        return False


# Half-field plus a margin for robots legitimately over the touchline. Robots
# parked beyond this are out of play and cannot obstruct a path inside it.
IN_PLAY_X_MM = 4500.0 + 500.0
IN_PLAY_Y_MM = 3000.0 + 500.0


def in_play(pos) -> bool:
    """Is this robot inside the field, or parked out of the way?

    place_teams parks every robot id it does not use, because grSim starts with
    a full team and an unplaced robot would otherwise be counted as an obstacle.
    Parking alone is not enough: grSim keeps reporting a parked robot, so its
    position has to be filtered too. Measured when a fresh grSim silently turned
    a 6-obstacle run into an 11-obstacle one and the visibility graph's build
    time went from 4.1-4.8 ms to 10.3-13.5 ms.
    """
    return abs(pos[0]) <= IN_PLAY_X_MM and abs(pos[1]) <= IN_PLAY_Y_MM


def obstacles_for(vision: Vision, skip_blue: int) -> tuple[Obstacle, ...]:
    """Every other robot in play, both teams."""
    out = []
    for rid, (x, y, _t) in vision.blue.items():
        if not in_play((x, y)):
            continue
        if rid == skip_blue:
            continue
        out.append(Obstacle(pos_mm=(x, y), radius_mm=DEFAULT_ROBOT_RADIUS_MM,
                            robot_id=rid, isYellow=False))
    for rid, (x, y, _t) in vision.yellow.items():
        if not in_play((x, y)):
            continue
        out.append(Obstacle(pos_mm=(x, y), radius_mm=DEFAULT_ROBOT_RADIUS_MM,
                            robot_id=rid, isYellow=True))
    return tuple(out)


@dataclass(frozen=True)
class PlannerCall:
    waypoints: tuple[Point, ...]
    ms: float
    status: str


def plan_for(planner: str, start: Point, goal: Point,
             obstacles: tuple[Obstacle, ...], key) -> PlannerCall:
    """Retain failure vs direct-path semantics across all three backends."""
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
        result = VoronoiDijkstraPlanner().plan(
            scene, start, goal, search_key=key,
            skip_direct_path=FORCE_FULL_BUILD)
        waypoints = tuple(result.waypoints_mm)
        status = 'direct' if result.used_direct_path else ('path' if waypoints else 'failed')
        if result.used_direct_path:
            waypoints = (result.target_mm,)
    else:
        request = PlanRequest(start_mm=start, goal_mm=goal, obstacles=obstacles)
        result = PLANNERS[planner](request, key)
        # The visibility graph and PRM include the start as their first
        # waypoint; Voronoi does not. Drop it so every backend hands back the
        # same thing: where to go next.
        waypoints = tuple(result.waypoints_mm)[1:] if result.success else ()
        status = ('direct' if result.message.startswith('direct line of sight') else 'path') if result.success and waypoints else 'failed'
    return PlannerCall(waypoints, (time.perf_counter() - t0) * 1000.0, status)


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
                 count: int, team: str = "yellow", controlled_blue: int = 1) -> None:
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
        extent: dict[tuple[bool, int], list] = {}
        for cap in self.captures:
            for o in cap.obstacles:
                key = (bool(o.isYellow), int(o.robot_id))
                if team != "both" and key[0] is not want_yellow:
                    continue
                seen[key] = seen.get(key, 0) + 1
                extent.setdefault(key, []).append(o.pos_mm)

        # How far a robot actually got, as the span of its extreme positions.
        # NOT path length: a robot jittering on the spot accumulates metres of
        # path without leaving its own footprint, and vision noise alone adds
        # about 0.13 m/s to any speed differentiated at these intervals.
        def span_mm(key) -> float:
            pts = extent[key]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

        # A robot tracked in under half the frames is held at a stale position
        # for the rest, so it is a worse obstacle than one that simply sits.
        # Among those that clear the gate, prefer the ones that move: coverage
        # correlates with speed at about -0.95 in this log, so ordering by
        # coverage alone systematically picks the parked robots.
        usable = [k for k in seen if seen[k] >= 0.5 * len(self.captures)]
        if not usable:
            usable = list(seen)
        self.keys = sorted(usable, key=lambda k: (-span_mm(k), k[0], k[1]))[:count]
        self.span_mm = {k: span_mm(k) for k in self.keys}
        self.coverage = {k: seen[k] / len(self.captures) for k in self.keys}
        if not self.keys:
            raise SystemExit(f"no {team} robots found in the clip")

        # holds counts ROBOT placements, not frames: one frame missing two
        # robots adds two. The denominator is robot_placements, not placements,
        # and dividing by the wrong one turned 5.1% into 30.9% once already.
        self.holds = 0
        self.placements = 0
        self.robot_placements = 0
        self._last: dict[tuple[bool, int], tuple[float, float]] = {}
        self.controlled_blue = controlled_blue

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
            rid, is_yellow = obstacle_slot(slot, self.controlled_blue)
            robots.append({"x": pos[0] / 1000.0, "y": pos[1] / 1000.0,
                           "orientation": 0.0, "robot_id": rid,
                           "isYellow": is_yellow})
        if robots:
            self.placements += 1
            self.robot_placements += len(robots)
            sender.send_packet(grSimPacketFactory.scenario_replacement_command(robots))


def check_endpoints(args, opponents):
    """Endpoints a replayed robot sits on, as (point, blocked_fraction, min_mm).

    Silent failure this prevents, measured on the blue replay of
    2024-07-19_TurtleRabbit-vs-NAMeC at t=180 s: the keeper occupies (-3200, 0)
    in 73.3 per cent of frames, no path to the goal exists, and 247 of 253
    planning calls returned failure while the robot stood still for 12 s of 30.
    Nothing in the output said so; the run looked like a fast planner.
    """
    from check_endpoint_clearance import frames_from_opponents, score

    frames = frames_from_opponents(opponents)
    lanes = [args.lane_centre_mm + (i - (args.robots - 1) / 2.0) * LANE_SPACING_MM
             for i in range(args.robots)]
    out = []
    for y in lanes:
        for x in (START_X, GOAL_X):
            s = score(frames, (x, y))
            if s["blocked_frac"] > 0:
                out.append(((x, y), s["blocked_frac"], s["min_mm"]))
    return out


# Far outside the 9000 x 6000 field, so a parked robot cannot be mistaken for
# an obstacle even if grSim keeps reporting it.
PARK_X_MM, PARK_Y_MM = 0.0, -7000.0
PARK_UP_TO = 11  # grSim allows up to 11 a side; park every id we do not use
TEAM_SIZE = 6    # Division B, matching "Robots Count" in ~/.grsim.xml


def obstacle_slot(slot: int, controlled_blue: int) -> tuple[int, bool]:
    """Which grSim robot carries obstacle `slot`, as (robot_id, is_yellow).

    Yellow fills first, then the blue robots we are not driving. Division B is
    6 a side, so one controlled robot leaves room for 6 + 5 = 11 obstacles,
    which is the full scene a real match presents.
    """
    if slot < TEAM_SIZE:
        return slot, True
    return controlled_blue + (slot - TEAM_SIZE), False


def obstacle_capacity(controlled_blue: int) -> int:
    return TEAM_SIZE + max(0, TEAM_SIZE - controlled_blue)


def place_teams(sender: grSimSender, blue: int, yellow: int,
                obstacles: int = 0, lane_centre_mm: float = 0.0) -> None:
    """Line the blue team up on one touchline, the yellow team across the middle.

    Also parks every robot id we are NOT using. grSim starts with a full team,
    and `obstacles_for` counts every robot it can see, so an unplaced robot
    becomes an uncounted obstacle: a fresh simulator turned a 6-obstacle run
    into an 11-obstacle run without changing a single command line.
    """
    robots = []
    for i in range(blue):
        robots.append({
            "x": START_X / 1000.0,
            "y": (lane_centre_mm
                  + (i - (blue - 1) / 2.0) * LANE_SPACING_MM) / 1000.0,
            "orientation": 0.0, "robot_id": i, "isYellow": False,
        })
    for i in range(yellow):
        robots.append({
            "x": (-1200.0 + i * 1200.0) / 1000.0,
            "y": (OBSTACLE_TURN_Y_MM if i % 2 else -OBSTACLE_TURN_Y_MM) / 1000.0,
            "orientation": 0.0, "robot_id": i, "isYellow": True,
        })
    # Robots an obstacle slot will drive must NOT be parked, or they sit off
    # the field until their first placement and are missing from the scene.
    used = {obstacle_slot(s, blue) for s in range(obstacles)}
    for i in range(blue, PARK_UP_TO):
        if (i, False) in used:
            continue
        robots.append({"x": PARK_X_MM / 1000.0, "y": PARK_Y_MM / 1000.0,
                       "orientation": 0.0, "robot_id": i, "isYellow": False})
    for i in range(PARK_UP_TO):
        if (i, True) in used:
            continue
        robots.append({"x": PARK_X_MM / 1000.0, "y": (PARK_Y_MM - 500.0) / 1000.0,
                       "orientation": 0.0, "robot_id": i, "isYellow": True})
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
    out['source_sha256_at_import'] = LOADED_SOURCE_HASHES
    # Which grSim process this run talked to. Timings from different simulator
    # sessions are not comparable: across one restart, with the same command
    # line and the same 6 in-play obstacles, every planner got 1.5 to 1.6 times
    # faster and the per-planner ranges did not overlap. Pool by this value.
    import platform

    on_windows = platform.system() == "Windows"

    def where_grsim_lives(cmd: str) -> str:
        """Run a shell command on the machine grSim runs on.

        There is no Windows build of grSim in this project, so a driver running
        on Windows is talking to grSim inside WSL over unicast (--grsim-host).
        The pid, start time and config it needs for provenance live there, not
        here; without this hop a Windows-driven run recorded None for the noise
        and delay settings that decide whether the result is comparable.
        """
        argv = (["wsl.exe", "-e", "bash", "-lc", cmd] if on_windows
                else ["bash", "-lc", cmd])
        return subprocess.run(argv, capture_output=True, text=True,
                              timeout=15).stdout.strip()

    out["grsim_location"] = "wsl" if on_windows else "local"
    try:
        pid = where_grsim_lives("pgrep -o grSim")
        out["grsim_pid"] = int(pid) if pid else None
        # Process start time, so a reused pid after a restart is still distinct.
        out["grsim_started"] = (
            where_grsim_lives(f"ps -o lstart= -p {pid}") or None) if pid else None
    except Exception:  # noqa: BLE001 - provenance must never fail a run
        out["grsim_pid"] = None
        out["grsim_started"] = None
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
    xml = ""
    if cfg.exists():
        xml = cfg.read_text(encoding="utf-8", errors="replace")
    elif on_windows:
        try:
            xml = where_grsim_lives("cat ~/.grsim.xml")
            grsim["config"] = "~/.grsim.xml (WSL)"
        except Exception:  # noqa: BLE001
            xml = ""
    if xml:
        # vision_address says whether vision went by multicast (224.5.23.2,
        # drivers inside WSL) or unicast to the Windows side of the WSL adapter
        # (drivers on Windows). The two are not both possible at once.
        for key, label in (("Noise", "noise_enabled"),
                           ("Deviation for x values", "noise_x_mm"),
                           ("Deviation for y values", "noise_y_mm"),
                           ("Sending delay (milliseconds)", "sending_delay_ms"),
                           ("Vision multicast address", "vision_address")):
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
        call = plan_for(name, (-2000.0, 0.0), (2000.0, 0.0), obstacles, key=("test", 0))
        waypoints, ms = call.waypoints, call.ms
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
    parser.add_argument("--opponents", choices=("patrol", "log", "static"), default="patrol",
                        help="patrol shuttles yellow robots up and down; log "
                             "replays recorded Division B match trajectories")
    parser.add_argument("--log", help="Match log for --opponents log")
    parser.add_argument("--log-seconds", type=float, default=60.0)
    parser.add_argument("--log-skip", type=float, default=180.0)
    parser.add_argument("--replay-team", choices=("yellow", "blue", "both"),
                        default="yellow",
                        help="Which recorded team to replay into grSim's yellow "
                             "slots. Selection is by robot identity, not list "
                             "position; see LogOpponents.")
    parser.add_argument("--force-full-build", action="store_true",
                        help="Disable the direct line-of-sight shortcut so every "
                             "planner call builds a roadmap, matching the offline "
                             "tables. Without it the planners take the shortcut at "
                             "different rates and per-call times compare unlike work.")
    parser.add_argument("--grsim-host", default=None,
                        help="IP that receives robot commands (default: grsim_command_ip "
                             "in network_input.yaml). Set to the WSL address when grSim runs "
                             "in WSL and this driver runs on Windows.")
    parser.add_argument("--allow-extra-robots", action="store_true",
                        help="Run even when grSim shows more robots than were asked "
                             "for. Off by default: every robot on the pitch counts as "
                             "an obstacle, so a stray silently changes the scenario.")
    parser.add_argument("--traverse-x", type=float, default=3200.0,
                        help="Half-length of the traverse in mm. The robot shuttles "
                             "between -X and +X at the lane centre. Use with "
                             "scripts/choose_traverse.py, which scores endpoints for "
                             "clearance and corridor traffic together.")
    parser.add_argument("--lane-centre-mm", type=float, default=0.0,
                        help="Shift every lane off the centre line. The default 0 puts "
                             "the traverse through both goal mouths, where a recorded "
                             "match parks its keepers; 1500 clears them on this clip.")
    parser.add_argument("--allow-blocked-endpoints", action="store_true",
                        help="Run even when a replayed robot sits on an endpoint. Off by "
                             "default: an endpoint inside an inflated obstacle has no "
                             "path, so the run measures how fast each planner reports "
                             "failure rather than how fast it plans.")
    parser.add_argument("--out-json", help="Write the per-robot results here")
    parser.add_argument('--grsim-pixel-truth', action='store_true',
                        help='Interpret pixel_x/y as unnoised world mm ONLY for the inspected grSim build; not SSL-Vision.')
    parser.add_argument('--waypoint-tolerance-mm', type=float, default=ARRIVED_MM,
                        help='Intermediate waypoint acceptance radius; independent of goal arrival radius.')
    parser.add_argument('--trigger-centre-threshold-mm', type=float, default=CONTACT_MM,
                        help='Geometric trigger centre-distance threshold: historical grSim default 180; offline experiment uses 90.')
    parser.add_argument("--no-place", action="store_true",
                        help="Leave the robots where they are")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.waypoint_tolerance_mm <= 0:
        parser.error('--waypoint-tolerance-mm must be positive')
    if args.trigger_centre_threshold_mm <= 0:
        parser.error('--trigger-centre-threshold-mm must be positive')
    global FORCE_FULL_BUILD, START_X, GOAL_X
    FORCE_FULL_BUILD = args.force_full_build
    START_X, GOAL_X = -abs(args.traverse_x), abs(args.traverse_x)
    if args.out_json and Path(args.out_json).exists():
        parser.error('--out-json already exists; choose a new result file')

    if args.selftest:
        return selftest()

    opponents = None
    if args.opponents == "log":
        if not args.log:
            parser.error("--opponents log needs --log PATH")
        if args.obstacles > obstacle_capacity(args.robots):
            parser.error(
                f"--obstacles {args.obstacles} exceeds the "
                f"{obstacle_capacity(args.robots)} grSim robots available with "
                f"{args.robots} controlled: Division B is {TEAM_SIZE} a side.")
        opponents = LogOpponents(args.log, args.log_seconds, args.log_skip,
                                 args.obstacles, team=args.replay_team,
                                 controlled_blue=args.robots)
        print(f"replaying {len(opponents.captures)} frames at "
              f"{opponents.hz:.0f} Hz mean from {Path(args.log).name}, "
              f"clip {args.log_skip:.0f}-{args.log_skip + args.log_seconds:.0f} s")
        if len(opponents.keys) < args.obstacles:
            print(f"\nRefusing to run: --obstacles {args.obstacles} but this clip "
                  f"tracks only {len(opponents.keys)} {args.replay_team} robots.")
            print("The robots that are never placed keep their startup position for the")
            print("whole run and are still counted as obstacles, so the scene would be")
            print(f"{args.obstacles - len(opponents.keys)} parked robot(s) plus "
                  f"{len(opponents.keys)} replayed ones.")
            print(f"Pass --obstacles {len(opponents.keys)}, or choose a clip with more "
                  f"tracked robots.")
            return 4

        blocked = check_endpoints(args, opponents)
        if blocked and not args.allow_blocked_endpoints:
            print("\nRefusing to run: a replayed robot occupies an endpoint.")
            for (x, y), frac, mm in blocked:
                print(f"  ({x:.0f}, {y:.0f}) blocked in {frac * 100:.1f}% of frames, "
                      f"nearest obstacle centre {mm:.0f} mm")
            print("The planners inflate obstacles by 210 mm, so such a goal has no path\n"
                  "and the run measures failure latency, not planning time.\n"
                  "Use --lane-centre-mm to move the traverse, or "
                  "--allow-blocked-endpoints to override.")
            return 2
        print("  recorded robots: " + ", ".join(
            f"{'Y' if y else 'B'}{r} into slot {i} "
            f"({opponents.coverage[(y, r)] * 100:.0f}% of frames, "
            f"spans {opponents.span_mm[(y, r)]:.0f} mm)"
            for i, (y, r) in enumerate(opponents.keys)))

    vision = Vision(pixel_truth=args.grsim_pixel_truth)
    # --grsim-host lets the driver run on Windows while grSim runs in WSL: commands
    # go by unicast to the WSL address and grSim is configured to send vision by
    # unicast back to the Windows address. WSL2 NAT blocks the multicast vision
    # stream, which is why everything grSim-related used to run inside WSL.
    sender = grSimSender(ip=args.grsim_host) if args.grsim_host else grSimSender()
    dispatcher = RobotCommandDispatcher(sender.send_robot_command, send_hz=COMMAND_HZ)
    dispatcher.start()

    if not args.no_place:
        place_teams(sender, args.robots, min(args.obstacles, TEAM_SIZE),
                    obstacles=args.obstacles,
                    lane_centre_mm=args.lane_centre_mm)
        time.sleep(0.5)
        if opponents is not None:
            # Put the replayed robots on the pitch BEFORE counting them. The
            # count check runs ahead of the control loop's first placement, so
            # a robot this run will drive but that a previous run left parked
            # off the field would otherwise read as absent and refuse the run.
            opponents.place(sender, 0.0)
            time.sleep(0.3)

    want_yellow = sum(1 for s in range(args.obstacles)
                      if obstacle_slot(s, args.robots)[1])
    want_blue = args.robots + (args.obstacles - want_yellow)
    if not vision.wait_for_robots(want_blue, want_yellow):
        print(f"Only saw {len(vision.blue)} blue and {len(vision.yellow)} yellow robots.")
        print("Is grSim running, and is this process on the same side of WSL as it?")
        dispatcher.stop()
        return 1

    # wait_for_robots accepts "at least", which is right for waiting and wrong
    # for measuring: obstacles_for counts every robot on the pitch, so a stray
    # is an obstacle nobody asked for and nothing in the output says so.
    seen_blue = sum(1 for p in vision.blue.values() if in_play(p[:2]))
    seen_yellow = sum(1 for p in vision.yellow.values() if in_play(p[:2]))
    if (seen_blue, seen_yellow) != (want_blue, want_yellow) \
            and not args.allow_extra_robots:
        print(f"\nRefusing to run: expected {want_blue} blue and "
              f"{want_yellow} yellow, but grSim is showing "
              f"{seen_blue} blue and {seen_yellow} yellow.")
        print(f"Every robot on the pitch counts as an obstacle, so this would be "
              f"a {seen_blue - args.robots + seen_yellow}-obstacle run labelled as "
              f"{args.obstacles}.")
        print("Restart grSim, or pass --allow-extra-robots to accept it.")
        dispatcher.stop()
        return 3

    lanes = [args.lane_centre_mm + (i - (args.robots - 1) / 2.0) * LANE_SPACING_MM
             for i in range(args.robots)]
    robots = [Robot(robot_id=i, target=(GOAL_X, lanes[i])) for i in range(args.robots)]

    print(f"\n{args.planner} planner, {args.robots} robots at {args.speed:.1f} m/s, "
          f"{args.obstacles} moving obstacles, {args.trigger} trigger")
    print(f"Running {args.duration:.0f} s. Watch the pitch; the numbers are below.\n")

    started = time.time()
    last_report = 0.0
    ticks = 0
    obstacle_track: list = []

    try:
        while time.time() - started < args.duration:
            tick_start = time.perf_counter()
            vision.update()
            ticks += 1
            if args.opponents == 'patrol':
                drive_obstacles(dispatcher, vision, args.obstacles)
            elif opponents is not None:
                opponents.place(sender, time.time() - started)
                if ticks % TRACK_EVERY == 0:
                    obstacle_track.append([
                        [round(p[0], 1), round(p[1], 1)]
                        for p in (vision.yellow.get(i, (None, None, 0))[:2]
                                  for i in range(args.obstacles))
                        if p[0] is not None])

            for robot in robots:
                expected_keys = {(False,i) for i in range(args.robots)} | {(True,i) for i in range(args.obstacles)}
                robot.diagnostics.sample(t=vision.capture_s, observed=vision.observed_snapshot,
                    truth=vision.truth_snapshot, robot_key=(False,robot.robot_id), expected_keys=expected_keys)
                pose = vision.blue.get(robot.robot_id)
                if pose is None:
                    continue
                x, y, theta = pose
                here = (x, y)

                if ticks % TRACK_EVERY == 0:
                    robot.track.append([round(x, 1), round(y, 1)])

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
                    due = path_blocked((here, *robot.path), obstacles,
                        threshold_mm=args.trigger_centre_threshold_mm - DEFAULT_ROBOT_RADIUS_MM)

                if not robot.path or due:
                    previous = (here, *robot.path) if robot.path else ()
                    call = plan_for(args.planner, here, robot.target,
                                    obstacles, key=("blue", robot.robot_id))
                    new_path, ms = call.waypoints, call.ms
                    robot.stability.observe(previous, (here, *new_path) if new_path else ())
                    robot.path = new_path
                    if call.status == 'path':
                        robot.replans += 1
                    elif call.status == 'direct':
                        robot.direct += 1
                    else:
                        robot.failed += 1
                    robot.plan_ms_total += ms
                    truth_obstacles = [p for k,p in vision.truth_snapshot.items() if k != (False,robot.robot_id)] if args.grsim_pixel_truth else None
                    robot.diagnostics.planned(t=vision.capture_s, start=here, path=new_path,
                        observed_obstacles=[o.pos_mm for o in obstacles], truth_obstacles=truth_obstacles,
                        status=call.status, ms=ms)

                robot.path, vx, vy = follow_waypoints(here,theta,robot.path,args.speed,args.waypoint_tolerance_mm)
                robot.diagnostics.commanded(t=vision.capture_s,here=here,path=robot.path,velocity=(vx,vy),
                    truth_here=vision.truth_snapshot.get((False,robot.robot_id)),
                    truth_obstacles=[p for k,p in vision.truth_snapshot.items() if k != (False,robot.robot_id)])
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
                        f"rev {s.reversals:3d} fail {r.failed:3d} {r.plan_ms_total / max(r.replans + r.direct + r.failed, 1):5.1f}ms"
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
              f"{r.plan_ms_total / max(r.replans + r.direct + r.failed, 1):>9.2f}"
              f"{r.closest_mm:>12.0f}")
    if args.out_json:
        import json
        Path(args.out_json).write_text(json.dumps({
            "provenance": run_provenance(),
            "meta": {"planner": args.planner, "robots": args.robots,
                     "diagnostics_schema": 1,
                     "control_tick_s": CONTROL_TICK_S,
                     "waypoint_tolerance_mm": args.waypoint_tolerance_mm,
                     "trigger_centre_threshold_mm": args.trigger_centre_threshold_mm,
                     "failed_plan_policy": "stop and retry, never drive to goal",
                     "truth_source": "grSim pixel_x/y unnoised world mm; source fe2bd2915a46f9f11ea6cb48dc426b8047952073" if args.grsim_pixel_truth else None,
                     "obstacles": args.obstacles, "speed_mps": args.speed,
                     "trigger": args.trigger, "duration_s": args.duration,
                     "opponents": args.opponents, "log": args.log,
                     "replay_team": args.replay_team,
                     "force_full_build": args.force_full_build,
                     "grsim_host": args.grsim_host,
                     "lane_centre_mm": args.lane_centre_mm,
                     "traverse_x_mm": args.traverse_x,
                     "observed_blue": seen_blue,
                     "observed_yellow": seen_yellow,
                     "log_skip_s": args.log_skip, "log_seconds": args.log_seconds,
                     "replay": None if opponents is None else {
                         "frames": len(opponents.captures),
                         "mean_hz": opponents.hz,
                         "robots": [
                             {"slot": i, "recorded": ("yellow" if y else "blue"),
                              "robot_id": r, "coverage": opponents.coverage[(y, r)]}
                             for i, (y, r) in enumerate(opponents.keys)],
                         "placements": opponents.placements,
                         "robot_placements": opponents.robot_placements,
                         "held_positions": opponents.holds,
                         "held_fraction_of_robot_placements":
                             opponents.holds / max(opponents.robot_placements, 1),
                         "obstacle_track": obstacle_track,
                     }},
            "robots": [
                {"robot_id": r.robot_id, "laps": r.laps, "replans": r.replans, "direct": r.direct,
                 "reversals": r.stability.reversals,
                 "replans_compared": r.stability.replans_compared,
                 "mean_heading_deg": r.stability.mean_heading_deg,
                 "mean_shift_mm": r.stability.mean_shift_mm,
                 "mean_plan_ms": r.plan_ms_total / max(r.replans + r.direct + r.failed, 1),
                 "failed_plans": r.failed,
                 "closest_mm": r.closest_mm, "closest_source": "vision, may include stale poses",
                 "diagnostics": r.diagnostics.report(), "track": r.track}
                for r in robots],
        }, indent=1), encoding="utf-8")
        print(f"wrote {args.out_json}")

    print(f"\nObserved separation below {CONTACT_MM:.0f} mm is not a physics contact count.")
    print("Reversals are replans that turned the robot more than 90 degrees from")
    print("the direction it was already going; see scripts/path_stability.py.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
