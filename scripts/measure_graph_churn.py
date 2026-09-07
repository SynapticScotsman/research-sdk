#!/usr/bin/env python3
"""Phase 0: how much of the visibility graph actually changes between frames?

Lim et al. (arXiv:2210.12851) change "a randomly chosen 10% of the edges"
between consecutive searches, and their whole case for incremental replanning
rests on the change being small enough that reusing the previous search pays.
Nobody has measured the SSL equivalent. This does.

The answer decides whether incremental replanning is worth implementing at all:

  small churn per frame  -> repair beats replanning from scratch, the paper has
                            its motivation
  most of the graph churns -> the premise fails, incremental search buys
                            nothing here, and we should say so

Both outcomes are results. Not knowing is not.

## Why capture-then-analyse

A visibility-graph build costs 10-40 ms, so planning live at 60 Hz would drop
most frames and the "per frame" denominator would be a lie. Instead we capture
obstacle positions at full vision rate (cheap), then rebuild graphs offline over
consecutive pairs.

That also buys the actual deliverable: churn as a function of *how long you
waited*. Sweeping the frame stride gives fraction-of-graph-changed versus
replan interval in ms, which is the curve "how often must a robot replan?"
is asking for.

## Edge identity

Graphs are rebuilt from scratch each frame with moved vertices, so edges cannot
be matched on coordinates. `visibility_graph.plan(record=...)` logs an
`obstacles` step carrying the polygon list and one `edge_test` step per pair
with endpoint coordinates, so coordinates map back to `p{poly}v{vert}` labels.
Obstacles are sorted by (is_yellow, robot_id) before planning so that `poly`
indices mean the same robot in both frames.

Usage:
    python scripts/measure_graph_churn.py --seconds 20
    python scripts/measure_graph_churn.py --selftest
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))

from research_sdk.planners.common import (
    Obstacle,
    PlanRequest,
    StepRecorder,
)
from research_sdk.planners.VisibilityGraph import visibility_graph

# da Silva Costa & Tonidandel's replan trigger: a path is invalid whenever an
# obstacle crosses it, threshold 90 mm, the robot radius. Reproduced exactly so
# our replan counts are comparable to their Table 9.
INVALIDATION_THRESHOLD_MM = 90.0


@dataclass(frozen=True, slots=True)
class Capture:
    """One vision frame's obstacle set, as positions only."""

    t_s: float
    obstacles: tuple[Obstacle, ...]


def _sorted_obstacles(obstacles) -> tuple[Obstacle, ...]:
    """Deterministic order, so polygon index means the same robot every frame."""
    return tuple(
        sorted(obstacles, key=lambda o: (bool(o.isYellow), int(o.robot_id)))
    )


class Stirrer:
    """Drive robots between random waypoints so the scene is genuinely dynamic.

    An idle grSim is perfectly static, and a static scene has exactly zero graph
    churn -- true, and useless as a measurement of SSL. Rather than wait for a
    behaviour stack, drive the robots ourselves so that *speed is a swept
    variable*, which answers a sharper question than a match replay would: how
    fast must robots move before the graph churns enough to matter.

    Waypoint steering, not open-loop headings. Two open-loop designs were tried
    and both measured near-zero net displacement while accumulating path length:

    - fast sinusoid (0.25-0.75 Hz): the commanded direction swept a full circle
      every 1.3-4 s and, with grSim's acceleration limits and
      veltangent/velnormal being in the robot's own frame, motion cancelled
      before it built -- 18 mm of travel in 3 s while commanding 1.5 m/s;
    - slow sinusoid (0.03-0.13 Hz): a near-constant heading at 0.4 m/s covers
      10 m in 25 s, so robots drove into the boundary within seconds and then
      vibrated against it.

    Steering to waypoints inside the field fixes both: robots keep moving, stay
    in bounds, and the courses cross.

    Reference speed: da Silva Costa & Tonidandel report SSL robots exceeding
    2.5 m/s.
    """

    # Kept inside the touchlines so a robot decelerating into its waypoint does
    # not end up pinned against the boundary.
    MARGIN_MM = 700.0
    ARRIVED_MM = 300.0

    def __init__(self, speed_mps: float, robot_ids=range(6), seed: int = 0) -> None:
        import random

        from research_sdk.network.robot_command import RobotCommand
        from research_sdk.network.ssl_sockets import grSimSender
        from research_sdk.planners.common import FIELD_LENGTH_MM, FIELD_WIDTH_MM

        self._RobotCommand = RobotCommand
        self.sender = grSimSender()
        self.speed = speed_mps
        self.robot_ids = list(robot_ids)
        self.rng = random.Random(seed)
        self.half_x = FIELD_LENGTH_MM / 2 - self.MARGIN_MM
        self.half_y = FIELD_WIDTH_MM / 2 - self.MARGIN_MM
        self.targets: dict[tuple[bool, int], tuple[float, float]] = {}

    def _target_for(self, key) -> tuple[float, float]:
        if key not in self.targets:
            self.targets[key] = self._new_target()
        return self.targets[key]

    def _new_target(self) -> tuple[float, float]:
        return (
            self.rng.uniform(-self.half_x, self.half_x),
            self.rng.uniform(-self.half_y, self.half_y),
        )

    def tick(self, obstacles, orientations: dict) -> None:
        """Steer each robot toward its waypoint, using the latest observed pose.

        `veltangent`/`velnormal` are in the ROBOT's frame, not the field's, so
        the world-frame steering vector is rotated by the robot's heading. Skip
        this and a robot drives off at its heading angle instead of toward the
        waypoint -- which is how the first version ended up pinned to a wall.
        """
        for o in obstacles:
            key = (bool(o.isYellow), int(o.robot_id))
            if int(o.robot_id) not in self.robot_ids:
                continue
            tx, ty = self._target_for(key)
            dx, dy = tx - o.pos_mm[0], ty - o.pos_mm[1]
            dist = math.hypot(dx, dy)
            if dist < self.ARRIVED_MM:
                self.targets[key] = self._new_target()
                tx, ty = self.targets[key]
                dx, dy = tx - o.pos_mm[0], ty - o.pos_mm[1]
                dist = math.hypot(dx, dy) or 1.0
            ux, uy = dx / dist, dy / dist
            theta = orientations.get(key, 0.0)
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            self.sender.send_robot_command(
                self._RobotCommand(
                    robot_id=key[1],
                    vx=self.speed * (ux * cos_t + uy * sin_t),
                    vy=self.speed * (-ux * sin_t + uy * cos_t),
                    w=0.0,
                    isYellow=key[0],
                )
            )

    def stop(self) -> None:
        for team_yellow in (True, False):
            for rid in self.robot_ids:
                self.sender.send_robot_command(
                    self._RobotCommand(robot_id=rid, vx=0.0, vy=0.0, w=0.0, isYellow=team_yellow)
                )


def capture_from_grsim(
    seconds: float, vision_port: int | None = None, stir_mps: float = 0.0
) -> list[Capture]:
    """Record obstacle positions at full vision rate. No planning in this loop."""
    from multiprocessing import Event
    from time import perf_counter

    from research_sdk.network.ssl_sockets import grSimVision
    from research_sdk.world.pipeline import VisionWorldPipeline

    is_running = Event()
    is_running.set()
    sock = grSimVision(is_running) if vision_port is None else grSimVision(is_running, port=vision_port)
    pipeline = VisionWorldPipeline(cameras=4)
    stirrer = Stirrer(stir_mps) if stir_mps > 0 else None

    out: list[Capture] = []
    orientations: dict[tuple[bool, int], float] = {}
    t0 = perf_counter()
    last_cmd = 0.0
    while perf_counter() - t0 < seconds:
        now = perf_counter() - t0
        packet = sock.listen()
        if packet is None:
            continue
        # Headings come straight off the detection frame; the planning scene
        # carries position only, and the stirrer needs orientation to steer.
        if packet.HasField("detection"):
            for r in packet.detection.robots_yellow:
                orientations[(True, int(r.robot_id))] = float(r.orientation)
            for r in packet.detection.robots_blue:
                orientations[(False, int(r.robot_id))] = float(r.orientation)
        update = pipeline.ingest(packet)
        if update is None:
            continue
        scene = pipeline.latest_scene
        if scene is None:
            continue
        obstacles = _sorted_obstacles(
            Obstacle(
                pos_mm=(float(o.pos_mm[0]), float(o.pos_mm[1])),
                radius_mm=float(o.radius_mm),
                robot_id=int(o.robot_id),
                isYellow=bool(o.isYellow),
            )
            for o in scene.obstacles
        )
        out.append(Capture(t_s=perf_counter() - t0, obstacles=obstacles))

        # 50 ms, matching the control tick the UI actually uses
        # (ui/execution/page.py:380), so robots are driven at the rate the real
        # system drives them. Sent after the capture so it always steers on the
        # freshest pose.
        if stirrer is not None and now - last_cmd >= 0.05:
            stirrer.tick(obstacles, orientations)
            last_cmd = now

    if stirrer is not None:
        stirrer.stop()
    return out


# SSL log format (ssl.robocup.org/game-logs): 12-byte "SSL_LOG_FILE" magic then
# an int32 version, followed by messages of
#   int64 receiver timestamp (ns) | int32 message type | int32 length | payload
# all big-endian. Type 2 and 4 are both SSL_WrapperPacket (the 2010 and 2014
# vision formats); everything else here is referee, tracker or index data.
LOG_MAGIC = b"SSL_LOG_FILE"
LOG_VISION_TYPES = (2, 4)


def _probe_log(path: str, skip_seconds: float, window_s: float = 10.0) -> tuple[int, int]:
    """(vision message type to use, number of distinct cameras).

    Both are properties of the recording, not constants. The 2024 Division B
    logs carry every vision frame TWICE, once as type 2 (the 2010 format) and
    once as type 4 (2014), so ingesting both doubles the apparent frame rate.
    They also turned out to be single-camera, while the live grSim path is
    four -- and the frame assembler only closes a cycle once it has seen every
    camera, so a wrong count makes the frame timing irregular.
    """
    import gzip
    import struct
    from collections import Counter

    from google.protobuf.message import DecodeError

    from research_sdk.network.proto2 import ssl_vision_wrapper_pb2

    types: Counter = Counter()
    cams: set[int] = set()
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rb") as fh:
        fh.read(16)  # magic + version, already validated by the caller
        t0 = None
        while True:
            head = fh.read(16)
            if len(head) < 16:
                break
            stamp_ns, msg_type, length = struct.unpack(">qii", head)
            payload = fh.read(length)
            if len(payload) < length:
                break
            if t0 is None:
                t0 = stamp_ns
            elapsed = (stamp_ns - t0) / 1e9
            if elapsed < skip_seconds:
                continue
            if elapsed - skip_seconds > window_s:
                break
            if msg_type not in LOG_VISION_TYPES:
                continue
            types[msg_type] += 1
            packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
            try:
                packet.ParseFromString(payload)
            except DecodeError:
                continue
            if packet.HasField("detection"):
                cams.add(int(packet.detection.camera_id))

    # Prefer the 2014 format when the log carries both.
    chosen = 4 if types.get(4) else (2 if types.get(2) else 4)
    return chosen, max(len(cams), 1)


def capture_from_logfile(
    path: str, seconds: float, cameras: int | None = None, skip_seconds: float = 0.0
) -> list[Capture]:
    """Replay a recorded SSL match through the same pipeline the live path uses.

    Real match motion, rather than a synthetic stirring function. Two attempts
    at driving grSim's robots produced jitter in place rather than travel, and
    even a working stirrer would only measure the stirrer: graph churn is a
    property of how robots actually move, so it has to come from real play.

    Emits the same `Capture` objects as `capture_from_grsim`, so `analyse()`
    does not care which source it got.
    """
    import gzip
    import struct

    from google.protobuf.message import DecodeError

    from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
    from research_sdk.world.pipeline import VisionWorldPipeline

    vision_type, detected_cameras = _probe_log(path, skip_seconds)
    if cameras is None:
        cameras = detected_cameras
    print(f"  log carries vision type {vision_type}, {detected_cameras} camera(s); "
          f"assembling with cameras={cameras}")

    pipeline = VisionWorldPipeline(cameras=cameras)
    out: list[Capture] = []
    malformed = 0

    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rb") as fh:
        magic = fh.read(12)
        if magic != LOG_MAGIC:
            raise ValueError(f"not an SSL log: magic was {magic!r}")
        (version,) = struct.unpack(">i", fh.read(4))
        if version != 1:
            raise ValueError(f"unsupported SSL log version {version}")

        t0_ns: int | None = None
        while True:
            head = fh.read(16)
            if len(head) < 16:
                break
            stamp_ns, msg_type, length = struct.unpack(">qii", head)
            payload = fh.read(length)
            if len(payload) < length:
                break
            # One type only: the log stores each frame under BOTH 2 and 4, and
            # ingesting both doubles the apparent frame rate.
            if msg_type != vision_type:
                continue

            if t0_ns is None:
                t0_ns = stamp_ns
            elapsed = (stamp_ns - t0_ns) / 1e9
            if elapsed < skip_seconds:
                continue
            if elapsed - skip_seconds > seconds:
                break

            packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
            try:
                packet.ParseFromString(payload)
            except DecodeError:
                # A real recording carries the odd truncated or mislabelled
                # message; skipping one frame is right, aborting the run is not.
                malformed += 1
                continue
            update = pipeline.ingest(packet)
            if update is None:
                continue
            scene = pipeline.latest_scene
            if scene is None:
                continue
            obstacles = _sorted_obstacles(
                Obstacle(
                    pos_mm=(float(o.pos_mm[0]), float(o.pos_mm[1])),
                    radius_mm=float(o.radius_mm),
                    robot_id=int(o.robot_id),
                    isYellow=bool(o.isYellow),
                )
                for o in scene.obstacles
            )
            # Log timestamps, not wall clock: gaps in the recording are real and
            # the stride-to-milliseconds mapping has to reflect them.
            out.append(Capture(t_s=elapsed - skip_seconds, obstacles=obstacles))

    if malformed:
        print(f"  note: skipped {malformed} undecodable vision messages")
    return out


def edge_states(request: PlanRequest) -> dict[frozenset[str], bool] | None:
    """{frozenset of two vertex labels: was that pair mutually visible}.

    Built from the planner's own StepRecorder rather than a reimplementation of
    the visibility test, so this measures the graph the planner really builds.
    """
    recorder = StepRecorder()
    # skip_direct_path=True forces the full graph build even when start sees
    # goal; otherwise a clear frame logs no edge tests at all.
    result = visibility_graph.plan(request, record=recorder, skip_direct_path=True)

    # The planner bails before testing any edge when an endpoint sits inside an
    # inflated obstacle (visibility_graph.py:304). On a real match log with the
    # endpoints at the goal mouths that is the GOALKEEPER, every frame: it
    # silently emptied 192 of 200 frame pairs before this was surfaced. Return
    # None so the caller can count it rather than mistake it for "no change".
    if "inside an inflated obstacle" in (result.message or ""):
        return None

    polygons = None
    for step in recorder.steps:
        if step["kind"] == "obstacles":
            polygons = step["polygons"]
            break
    if polygons is None:
        return {}

    # Labels carry the ROBOT's identity, never its index in the obstacle list.
    # `polygons[i]` is built from `request.obstacles[i]`, so the mapping is
    # positional here but the label is not: in a real match a robot drops out of
    # vision for a frame, every later index shifts, and positional labels would
    # match nothing between the two frames. That silently cut 4197 captured
    # frames down to 448 usable pairs before this was fixed.
    #
    # Rounded because the same vertex reaches us through two different float
    # paths (the polygon list and the edge_test endpoints) and must hash
    # identically.
    label_of: dict[tuple[int, int], str] = {}
    for poly_idx, poly in enumerate(polygons):
        obstacle = request.obstacles[poly_idx] if poly_idx < len(request.obstacles) else None
        who = (
            f"{'y' if obstacle.isYellow else 'b'}{obstacle.robot_id}"
            if obstacle is not None
            else f"p{poly_idx}"
        )
        for vert_idx, (vx, vy) in enumerate(poly):
            label_of[(round(vx), round(vy))] = f"{who}v{vert_idx}"
    label_of[(round(request.start_mm[0]), round(request.start_mm[1]))] = "start"
    label_of[(round(request.goal_mm[0]), round(request.goal_mm[1]))] = "goal"

    states: dict[frozenset[str], bool] = {}
    for step in recorder.steps:
        if step["kind"] != "edge_test":
            continue
        a = label_of.get((round(step["a"][0]), round(step["a"][1])))
        b = label_of.get((round(step["b"][0]), round(step["b"][1])))
        if a is None or b is None or a == b:
            continue
        states[frozenset((a, b))] = bool(step["accepted"])
    return states


def churn(before: dict[frozenset[str], bool], after: dict[frozenset[str], bool]) -> dict:
    """Compare two frames' edge sets.

    `flipped_frac` is over edges present in BOTH frames -- that is the quantity
    comparable to Lim et al.'s "10% of edges changed". Edges that appear or
    vanish because a robot entered or left vision are reported separately
    rather than folded in, since they are a different phenomenon.
    """
    shared = before.keys() & after.keys()
    flipped = sum(1 for e in shared if before[e] != after[e])
    return {
        "shared": len(shared),
        "flipped": flipped,
        "flipped_frac": (flipped / len(shared)) if shared else 0.0,
        "appeared": len(after.keys() - before.keys()),
        "vanished": len(before.keys() - after.keys()),
    }


def point_to_path_mm(point: tuple[float, float], waypoints) -> float:
    """Shortest distance from a point to a polyline.

    Exact per-segment projection, not the 20 mm sampling `min_clearance_mm` in
    benchmark_planner_sweep.py uses: this feeds a 90 mm threshold decision, so
    sampling error would change the replan count.
    """
    if len(waypoints) < 2:
        return float("inf")
    px, py = point
    best = float("inf")
    for (x0, y0), (x1, y1) in pairwise(waypoints):
        dx, dy = x1 - x0, y1 - y0
        seg_sq = dx * dx + dy * dy
        if seg_sq == 0.0:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg_sq))
        best = min(best, math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)))
    return best


def path_invalidated(waypoints, obstacles, threshold_mm: float = INVALIDATION_THRESHOLD_MM) -> bool:
    """da Silva Costa & Tonidandel's trigger: any obstacle within `threshold` of the path."""
    return any(point_to_path_mm(o.pos_mm, waypoints) <= threshold_mm for o in obstacles)


def analyse(captures: list[Capture], strides, start_mm, goal_mm) -> dict:
    """Churn between frames separated by each stride in `strides`."""
    results: dict[int, dict] = {}
    cache: dict[int, dict[frozenset[str], bool]] = {}

    def states_at(i: int):
        if i not in cache:
            cache[i] = edge_states(
                PlanRequest(start_mm=start_mm, goal_mm=goal_mm, obstacles=captures[i].obstacles)
            )
        return cache[i]

    blocked = 0
    for stride in strides:
        fracs, gaps_ms, moved, invalidated = [], [], [], []
        # Step by `stride` so each pair is disjoint: overlapping pairs would
        # correlate the samples and understate the spread.
        for i in range(0, len(captures) - stride, stride):
            j = i + stride
            sa, sb = states_at(i), states_at(j)
            if sa is None or sb is None:
                # An endpoint was occupied in one of the two frames.
                blocked += 1
                continue
            c = churn(sa, sb)
            if c["shared"] == 0:
                continue
            fracs.append(c["flipped_frac"])
            gaps_ms.append((captures[j].t_s - captures[i].t_s) * 1000.0)

            before = {(o.isYellow, o.robot_id): o.pos_mm for o in captures[i].obstacles}
            moved.append(
                sum(
                    1
                    for o in captures[j].obstacles
                    if (o.isYellow, o.robot_id) in before
                    and math.dist(o.pos_mm, before[(o.isYellow, o.robot_id)]) > INVALIDATION_THRESHOLD_MM
                )
            )

            # Would the path planned at frame i still be legal at frame j?
            plan = visibility_graph.plan(
                PlanRequest(start_mm=start_mm, goal_mm=goal_mm, obstacles=captures[i].obstacles),
                skip_direct_path=True,
            )
            invalidated.append(
                bool(plan.success)
                and path_invalidated(plan.waypoints_mm, captures[j].obstacles)
            )

        if fracs:
            results[stride] = {
                "n": len(fracs),
                "gap_ms": statistics.median(gaps_ms),
                "flipped_frac_median": statistics.median(fracs),
                "flipped_frac_max": max(fracs),
                "moved_median": statistics.median(moved),
                "invalidated_frac": sum(invalidated) / len(invalidated),
            }
    results["_blocked_pairs"] = blocked
    return results


def frame_step_mm(captures: list[Capture]) -> list[float]:
    """Per-robot displacement between consecutive frames, sorted.

    At 70 Hz this is dominated by vision jitter for a stationary robot, so its
    median doubles as a noise floor. Reported alongside speed because summing
    these steps is exactly how a jitter-inflated speed arises.
    """
    steps: list[float] = []
    for a, b in pairwise(captures):
        prev = {(o.isYellow, o.robot_id): o.pos_mm for o in a.obstacles}
        for o in b.obstacles:
            k = (o.isYellow, o.robot_id)
            if k in prev:
                steps.append(math.dist(o.pos_mm, prev[k]))
    return sorted(steps)


def measured_speeds_mps(captures: list[Capture], window_s: float = 0.5) -> list[float]:
    """Per-robot speed from NET displacement over `window_s`, m/s.

    Summing per-frame steps is wrong here: real vision jitters, and a robot
    standing still with +/-20 mm of jitter at 70 Hz sums to an apparent
    1.6 m/s while its net displacement is zero. That produced a table claiming
    1.645 m/s median speed alongside a column saying no robot ever moved 90 mm,
    which cannot both be true. Net displacement over a window is jitter-robust:
    the noise averages out, real travel does not.
    """
    if len(captures) < 2:
        return []
    speeds: list[float] = []
    i = 0
    while i < len(captures):
        t_end = captures[i].t_s + window_s
        j = i
        while j < len(captures) and captures[j].t_s < t_end:
            j += 1
        if j >= len(captures):
            break
        dt = captures[j].t_s - captures[i].t_s
        prev = {(o.isYellow, o.robot_id): o.pos_mm for o in captures[i].obstacles}
        for o in captures[j].obstacles:
            k = (o.isYellow, o.robot_id)
            if k in prev and dt > 0:
                speeds.append(math.dist(o.pos_mm, prev[k]) / dt / 1000.0)
        i = j
    return sorted(speeds)


def report(
    results: dict,
    captures: list[Capture],
    stir_mps: float = 0.0,
    source: str = "grsim",
    label: str = "live grSim",
) -> None:
    obstacle_counts = {len(c.obstacles) for c in captures}
    speeds = measured_speeds_mps(captures)
    steps = frame_step_mm(captures)
    speed_note = (
        f"robot speed from net displacement over 0.5 s: median "
        f"{speeds[len(speeds) // 2]:.3f} m/s, p95 {speeds[int(len(speeds) * 0.95)]:.3f}, "
        f"max {speeds[-1]:.3f} m/s"
        if speeds
        else "robot speed not measurable"
    )
    noise_note = (
        f"per-frame step: median {steps[len(steps) // 2]:.1f} mm "
        f"(the vision noise floor at this frame rate)"
        if steps
        else ""
    )
    if source == "log":
        origin = f"REAL MATCH: {label}"
    elif stir_mps > 0:
        origin = f"synthetic: robots commanded at {stir_mps:.1f} m/s"
    else:
        origin = "STATIC scene, nothing is driving the robots"
    print(
        f"\n{len(captures)} frames over {captures[-1].t_s:.1f} s "
        f"({len(captures) / captures[-1].t_s:.1f} Hz), "
        f"{sorted(obstacle_counts)} obstacles per frame\n"
        f"  {origin}\n"
        f"  {speed_note}\n"
        f"  {noise_note}\n"
    )
    header = (
        f"{'stride':>7}{'gap ms':>9}{'n pairs':>9}{'edges flipped %':>17}"
        f"{'worst %':>10}{'robots moved >90mm':>20}{'path invalid %':>16}"
    )
    print("  " + header)
    print("  " + "-" * len(header))
    blocked = results.get("_blocked_pairs", 0)
    for stride, r in sorted((k, v) for k, v in results.items() if isinstance(k, int)):
        print(
            f"  {stride:>7}{r['gap_ms']:>9.1f}{r['n']:>9}"
            f"{r['flipped_frac_median'] * 100:>17.2f}{r['flipped_frac_max'] * 100:>10.2f}"
            f"{r['moved_median']:>20.0f}{r['invalidated_frac'] * 100:>16.1f}"
        )
    if blocked:
        print(
            f"\n  {blocked} pair(s) skipped: an endpoint sat inside an inflated obstacle in one\n"
            f"  of the two frames, so the planner returned before testing any edge\n"
            f"  (visibility_graph.py:304). Keep start/goal off the goal mouths -- a keeper\n"
            f"  parks within the 210 mm inflation radius there for most of a real match."
        )
    print(
        "\n  edges flipped %: of edges present in BOTH frames, the share whose visibility\n"
        "                   changed. This is the quantity comparable to Lim et al.'s\n"
        "                   '10% of edges changed' between consecutive searches.\n"
        f"  robots moved:    obstacles that moved more than {INVALIDATION_THRESHOLD_MM:.0f} mm, the\n"
        "                   invalidation threshold used by da Silva Costa & Tonidandel.\n"
        "  path invalid %:  share of pairs where the path planned at the earlier frame had\n"
        "                   an obstacle within 90 mm of it at the later frame -- i.e. the\n"
        "                   replan rate their trigger would actually produce.\n"
    )


def selftest() -> None:
    """Checks the measurement, not the planners."""
    # Point-to-path geometry, which decides the 90 mm threshold.
    path = ((0.0, 0.0), (1000.0, 0.0))
    assert abs(point_to_path_mm((500.0, 300.0), path) - 300.0) < 1e-9
    assert abs(point_to_path_mm((500.0, 0.0), path) - 0.0) < 1e-9
    # Beyond the segment end it must measure to the ENDPOINT, not the infinite
    # line: an obstacle 2 m past the goal is not on the path.
    assert abs(point_to_path_mm((3000.0, 0.0), path) - 2000.0) < 1e-9
    assert point_to_path_mm((0.0, 0.0), ((1.0, 1.0),)) == float("inf")

    assert path_invalidated(path, (Obstacle(pos_mm=(500.0, 50.0)),))
    assert not path_invalidated(path, (Obstacle(pos_mm=(500.0, 500.0)),))

    # Edge identity: an unmoved scene must produce zero churn, and a scene with
    # one robot shifted must produce some.
    start, goal = (-4000.0, 0.0), (4000.0, 0.0)
    obs_a = _sorted_obstacles(
        (
            Obstacle(pos_mm=(0.0, 200.0), robot_id=1, isYellow=True),
            Obstacle(pos_mm=(1500.0, -800.0), robot_id=2, isYellow=True),
            Obstacle(pos_mm=(-1200.0, 600.0), robot_id=3, isYellow=False),
        )
    )
    s_a = edge_states(PlanRequest(start_mm=start, goal_mm=goal, obstacles=obs_a))
    assert s_a, "no edge tests recorded -- the recorder contract changed"
    assert churn(s_a, s_a)["flipped"] == 0, "identical scenes must not churn"

    obs_b = _sorted_obstacles(
        (
            Obstacle(pos_mm=(0.0, 200.0), robot_id=1, isYellow=True),
            Obstacle(pos_mm=(1500.0, -800.0), robot_id=2, isYellow=True),
            Obstacle(pos_mm=(400.0, -100.0), robot_id=3, isYellow=False),  # moved a long way
        )
    )
    s_b = edge_states(PlanRequest(start_mm=start, goal_mm=goal, obstacles=obs_b))
    c = churn(s_a, s_b)
    assert c["shared"] > 0, "labels did not line up between frames"
    assert c["flipped"] > 0, "a large obstacle move produced no visibility change"

    # Sorting must be stable regardless of input order, or poly indices drift.
    assert [o.pos_mm for o in _sorted_obstacles(tuple(reversed(obs_a)))] == [
        o.pos_mm for o in obs_a
    ]

    print("selftest OK")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=20.0, help="Capture duration")
    parser.add_argument(
        "--strides",
        type=int,
        nargs="+",
        default=[1, 2, 3, 6, 12, 30, 60],
        help="Frame gaps to analyse (1 = consecutive frames, 60 = about one second)",
    )
    parser.add_argument(
        "--stir",
        type=float,
        default=0.0,
        metavar="MPS",
        help="Drive the robots at this speed during capture (m/s). An idle grSim is "
             "static and yields exactly zero churn. SSL robots exceed 2.5 m/s.",
    )
    parser.add_argument(
        "--log",
        metavar="PATH",
        help="Replay a recorded SSL match (.log or .log.gz) instead of live grSim. "
             "Real match motion; strongly preferred over --stir.",
    )
    parser.add_argument("--skip", type=float, default=0.0,
                        help="Seconds of log to skip before capturing, to get past a stoppage")
    parser.add_argument("--cameras", type=int, default=None,
                        help="Override the camera count (default: detect from the log)")
    parser.add_argument("--selftest", action="store_true")
    # NOT the goal mouths: a keeper parks within the 210 mm inflation radius of
    # (+/-4000, 0) for most of a real match, which blocks the query outright.
    parser.add_argument("--start", type=float, nargs=2, default=[-3500.0, -2000.0])
    parser.add_argument("--goal", type=float, nargs=2, default=[3500.0, 2000.0])
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return

    if args.log:
        print(f"replaying {args.seconds:.0f} s of {Path(args.log).name}"
              f"{f' from +{args.skip:.0f} s' if args.skip else ''} ...")
        captures = capture_from_logfile(
            args.log, args.seconds, cameras=args.cameras, skip_seconds=args.skip
        )
        source = "log"
    else:
        stir_note = f", robots stirred at {args.stir:.1f} m/s" if args.stir > 0 else " (STATIC scene)"
        print(f"capturing {args.seconds:.0f} s of vision{stir_note} ...")
        captures = capture_from_grsim(args.seconds, stir_mps=args.stir)
        source = "grsim"
    if len(captures) < max(args.strides) + 1:
        hint = "is the log path right?" if args.log else "is grSim running?"
        print(f"only {len(captures)} frames captured -- {hint}")
        return
    strides = [s for s in args.strides if s < len(captures)]
    report(
        analyse(captures, strides, tuple(args.start), tuple(args.goal)),
        captures,
        args.stir,
        source=source,
        label=Path(args.log).name if args.log else "live grSim",
    )


if __name__ == "__main__":
    main()
