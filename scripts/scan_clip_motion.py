"""Which 30 s window of the match log has the most robot motion?

The recorded clip every grSim comparison so far uses (skip 180 s, 30 s) was
chosen before anyone measured its motion. Measured afterwards at 4 Hz
sampling: only two of the eleven replayed robots move more than their own
footprint (B3 spans 8450 mm, B5 7784 mm); the yellow team spans 81 to 143 mm
in the whole clip. A replanning comparison on a field where nine obstacles
are parked mostly measures the parked case. This scans every window of the
log with the same reader the driver uses (measure_graph_churn.capture_from_logfile,
so the frames are the ones drive_grsim would replay) and ranks windows by how
many robots travel and how fast.

Speed is sampled at 4 Hz on purpose. Differentiating the 70 Hz vision stream
turns the 23 mm per-frame noise into about 1.6 m/s of apparent speed for a
parked robot; at 0.25 s intervals the same noise adds about 0.13 m/s, which is
the floor to read every speed against. "Moving" means the span of a robot's
extreme positions in the window exceeds --min-span-mm, not its path length: a
robot jittering in place accumulates metres of path without leaving its
footprint.

Usage:
    python scripts/scan_clip_motion.py                      # the Windows copy of the log
    python scripts/scan_clip_motion.py --log ~/ssl-gamelogs/2024-07-19_TurtleRabbit-vs-NAMeC.log.gz
    python scripts/scan_clip_motion.py --window-s 30 --step-s 15 --top 12
    python scripts/scan_clip_motion.py --selftest
"""
from __future__ import annotations

import argparse
import math
import os
import platform
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))

CLIP = "2024-07-19_TurtleRabbit-vs-NAMeC.log.gz"
SAMPLE_S = 0.25          # 4 Hz: the rate the existing per-robot speeds were measured at
NOISE_FLOOR_MPS = 0.13   # what 23 mm vision noise alone reads as at 4 Hz


def default_log() -> Path:
    if platform.system() == "Windows":
        return Path.home() / "grsim" / "logs" / CLIP
    return Path.home() / "ssl-gamelogs" / CLIP


def window_stats(captures, start_s: float, window_s: float, min_span_mm: float,
                 coverage_gate: float = 0.5) -> dict:
    """Per-robot span and 4 Hz speed inside [start_s, start_s + window_s).

    Coverage gate and span definition match LogOpponents in drive_grsim.py, so a
    robot counted as moving here is one the driver would select and replay.
    """
    frames = [c for c in captures if start_s <= c.t_s < start_s + window_s]
    if not frames:
        return {"start_s": start_s, "frames": 0, "tracked": 0, "moving": 0,
                "mean_speed_mps": 0.0, "max_speed_mps": 0.0, "total_span_mm": 0.0, "robots": {}}
    seen: dict[tuple[bool, int], list] = {}
    for c in frames:
        for o in c.obstacles:
            seen.setdefault((bool(o.isYellow), int(o.robot_id)), []).append((c.t_s, o.pos_mm))
    robots = {}
    for key, track in seen.items():
        if len(track) < coverage_gate * len(frames):
            continue
        xs = [p[0] for _, p in track]
        ys = [p[1] for _, p in track]
        span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        # 4 Hz resample: the first observation at or after each sample instant.
        samples = []
        next_t = track[0][0]
        for t, p in track:
            if t >= next_t:
                samples.append((t, p))
                next_t = t + SAMPLE_S
        steps = [math.dist(b[1], a[1]) / (b[0] - a[0])
                 for a, b in zip(samples, samples[1:]) if b[0] > a[0]]
        speed = (sum(steps) / len(steps) / 1000.0) if steps else 0.0
        robots[key] = {"span_mm": span, "speed_mps": speed, "coverage": len(track) / len(frames)}
    moving = [r for r in robots.values() if r["span_mm"] >= min_span_mm]
    return {
        "start_s": start_s,
        "frames": len(frames),
        "tracked": len(robots),
        "moving": len(moving),
        "mean_speed_mps": (sum(r["speed_mps"] for r in moving) / len(moving)) if moving else 0.0,
        "max_speed_mps": max((r["speed_mps"] for r in robots.values()), default=0.0),
        "total_span_mm": sum(r["span_mm"] for r in robots.values()),
        "robots": robots,
    }


def scan(captures, window_s: float, step_s: float, min_span_mm: float) -> list[dict]:
    end = captures[-1].t_s
    starts = []
    t = 0.0
    while t + window_s <= end + 1e-9:
        starts.append(t)
        t += step_s
    return [window_stats(captures, s, window_s, min_span_mm) for s in starts]


def report(rows: list[dict], top: int, current_skip: float | None) -> None:
    ranked = sorted(rows, key=lambda r: (-r["moving"], -r["mean_speed_mps"]))
    print(f"\n  {len(rows)} windows scanned, log ends at {rows[-1]['start_s'] + 0:.0f}+ s; "
          f"speed floor from vision noise at 4 Hz is about {NOISE_FLOOR_MPS} m/s")
    header = f"{'skip s':>8}{'frames':>8}{'tracked':>9}{'moving':>8}{'mean m/s':>10}{'max m/s':>9}{'total span m':>14}"
    print("  " + header)
    print("  " + "-" * len(header))
    for r in ranked[:top]:
        print(f"  {r['start_s']:>8.0f}{r['frames']:>8}{r['tracked']:>9}{r['moving']:>8}"
              f"{r['mean_speed_mps']:>10.2f}{r['max_speed_mps']:>9.2f}{r['total_span_mm'] / 1000:>14.1f}")
    if current_skip is not None:
        cur = min(rows, key=lambda r: abs(r["start_s"] - current_skip))
        rank = ranked.index(cur) + 1
        print(f"\n  the clip in use (skip {cur['start_s']:.0f} s) ranks {rank} of {len(rows)}: "
              f"{cur['moving']} moving of {cur['tracked']} tracked, mean {cur['mean_speed_mps']:.2f} m/s")
    print("\n  moving = span of extreme positions in the window >= --min-span-mm; tracked = present in"
          "\n  at least half the frames (the driver's gate). mean m/s is over moving robots only."
          "\n  Sanity limits: one robot's span cannot exceed the field diagonal with margins (about 13 m)"
          "\n  and SSL robots do not exceed about 4 m/s. A window breaking either has an identity jump"
          "\n  (two physical robots under one id) and is not a livelier clip; use --detail to see it.")


def detail(captures, starts, window_s: float, min_span_mm: float) -> None:
    """Per-robot span, speed and coverage for chosen windows.

    The ranking above can be fooled by an identity jump: a single (team, id)
    key tracked on two physical robots reads as one robot teleporting, which
    inflates both span and speed. Spans above the field diagonal or speeds
    above 4 m/s are that, not motion.
    """
    for s in starts:
        w = window_stats(captures, s, window_s, min_span_mm)
        print(f"\n  window skip {s:.0f} s: {w['frames']} frames, {w['tracked']} tracked, {w['moving']} moving")
        print(f"  {'robot':>10}{'coverage':>10}{'span m':>9}{'m/s':>7}  flag")
        for key, r in sorted(w["robots"].items(), key=lambda kv: -kv[1]["span_mm"]):
            name = f"{'Y' if key[0] else 'B'}{key[1]}"
            flag = "ID JUMP?" if r["span_mm"] > 13000 or r["speed_mps"] > 4.0 else ("moving" if r["span_mm"] >= min_span_mm else "")
            print(f"  {name:>10}{r['coverage']:>10.2f}{r['span_mm'] / 1000:>9.1f}{r['speed_mps']:>7.2f}  {flag}")


def _synthetic():
    """Two robots for 60 s at 70 Hz: one crossing at 1 m/s, one parked with 23 mm noise."""
    import random
    from measure_graph_churn import Capture, Obstacle

    rng = random.Random(0)
    caps = []
    t = 0.0
    while t < 60.0:
        mover = Obstacle(robot_id=3, isYellow=False, pos_mm=(-4000.0 + 1000.0 * (t % 30), 0.0), radius_mm=90.0)
        parked = Obstacle(robot_id=1, isYellow=True,
                          pos_mm=(1000.0 + rng.gauss(0, 23), 500.0 + rng.gauss(0, 23)), radius_mm=90.0)
        caps.append(Capture(t_s=t, obstacles=(mover, parked)))
        t += 1 / 70
    return caps


def selftest() -> None:
    rows = scan(_synthetic(), window_s=30.0, step_s=30.0, min_span_mm=1000.0)
    assert len(rows) == 2, len(rows)
    w = rows[0]
    assert w["tracked"] == 2 and w["moving"] == 1, w
    mover = w["robots"][(False, 3)]
    parked = w["robots"][(True, 1)]
    assert abs(mover["speed_mps"] - 1.0) < 0.05, mover
    assert parked["speed_mps"] < 0.2, parked          # noise floor, not motion
    assert parked["span_mm"] < 300, parked
    print("scan_clip_motion selftest ok")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", type=Path, default=default_log())
    ap.add_argument("--window-s", type=float, default=30.0)
    ap.add_argument("--step-s", type=float, default=15.0)
    ap.add_argument("--min-span-mm", type=float, default=1000.0)
    ap.add_argument("--top", type=int, default=12)
    ap.add_argument("--current-skip", type=float, default=180.0, help="the clip in use, for its rank")
    ap.add_argument("--detail", type=float, nargs="*", default=[],
                    help="window start(s) to print per robot, after the ranking")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args(argv)
    if args.selftest:
        selftest()
        return 0
    from measure_graph_churn import capture_from_logfile

    captures = capture_from_logfile(str(args.log), seconds=float("inf"), skip_seconds=0.0)
    print(f"  {len(captures)} frames, {captures[-1].t_s:.0f} s of vision")
    rows = scan(captures, args.window_s, args.step_s, args.min_span_mm)
    report(rows, args.top, args.current_skip)
    if args.detail:
        detail(captures, args.detail, args.window_s, args.min_span_mm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
