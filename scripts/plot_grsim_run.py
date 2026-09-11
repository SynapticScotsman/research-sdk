#!/usr/bin/env python3
"""Draw a saved grSim run as an SVG: recorded obstacle tracks and the robot's path.

`drive_grsim.py --out-json` records both tracks at 4 Hz, so a run can be drawn
after the fact without rerunning it or capturing the screen. That matters here
because WSLg returns a 0x0 grab through Qt and installing a screenshot tool
needs a sudo password, so the saved data is the only practical route to a
figure.

SVG rather than matplotlib because the output is embedded in the student guide,
which is dark by default: an SVG inherits the page's colours and stays readable,
where a rendered PNG carries whichever background it was drawn on.

Usage:
    python scripts/plot_grsim_run.py results/grsim/voronoi_1v6_skip180_30s.json
    python scripts/plot_grsim_run.py RUN.json --out figure.svg
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

# Division B, from config/ssl_field_config.yaml.
FIELD_X, FIELD_Y = 9000.0, 6000.0
MARGIN = 26.0
SCALE = 0.062                 # px per mm: a 9000 mm field lands at 558 px

PLATE, GRID, DIM = "#05070A", "#1C242C", "#8A97A0"
OBSTACLE, ROBOT, START, GOAL = "#C33A50", "#7CE0C0", "#FFFFFF", "#FFB000"


def _sx(x: float) -> float:
    return MARGIN + (x + FIELD_X / 2) * SCALE


def _sy(y: float) -> float:
    # +y is up the pitch and down the SVG.
    return MARGIN + (FIELD_Y / 2 - y) * SCALE


def render(run: dict) -> str:
    meta, prov = run["meta"], run.get("provenance", {})
    replay = meta.get("replay") or {}
    robot = run["robots"][0]
    w = FIELD_X * SCALE + 2 * MARGIN
    h = FIELD_Y * SCALE + 2 * MARGIN + 30

    out = [f'<svg viewBox="0 0 {w:.0f} {h:.0f}" role="img" aria-label="'
           f'One controlled robot crossing the field while six recorded robots replay '
           f'from a match. The robot completed {robot["laps"]} crossings with '
           f'{robot["replans"]} replans.">',
           f'<rect x="0" y="0" width="{w:.0f}" height="{h:.0f}" fill="{PLATE}"/>',
           f'<rect x="{_sx(-FIELD_X/2):.1f}" y="{_sy(FIELD_Y/2):.1f}" '
           f'width="{FIELD_X*SCALE:.1f}" height="{FIELD_Y*SCALE:.1f}" fill="none" '
           f'stroke="{GRID}" stroke-width="1"/>',
           f'<line x1="{_sx(0):.1f}" y1="{_sy(FIELD_Y/2):.1f}" x2="{_sx(0):.1f}" '
           f'y2="{_sy(-FIELD_Y/2):.1f}" stroke="{GRID}" stroke-width="1"/>']

    # Recorded obstacles: one polyline per slot, transposed out of the frames.
    frames = replay.get("obstacle_track") or []
    slots = max((len(f) for f in frames), default=0)
    for slot in range(slots):
        pts = [f[slot] for f in frames if slot < len(f)]
        if len(pts) < 2:
            continue
        d = " ".join(f"{_sx(x):.1f},{_sy(y):.1f}" for x, y in pts)
        out.append(f'<polyline points="{d}" fill="none" stroke="{OBSTACLE}" '
                   f'stroke-width="1" opacity="0.55"/>')
        out.append(f'<circle cx="{_sx(pts[-1][0]):.1f}" cy="{_sy(pts[-1][1]):.1f}" '
                   f'r="{90*SCALE:.1f}" fill="{OBSTACLE}" opacity="0.85"/>')

    track = robot.get("track") or []
    if len(track) >= 2:
        d = " ".join(f"{_sx(x):.1f},{_sy(y):.1f}" for x, y in track)
        out.append(f'<polyline points="{d}" fill="none" stroke="{ROBOT}" '
                   f'stroke-width="1.8" stroke-linejoin="round"/>')
        out.append(f'<circle cx="{_sx(track[0][0]):.1f}" cy="{_sy(track[0][1]):.1f}" '
                   f'r="3" fill="{START}"/>')

    for x, label in ((-3200.0, "start"), (3200.0, "goal")):
        out.append(f'<circle cx="{_sx(x):.1f}" cy="{_sy(0):.1f}" r="3.4" fill="none" '
                   f'stroke="{GOAL}" stroke-width="1.4"/>')
        out.append(f'<text x="{_sx(x):.1f}" y="{_sy(0) - 9:.1f}" fill="{GOAL}" '
                   f'font-family="ui-monospace, Menlo, monospace" font-size="9" '
                   f'text-anchor="middle">{label}</text>')

    g = prov.get("grsim", {})
    line1 = (f'{meta["planner"]} planner  ·  {robot["laps"]} crossings  ·  '
             f'{robot["replans"]} replans  ·  {robot["mean_plan_ms"]:.2f} ms mean  ·  '
             f'closest {robot["closest_mm"]:.0f} mm by vision')
    line2 = (f'clip {meta.get("log_skip_s", 0):.0f}-'
             f'{meta.get("log_skip_s", 0) + meta.get("log_seconds", 0):.0f} s  ·  '
             f'{slots} recorded robots replayed  ·  noise '
             f'{float(g.get("noise_x_mm", 0)):.0f} mm, delay {g.get("sending_delay_ms", "?")} ms')
    out.append(f'<text x="{MARGIN:.0f}" y="{h-18:.0f}" fill="{DIM}" '
               f'font-family="ui-monospace, Menlo, monospace" font-size="9.5">{line1}</text>')
    out.append(f'<text x="{MARGIN:.0f}" y="{h-6:.0f}" fill="{DIM}" '
               f'font-family="ui-monospace, Menlo, monospace" font-size="9.5">{line2}</text>')
    out.append(f'<text x="{w-MARGIN:.0f}" y="{MARGIN-10:.0f}" fill="{DIM}" '
               f'font-family="ui-monospace, Menlo, monospace" font-size="9" '
               f'text-anchor="end">green: controlled robot · red: replayed match robots</text>')
    out.append("</svg>")
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_json")
    parser.add_argument("--out", help="Defaults to the input path with .svg")
    args = parser.parse_args()

    run = json.loads(Path(args.run_json).read_text(encoding="utf-8"))
    if not run.get("robots"):
        raise SystemExit("no robots in that results file")
    svg = render(run)
    out = Path(args.out or Path(args.run_json).with_suffix(".svg"))
    out.write_text(svg, encoding="utf-8")
    print(f"wrote {out} ({len(svg):,} characters)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
