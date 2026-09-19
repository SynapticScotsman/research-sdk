"""Paired multi-run grSim batches: repeats x planners on ONE simulator process.

The WSL reference sets results/grsim/div-b-11obs-y750-{90,180} were made by
overnight_runs.sh job 1. Three things about that design carry over here and
decide whether a batch counts as one comparison:

  * Planners ALTERNATE run to run (voronoi, visibility, prm, voronoi, ...) so
    that any drift in the simulator lands on all three equally rather than on
    whichever planner went last.
  * All 45 runs of a set ran on one grSim process (pid 449 in every provenance
    block of the reference sets). A restart mid-batch changed ms/build by
    1.5-1.6x (docs/grsim-scenario-validity.md), so this runner never restarts
    grSim itself and records the pid at the start and end of the batch. If
    they differ, the batch is two batches.
  * Every output has a unique name and existing files are skipped, so an
    interrupted batch resumes by rerunning the same command.

This is that job for Windows as well as WSL. On Windows with no grSim running
it starts the native headless build through scripts/grsim-windows.ps1, which
returns once vision is flowing, and reads the match log from a Windows path
(%USERPROFILE%\\grsim\\logs), so nothing here touches WSL. The launcher's
output is NOT captured: grSim is started without redirects on purpose (see the
launcher header), and capturing here would bring back the inherited-pipe hang
the launcher was rewritten to avoid.

Defaults are the reference sets' flags, read back from their meta blocks:
11 obstacles, both recorded teams replayed, traverse x = +-2400 mm on the
y = 750 mm lane, clip skip 180 s for 30 s, full roadmap rebuild at every
replan.

Usage:
    python scripts/batch_grsim.py --out-dir results/grsim/win-div-b-11obs-y750-90 --threshold-mm 90
    python scripts/batch_grsim.py --out-dir results/grsim/win-div-b-11obs-y750-180 --threshold-mm 180
    python scripts/batch_grsim.py --dry-run --out-dir x        # print the commands only
    python scripts/batch_grsim.py --selftest
Anything after `--` goes to drive_grsim.py unchanged.
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PLANNERS = ("voronoi", "visibility", "prm")
ON_WINDOWS = platform.system() == "Windows"
CLIP = "2024-07-19_TurtleRabbit-vs-NAMeC.log.gz"


def default_log() -> Path:
    if ON_WINDOWS:
        return Path.home() / "grsim" / "logs" / CLIP
    return Path.home() / "ssl-gamelogs" / CLIP


def schedule(repeats: int, planners, out_dir: Path) -> list[tuple[str, int, Path]]:
    """Run 1 of every planner, then run 2 of every planner: the alternation."""
    return [(p, i, out_dir / f"{p}_run{i:02d}.json")
            for i in range(1, repeats + 1) for p in planners]


def driver_command(python: str, planner: str, out: Path, args, extra) -> list[str]:
    cmd = [
        python, str(ROOT / "scripts" / "drive_grsim.py"),
        "--planner", planner, "--robots", "1", "--obstacles", str(args.obstacles),
        "--opponents", "log", "--log", str(args.log),
        "--log-skip", str(args.log_skip), "--log-seconds", str(args.duration),
        "--duration", str(args.duration),
        "--trigger", "geometric", "--trigger-centre-threshold-mm", str(args.threshold_mm),
        "--replay-team", args.replay_team, "--traverse-x", str(args.traverse_x),
        "--lane-centre-mm", str(args.lane_centre_mm),
        "--out-json", str(out),
    ]
    if args.force_full_build:
        cmd.append("--force-full-build")
    return cmd + list(extra)


def grsim_pid() -> int | None:
    if ON_WINDOWS:
        listing = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq grSim.exe", "/FO", "CSV", "/NH"],
            capture_output=True, text=True, timeout=15).stdout
        found = re.search(r'"grSim\.exe","(\d+)"', listing)
        return int(found.group(1)) if found else None
    out = subprocess.run(["pgrep", "-o", "grSim"], capture_output=True, text=True).stdout.strip()
    return int(out.split()[0]) if out else None


def ensure_grsim(say) -> int:
    pid = grsim_pid()
    if pid:
        say(f"grSim already running, pid {pid}; the batch uses it as is")
        return pid
    if not ON_WINDOWS:
        raise SystemExit("grSim is not running; start it first (WSL: ~/ssl-software/grSim/bin/grSim)")
    launcher = ROOT / "scripts" / "grsim-windows.ps1"
    rc = subprocess.run(
        ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(launcher), "-Headless"],
        timeout=120).returncode
    pid = grsim_pid()
    if rc != 0 or not pid:
        raise SystemExit(f"launcher exit {rc}, grSim pid {pid}; see the launcher's message above")
    say(f"started native headless grSim, pid {pid}")
    return pid


def parse(argv: list[str]):
    if "--" in argv:
        cut = argv.index("--")
        argv, extra = argv[:cut], argv[cut + 1:]
    else:
        extra = []
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", type=Path)
    ap.add_argument("--repeats", type=int, default=15)
    ap.add_argument("--planners", nargs="+", default=list(PLANNERS), choices=PLANNERS)
    ap.add_argument("--threshold-mm", type=float, default=90.0,
                    help="centre-to-path replan threshold; 90 reproduces the offline rule, 180 is the driver default")
    ap.add_argument("--obstacles", type=int, default=11)
    ap.add_argument("--replay-team", default="both", choices=("yellow", "blue", "both"))
    ap.add_argument("--traverse-x", type=float, default=2400.0)
    ap.add_argument("--lane-centre-mm", type=float, default=750.0)
    ap.add_argument("--log", type=Path, default=default_log())
    ap.add_argument("--log-skip", type=float, default=180.0)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--no-force-full-build", dest="force_full_build", action="store_false")
    ap.add_argument("--timeout-s", type=float, default=120.0, help="per run; a 30 s run takes about 40 s")
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ns = ap.parse_args(argv)
    if not ns.selftest and ns.out_dir is None:
        ap.error("--out-dir is required")
    return ns, extra


def run_batch(args, extra) -> int:
    out_dir: Path = args.out_dir
    jobs = schedule(args.repeats, args.planners, out_dir)
    if args.dry_run:
        for planner, i, out in jobs:
            print(subprocess.list2cmdline(driver_command(args.python, planner, out, args, extra)))
        return 0
    out_dir.mkdir(parents=True, exist_ok=True)
    log = (out_dir / "batch.log").open("a", encoding="utf-8")

    def say(msg: str) -> None:
        line = f"[{dt.datetime.now():%H:%M:%S}] {msg}"
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    if not args.log.is_file():
        raise SystemExit(f"no match log at {args.log}")
    pid0 = ensure_grsim(say)
    say(f"batch start: {len(jobs)} runs into {out_dir}, threshold {args.threshold_mm:.0f} mm, "
        f"grSim pid {pid0}, {platform.system()} {platform.release()}")
    env = dict(os.environ, PYTHONPATH=str(ROOT / "src"))
    ok = failed = skipped = 0
    for planner, i, out in jobs:
        if out.exists():
            skipped += 1
            say(f"skip {out.name}")
            continue
        cmd = driver_command(args.python, planner, out, args, extra)
        t0 = time.monotonic()
        try:
            r = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True,
                               timeout=args.timeout_s)
        except subprocess.TimeoutExpired:
            failed += 1
            say(f"{planner} run {i} TIMEOUT after {args.timeout_s:.0f} s")
            continue
        tail = (r.stdout + r.stderr).strip().splitlines()[-1:] or [""]
        if r.returncode == 0 and out.exists():
            ok += 1
            say(f"{planner} run {i} ok, {time.monotonic() - t0:.0f} s: {tail[0][:110]}")
        else:
            failed += 1
            say(f"{planner} run {i} FAILED rc {r.returncode}: {tail[0][:160]}")
    pid1 = grsim_pid()
    note = "" if pid1 == pid0 else " (grSim RESTARTED mid-batch: these are not one set)"
    say(f"batch done: {ok} ok, {failed} failed, {skipped} skipped; grSim pid {pid0} -> {pid1}{note}")
    return 0 if failed == 0 else 1


def selftest() -> None:
    jobs = schedule(2, PLANNERS, Path("x"))
    assert [j[0] for j in jobs] == ["voronoi", "visibility", "prm"] * 2, jobs
    assert jobs[3][2].name == "voronoi_run02.json"
    ns, extra = parse(["--out-dir", "x", "--threshold-mm", "90", "--", "--allow-extra-robots"])
    cmd = driver_command("py", "prm", Path("o.json"), ns, extra)
    assert cmd[cmd.index("--trigger-centre-threshold-mm") + 1] == "90.0"
    assert cmd[-1] == "--allow-extra-robots" and "--force-full-build" in cmd
    assert cmd[cmd.index("--log-seconds") + 1] == cmd[cmd.index("--duration") + 1] == "30.0"
    ns, _ = parse(["--out-dir", "x", "--no-force-full-build"])
    assert "--force-full-build" not in driver_command("py", "prm", Path("o"), ns, [])
    print("batch_grsim selftest ok")


if __name__ == "__main__":
    ns, extra = parse(sys.argv[1:])
    if ns.selftest:
        selftest()
    else:
        sys.exit(run_batch(ns, extra))
