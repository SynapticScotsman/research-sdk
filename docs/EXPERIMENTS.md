# Which script produces which result

Every number in `docs/paper_sections.tex` comes from one of these. Each script
takes `--selftest`, which checks the properties the measurement is worthless
without. Run that first after any edit.

## Two checkouts

The scripts live in two places and it matters which venv you use.

| Checkout | Branch | Holds |
|---|---|---|
| `research-sdk` | `paulk/replanning-policy-experiments` | scenarios, triggers, stability |
| `research-sdk-plannertest` | `paulk/dstar-lite-shared-search` | shared D* Lite, gap and reuse work |

The `research-sdk` scripts run under its own `.venv`. The `plannertest` ones
need that worktree on the import path, because they exercise the shared search:

```
PYTHONPATH="<plannertest>/src" python scripts/measure_search_reuse.py --selftest
```

For anything touching grSim, run inside WSL from `~/rsdk-venv`. WSL2 uses NAT
networking, so multicast does not reach Windows, and the fix needs Windows 11.

## The map

| Result | Script | Command |
|---|---|---|
| Static scenarios, Table 2 | `static_scenario.py` | `--samples 200` |
| Replication anchor, 13.37 replans | `dynamic_scenario.py` | `--samples 200` |
| Trigger comparison, Tables 3 and 4 | `dynamic_scenario.py` | `--samples 200 --sweep-triggers --workers 8` |
| Collisions against speed, Table 5 | `dynamic_scenario.py` | `--samples 100 --sweep-speeds --noise-mm 23` |
| Path stability, Table 7 | `report_stability.py` | `--samples 200 --workers 8` |
| Shared D* Lite, Table 8 | `measure_search_reuse.py` | `--source log --log MATCH.log.gz --repeats 5` |
| Roadmap stability by site placement | `measure_search_reuse.py` | `--voronoi-placement grid --voronoi-spacing 1200` |
| Gap resolution | `measure_gap_resolution.py` | (no arguments needed) |
| Voronoi build cost by lattice spacing | `probe_voronoi_sites.py` | `<repo root>` |
| Planning load on the UI thread | `benchmark_ui_load.py` | (no arguments needed) |
| Watch it run in grSim | `drive_grsim.py` | `--planner voronoi --duration 120` |
| Closed-loop validation, 6v6, real opponents | `drive_grsim.py` | `--robots 6 --obstacles 6 --opponents log --log LOG --out-json OUT` |
| Turn grSim's idealisations off first | `set_grsim_realism.sh` | `bash scripts/set_grsim_realism.sh` |
| Fetch the match log those need | `fetch_match_log.sh` | `bash scripts/fetch_match_log.sh` |

## The match log

Two measurements replay a recorded game rather than a synthetic scene, because
roadmap churn is a property of how robots actually move. `fetch_match_log.sh`
downloads it, checks the gzip and the `SSL_LOG_FILE` header, and prints the path
to pass as `--log`. It lands in `$HOME/ssl-gamelogs`, not the repo: 159 MB in a
OneDrive working copy would sync forever.

## Scenarios and triggers

Both live at the top of `dynamic_scenario.py`.

`SCENARIOS` holds the two obstacle layouts, seven robots and five, each robot
patrolling a closed polyline at constant speed. `TRIGGERS` holds eleven
replanning policies: the source study's geometric test, three fixed timers,
three velocity projections, a smoothed projection, and near-field commitment
over one or two path segments.

Add a `Loop` to a scenario or a function to `TRIGGERS` and it flows through the
sweeps, the stability report and the saved JSON without further changes.

```
python scripts/dynamic_scenario.py --samples 50 --obstacle-speed 1.5
python scripts/dynamic_scenario.py --plot scenario.png
```

`--plot` draws the trajectories, which is the fastest way to see whether a new
scenario does what you meant.

Runs are paired by construction: obstacle phase set *k* is shared across every
trigger and every planner, so a difference between two rows is the trigger and
not the draw. Keep `--samples` identical when comparing anything.

## Saving and reloading

The sweeps take 10 to 30 minutes at 200 samples. Save them.

```
python scripts/dynamic_scenario.py --samples 200 --sweep-triggers --workers 8 \
    --out-json trigger_sweep_n200_scenario_1.json
python scripts/dynamic_scenario.py --from-json trigger_sweep_n200_scenario_1.json --sweep-plot out.png
```

The committed JSON files are the evidence for the paper's tables. Regenerate
them rather than editing a number by hand.

## Watching it in grSim

Start grSim, then:

```
PYTHONPATH="<plannertest>/src" python scripts/drive_grsim.py --planner voronoi --duration 120
```

Three blue robots cross the pitch and back planning every control tick, four
yellow robots patrol the middle as moving obstacles. Swap `--planner visibility`
or `--planner prm`. `--trigger periodic --period-ms 50` reproduces the failure
where the Voronoi roadmap dithers and stops arriving.

Replans, reversals, planning time and closest approach print every two seconds.

## The closed-loop tier

Everything above is offline and headless. grSim is not a fourth experiment; it
is the validity check on the offline ones. Offline the robot is exactly on its
path every tick, which is the assumption behind the collision counts and the
whole path-stability result. grSim breaks that assumption and says whether the
ordering survives.

Two things make a grSim run worth quoting.

**Turn the idealisations off.** Out of the box grSim reports exact positions
with zero latency, which is the same perfect world model the offline runs
assume, so a run against it cannot falsify anything. `set_grsim_realism.sh`
sets both from the match log rather than from a plausible default: 23 mm
position noise, measured as frame-to-frame jitter, and 9 ms sending delay, the
median of `t_sent - t_capture` over 4000 detection frames (p95 10.2, max 19.7).
grSim ships with 3 mm and 0 ms. Restart grSim afterwards; it reads the config
once at startup.

**Use real opponents.** `--opponents log` replays a recorded Division B match
into grSim's yellow team, frame by frame, through the same reader the offline
churn measurements use. They are placed rather than driven: these are a
recorded obstacle field, not simulated agents, so teleporting reproduces the
recorded trajectories exactly where a controller chasing them would add its own
tracking error. It also avoids synthetic motion, which failed twice on this
project and produced jitter in place rather than travel.

```
bash scripts/set_grsim_realism.sh
python scripts/drive_grsim.py --planner voronoi --robots 6 --obstacles 6     --opponents log --log ~/ssl-gamelogs/MATCH.log.gz --out-json voronoi.json
```

Real time, so budget one wall-clock second per simulated second and about
twenty runs per condition rather than 200. The point is agreement with the
offline ordering, not higher confidence in the offline numbers.

Read `replans` and `direct` together. A call returning no waypoints saw a clear
line to the goal and built no roadmap; counting those as replans reported ~550
per 30 s, one per control tick, which counts sightlines rather than planning.

## One thing that will bite you

grSim does not hold a velocity command between packets. A robot commanded at a
steady 1.2 m/s for 3 s moves 22 mm if you send once per 50 ms control tick, and
618 mm at 50 Hz, against the 3600 mm the command asks for. Deliver through
`RobotCommandDispatcher`, which repeats the latest command at 100 Hz. And drain
the vision socket before reading a pose, or you get one from the previous tick
and conclude the command channel is dead. It is not; that has now happened
twice.
