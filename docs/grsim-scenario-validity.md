# Six faults that made the grSim comparisons unreadable

`results/` is not version controlled, so the record of why certain result
directories must not be quoted lives here. Copies sit next to the data in
`results/grsim/BLOCKED-ENDPOINTS.md` and `results/grsim/paired/README.md`.

The six faults below are independent, and each was found by checking the
blast radius of the one before it. An earlier revision of the results README
collapsed the first two into one and retracted the second; that retraction was
wrong and has been withdrawn.

Two of the three changed results without changing any output the summary
printed. That is the common thread: each is now either refused at startup or
printed in the summary table.

## Fault 1: the goal was inside an obstacle

`drive_grsim.py` traverses between (-3200, 0) and (3200, 0). Those are the two
goal mouths, and a recorded match parks its goalkeepers there.

Replaying the blue team of `2024-07-19_TurtleRabbit-vs-NAMeC` from t=180 s, a
keeper sits within 210 mm of (-3200, 0) in **73.3 percent of frames**, minimum
clearance 1 mm. 210 mm is the planners' inflation radius: 90 mm obstacle + 90 mm
robot + 30 mm clearance. A goal inside an inflated obstacle has no path, and all
three planners correctly returned failure.

A failed plan empties `robot.path`, and `not robot.path` re-triggers a plan on
the next tick, so one blocked goal produces a failure every tick until the
keeper moves.

| set | failed calls | laps |
|---|---|---|
| `paired-blue-180/` | 96 percent of all calls | 2 |
| `paired-blue-90-fullbuild-ABORTED/` | 247 of 253 in one run, 240 consecutive | 2 |
| `paired/` (yellow, endpoints clear) | 0 | 5 |

The damage is not confined to the failure count. `mean_plan_ms` divides total
planning time by every call including failures, and a failed call returns in
about 0.15 ms, so the reported timing fell as the scenario got worse. Recomputed
over successful builds only, from the per-call `planning_ms` already stored in
`diagnostics.plan_events`:

| planner | ms/call, as reported | ms/build, successful calls only |
|---|---|---|
| voronoi | 1.29 | 0.59 |
| visibility | 0.07 | 3.64 |
| prm | 0.21 | 14.69 |

The reported column ordered PRM and the visibility graph fastest and Voronoi
slowest, contradicting the offline tables. Corrected, Voronoi is fastest per
build and the offline ordering holds. PRM differs by a factor of 70 between the
two columns.

**Fixed by** `scripts/check_endpoint_clearance.py`, which scores candidate
endpoints against the clip, and a startup check in `drive_grsim.py` that refuses
a blocked endpoint unless `--allow-blocked-endpoints` is passed. The replacement
runs use `--lane-centre-mm 1500`: the same 6400 mm traverse, through a corridor
the clip leaves clear in every frame with 1260 mm of worst-case clearance.

**Why it went unnoticed for 45 runs:** the summary table printed replans, direct
calls, plan time and closest approach, but not failures. It now prints a
`failed` column, a `ms/build` column, and warns above a 5 percent failure
share.

## Fault 2: the source study's 90 mm trigger is nearly inert

This one is real and survives on a valid scenario, which is why the retraction
was withdrawn.

The driver's rule, `drive_grsim.py:166`, is

    _point_to_segment(o.pos_mm, a, b) <= o.radius_mm + threshold_mm

with `threshold_mm = centre_threshold - 90` and `o.radius_mm = 90`, so it
reduces to `dist_to_path <= centre_threshold`. The offline rule,
`dynamic_scenario.py:209`, is `point_to_path_mm(o.pos_mm, path) <= 90`.

So `--trigger-centre-threshold-mm 90` reproduces the offline predicate exactly,
and 180 mm is a different, looser rule. That is the reason to prefer 90 mm for
any run compared against the offline tables. It is not a matter of convention.

On the corrected scenario the rule fires 0 to 2 times per 30 s run. Plans per
run equal legs travelled plus zero or one, and `replans_compared` is 0 in most
runs. The two cases are distinguishable because a plan is issued only when the
path is empty or the trigger is due, and an empty path has no segments for the
rule to test, so a previous path exists exactly when the trigger fired.

Checked across 30 runs, whether the trigger fired agrees with "an obstacle came
within 90 mm of the robot" in 27. All 3 exceptions fired at 91 to 128 mm closest
approach, the expected direction: the rule measures obstacle-to-path distance
and the path runs ahead of the robot.

The mechanism is that a freshly planned path already carries 210 mm of
clearance, so an obstacle must intrude 120 mm into the corridor before a 90 mm
rule notices. This is an argument for the predictive and near-field triggers,
and it belongs with RQ3.

**Settled.** `paired-blue-lane1500-180-fullbuild/`, 45 runs on the corrected
scenario, gives mid-path replans per run as `replans_compared`:

| trigger | voronoi | visibility | prm |
|---|---|---|---|
| 90 mm | 0.73 | 0.00 | 0.00 |
| 180 mm | 3.13 | 2.00 | 0.00 |
| 180 mm, reversals over 15 runs | 12 | 0 | 0 |

A 180 mm trigger does fire, 2 to 3 times per 30 s run for the Voronoi and
visibility roadmaps. A 90 mm one is effectively inert. PRM never replans
mid-path in either condition.

Reversals appear only under the looser trigger and only for Voronoi, which is
the closed-loop counterpart of the offline oscillation result, on recorded SSL
motion rather than synthetic patrol loops. It is a small count on one clip and
is consistent with that finding rather than independent confirmation of it.

**Use `replans_compared`, not "replans + direct - legs".** The latter also
counts failure recovery, since a failed plan empties the path and an empty path
replans on the next tick. It gave the visibility graph 1.00 apparent mid-path
replans per run at 90 mm where `replans_compared` is 0.00.

## Fault 3: grSim's own robots joined the obstacle set

Found when a WSL upgrade forced a fresh grSim in the middle of the 180 mm batch.

`obstacles_for` counted every robot on the pitch, and `wait_for_robots` checked
only for *at least* the requested number. grSim starts with a full team of 6
blue robots; the driver placed blue 0 and left blue 1 to 5 sitting where the
simulator put them, where they were counted as obstacles. With `--obstacles 6`
that is 5 + 6 = 11.

The same command line therefore produced a 6-obstacle run before the restart and
an 11-obstacle run after it, with nothing in the output saying so.

| planner | ms/build, 6 obstacles | ms/build, 11 obstacles |
|---|---|---|
| visibility | 4.1 to 4.8 | 10.3 to 13.5 |
| prm | 9.1 to 15.5 | 9.0 to 18.9 |
| voronoi | 1.9 to 2.7 | 1.3 to 2.6 |

Only the visibility graph moved, because only its cost grows with the obstacle
count. Had this landed in a set where the visibility graph was not the planner
under scrutiny, it would have read as run-to-run noise.

**Scope.** Every result directory was checked for the obstacle count recorded in
each plan event. `paired/`, `paired-blue-180/` and
`paired-blue-lane1500-fullbuild/` are uniformly 6, so the 90 mm comparison is
unaffected. Only `paired-blue-lane1500-180-fullbuild/` was contaminated; its 15
eleven-obstacle files were deleted and the 14 clean ones kept.

**Fixed by** three changes in `drive_grsim.py`. `place_teams` parks every robot
id it does not use. `obstacles_for` and the startup check count only robots
inside the field plus a 500 mm margin, because parking alone does not work:
grSim keeps reporting a robot at y = -7000. The run then refuses unless the
in-play counts match exactly, with `--allow-extra-robots` to override, and
`observed_blue` and `observed_yellow` are recorded in each result file so a set
can be audited afterwards.

## Fault 4: timings do not travel between simulator sessions

A grSim restart forced by a WSL upgrade landed mid-batch. Same command line,
same 6 in-play obstacles, but every planner ran 1.5 to 1.6 times faster
afterwards, with disjoint per-planner ranges:

| planner | before restart | after restart | ratio |
|---|---|---|---|
| prm | 11.30 [9.10, 15.47] | 7.01 [5.88, 7.68] | 1.61 |
| visibility | 4.34 [4.08, 4.81] | 2.84 [2.73, 3.05] | 1.53 |
| voronoi | 2.17 [1.86, 2.68] | 1.32 [1.22, 1.59] | 1.64 |

The simulator had been up for a 45-run batch before the restart and was fresh
after it. Within one session the paired design absorbs this, because planners
alternate run to run. Across sessions it does not, and nothing in the result
file recorded which session a run belonged to.

The ratio is near-uniform, so planner ordering survives and the 90 mm
comparison's ratios hold. Absolute milliseconds do not travel between sessions.

**Fixed by** recording `grsim_pid` and `grsim_started` in `run_provenance`, so a
set can be checked for session boundaries instead of assumed uniform.

## Fault 5: the rule choosing which robots to replay preferred parked ones

`LogOpponents` took the `count` most-consistently-tracked robots. Coverage was
meant to avoid robots that vanish and get held at a stale position. It also
selects against motion: a robot standing still is easy to track and one crossing
traffic is occluded. Measured over 30 s windows across this log, coverage
correlates with speed at r = -0.94, -0.98, -0.95, -0.99 and similar.

So the replayed obstacle field was, by construction, the slowest robots
available. Spotted from the figures, as a scene where everything sits still and
one robot moves.

**Fixed by** selecting on span, the distance between a robot's extreme
positions, with coverage kept only as a gate at 50 percent. Span rather than
path length: a robot oscillating on the spot accumulates path length without
going anywhere, and at these sampling intervals vision noise alone contributes
about 0.13 m/s to any differentiated speed.

## Fault 6: a requested obstacle that the clip could not supply

`--obstacles 6` with a clip tracking only 5 robots gave 5 replayed obstacles and
one grSim robot that kept its startup position for the whole run while still
being counted as an obstacle. The driver printed the five it had chosen and said
nothing about the sixth.

This affects `paired-blue-lane1500-fullbuild/` and
`paired-blue-lane1500-180-fullbuild/`: both are a 5-replayed-plus-1-parked
scene, not the 6 replayed their label implies. Every planner saw the identical
scene, so the comparison between planners holds; the scene description does not.

**Fixed by** refusing when the clip tracks fewer robots than `--obstacles`
requests, naming the number to pass instead.

## A note on measuring motion in these logs

Differentiating recorded positions at the log's native 70 Hz turns 23 mm of
vision noise into about 1.6 m/s of apparent speed, which swamps real motion. An
early pass at choosing a clip did exactly that and reported every window as a
2 m/s scramble. Sampled at 4 Hz the same windows run 0.05 to 0.56 m/s.

The 180 to 210 s window is among the most active in this recording, and the
recording is quiet by competitive standards. `scripts/scan_clip_motion.py`
scored all 173 thirty-second windows of the 2611 s log (4 Hz sampling, the
driver's own reader and coverage gate) on 20 September 2026: the 180 s window
ranks 15th, with two robots that travel more than 1 m (B3 8.4 m at 0.54 m/s,
B5 7.8 m at 0.34 m/s) and nine that hold position; the 195 s window has four
(B3, B5 and two blue robots that move 1.0 m). Every window that scored higher
does so through an identity jump, one (team, id) tracked on two physical
robots: per-robot spans of 63 to 160 m and a "speed" of 5.7 m/s, both
impossible on a 9 m field, at 60, 1095, 1440 and 1530 s. Replayed by identity,
those windows would teleport an obstacle. The yellow team (NAMeC) moves less
than 0.3 m in any window of the log. A livelier scene needs a different
recording, not a different offset in this one.

## Telling a blocked goal from a corridor that closes

Both raise the failure share, and the remedies are opposite: an
unreachable goal invalidates the run, a corridor that closes and reopens is a
result. The longest unbroken run of failed calls separates them, and the summary
now prints it.

| set | worst failure share | longest failure streak | verdict |
|---|---|---|---|
| `paired-blue-180/` | 96 percent | 275 | goal unreachable, invalid |
| `paired-blue-lane1500-fullbuild/` | 74 percent | 17 | corridor closing, a result |

The failure share alone does not distinguish them, which is why the check is on
the streak.

## The corrected comparison

45 runs, 15 per planner, `--lane-centre-mm 1500 --force-full-build
--trigger-centre-threshold-mm 90`. Median [min, max].

Read with fault 6: this is a 5-replayed-plus-1-parked scene, not the 6 replayed
obstacles the command line says. All three planners saw the identical scene, so
the comparison holds and the scene description does not. Superseded for realism
by `div-b-11obs-90/`, a full Division B field of 6 opponents and 5 teammates.

| planner | laps | replans | failed | ms/build | ms/build ex.1st | closest mm |
|---|---|---|---|---|---|---|
| voronoi | 3 [3, 4] | 6 [4, 8] | 0 [0, 0] | 3.23 [2.26, 5.04] | 3.30 [2.24, 5.47] | 70 |
| visibility | 5 [4, 5] | 7 [6, 7] | 3 [2, 17] | 6.10 [4.03, 8.54] | 5.99 [3.92, 7.40] | 75 |
| prm | 4 [4, 5] | 5 [5, 7] | 0 [0, 4] | 18.76 [8.10, 27.23] | 9.05 [4.25, 16.74] | 105 |

Read the `ex.1st` column for PRM: fault 7 below explains why the plain
`ms/build` doubles it.

The offline ordering survives the move into closed loop, on a different
obstacle field and with vision noise and latency present:

| planner | offline ms/replan (`tab:predictive`) | grSim ms/build ex.1st |
|---|---|---|
| voronoi | 14.09 | 3.30 |
| visibility | 15.67 | 5.99 |
| prm | 39.33 | 9.05 |

PRM is 2.8 times Voronoi offline and 2.7 times here; on the 11-obstacle
Division B sets below it is 1.4 to 1.6 times. The earlier version of this
table read 18.76 for PRM and "5.8 times", which was the first-call artefact.

Failures concentrate in the visibility graph, which holds 5 of the 6
highest-failure runs, each coinciding with an obstacle 35 to 82 mm away.

Voronoi completes 3 laps to the visibility graph's 5 while planning at half the
cost per call, consistent with the committed offline finding that its path
carries 8.6 percent redundant length at identical clearance.

Standing caveats: closest approach is from vision positions and is not a
collision count; replayed obstacles do not react to the controlled robot; this
is one 30 s clip and compares implementations rather than establishing which
planner is best; controller error and planner error are not yet separated.

## Fault 7: PRM's first call paid numpy's RNG initialisation

Every driver run is a fresh process. numpy's `default_rng` costs 39.4 ms on
its first use in a bare interpreter, 24.8 ms once the planners are imported,
and 0.02 ms after that (measured 19 September 2026). PRM is the only backend
that draws random numbers, so its first roadmap in every run carried that
cost. In the per-call diagnostics of all four Division B sets (Windows and
WSL, 90 and 180 mm) PRM's first successful build took 33 to 43 ms at the
median and every later build 4.5 to 5.5 ms; the slowest call was the first in
60 of 60 runs. Voronoi and the visibility graph, which draw nothing, showed no
first-call effect (first 2.9 against rest 3.1 ms; 14.2 against 13.8 ms).

With five or six builds per run, one 35 ms call doubles the mean:

| set | PRM ms/build | PRM ms/build ex.1st |
|---|---|---|
| div-b-11obs-y750-90 (WSL) | 11.71 [9.10, 17.14] | 5.76 [4.36, 7.57] |
| div-b-11obs-y750-180 (WSL) | 10.04 [8.34, 16.18] | 6.19 [4.92, 8.64] |
| win-div-b-11obs-y750-90 | 10.38 [9.15, 67.26] | 5.09 [4.50, 8.29] |
| win-div-b-11obs-y750-180 | 9.30 [8.15, 12.83] | 4.65 [4.37, 6.48] |
| paired-blue-lane1500-fullbuild | 18.76 [8.10, 27.23] | 9.05 [4.25, 16.74] |

**Fixed by** `drive_grsim.py` calling `default_rng` once at import, before any
timed call; a PRM run made afterwards had builds of 13.1, 11.4, 11.1, 14.2,
11.4 and 18.6 ms with no first-call step (absolute values inflated by an
offline sweep running at the same time). `summarise_overnight.py` prints
`ms/build ex.1st` so the sets recorded before the fix read correctly. The
per-build ordering with the artefact removed is Voronoi, PRM, visibility at
about 1 : 1.6 : 4.6 on the Division B sets.

The offline harness is not affected in the same way: `dynamic_scenario.py`
runs hundreds of samples in one process, so the initialisation lands on one
call in thousands.

## The same comparison on native Windows

`results/grsim/win-div-b-11obs-y750-{90,180}/` repeat the two Division B
sets on the native Windows grSim with Windows Python 3.13, 45 runs each on
one simulator process, made by `scripts/batch_grsim.py`. Crossings, replans
and failures match the WSL medians in every cell (WSL visibility replans at
180 mm are 18 [13, 21] against 19 [13, 27]); per-build times are 0.78 to 0.96
of the WSL values across the nine planner-by-threshold cells; closest
approach agrees within the run-to-run spread. The README in the 90 mm
directory has the full tables. This says the WSL result reproduces on a
different host and interpreter; it does not add evidence about which planner
is best.
