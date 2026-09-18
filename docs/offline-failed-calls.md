# Roughly half of the baselines' offline planning calls return no path

Found while checking whether the grSim blocked-endpoint fault had an offline
counterpart. It does not, but something larger turned up in the same check.

## What was measured

`scripts/dynamic_scenario.py` replans whenever the trigger fires **or the path
is empty**, at `dynamic_scenario.py:436`:

    if not path or trigger(ctx):

A failed plan returns `()` at line 401, so every failure produces another call
on the next tick, exactly as the grSim driver did. `accumulated_ms` is
incremented at line 400 for every call, successes and failures alike, and
`RunResult` has no failure counter at all.

Wrapping `PLANNER_RUNNERS` to count outcomes, without modifying the harness.
Geometric trigger, obstacles at 1.0 m/s, robot 1.5 m/s, n=60 phase draws:

| scenario | planner | arrived | calls/run | failed | ms/call | ms/build |
|---|---|---|---|---|---|---|
| 1 (7 obs) | PRM | 60/60 | 6.80 | 171 (41.9%) | 3.70 | 6.36 |
| 1 | VisibilityGraph | 60/60 | 14.18 | 499 (58.6%) | 1.82 | 4.39 |
| 1 | Voronoi | 60/60 | 2.93 | 0 (0.0%) | 1.70 | 1.70 |
| 2 (5 obs) | PRM | 60/60 | 8.38 | 234 (46.5%) | 2.48 | 4.61 |
| 2 | VisibilityGraph | 60/60 | 10.98 | 365 (55.4%) | 0.88 | 1.97 |
| 2 | Voronoi | 60/60 | 3.88 | 0 (0.0%) | 1.05 | 1.05 |

Every failure is `success=False`, not a successful call with an empty path.

The runners force `skip_direct_path=True` (`dynamic_scenario.py:67`), so these
are full roadmap builds, the same thing `--force-full-build` now does in grSim.

## Why this is the same condition as the published table

Calls per run reproduce the committed replan counts closely: 6.80 against 6.49
for PRM, 14.18 against 13.37 for the visibility graph, 2.93 against 3.09 for
Voronoi. Whatever else differs, the scenario and trigger are the ones the table
reports.

## What follows

**The Replans column counts failure retries.** Of the visibility graph's 14.18
calls per run in scenario 1, about 8.3 are failures being retried; roughly 5.9
produce a path. Voronoi never fails. So part of the headline gap, Voronoi's 3.09
replans against the source study's 13.09, is a difference in how often a roadmap
connects at all, not in how often the robot needs a new plan. That is still a
result in Voronoi's favour, but it is a different claim from the one the table
currently supports, and the paper should say which it means.

**ms/replan averages failure returns into build cost.** Separating them moves
the visibility graph from 0.88 to 1.97 ms in scenario 2, a factor of 2.2, and
PRM from 2.48 to 4.61. Voronoi is unaffected because it never fails. The
direction of the bias therefore differs per planner, which is the case where an
average over mixed outcomes is least safe.

**Arrival is not affected.** All 60/60 runs arrive in every cell. The failures
are transient: the robot holds position and the next tick succeeds.

## Not established

- **The absolute times here are about eight times lower than the published
  table** (Voronoi 1.70 ms against 14.09). The published sweeps ran with
  `--workers 8`, so contention between eight worker processes is a candidate
  explanation, but I have not tested it. Until that is resolved, treat the
  columns above as relative, not as replacements for the published values.
- One trigger, one obstacle speed, n=60. The published tables use n=200 across
  several triggers.
- Whether the failure rate is a defect or a property. PRM uses 40 milestones
  (`dynamic_scenario.py:71`), and more samples would likely connect more often;
  the visibility graph's failures are consistent with the zero-width corridor
  behaviour already documented for hexagonal inflation. Both are legitimate
  characteristics of those implementations. The reporting is the issue, not the
  planners.

## Suggested next step

Add a failure counter to `RunResult` and report replans and ms over successful
builds separately, as `summarise_overnight.py` now does for grSim. That changes
published numbers, so it is a decision rather than a fix, and it needs the
`--workers` question settled first.
