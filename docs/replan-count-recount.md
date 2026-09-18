# The replan counts include failure retries

`dynamic_scenario.py` replans whenever the trigger fires **or the path is
empty**, and a failed plan returns `()`, so every failure produces another call
on the next tick. Those retries were counted in `recalculated` and their
planning time was averaged into `accumulated_ms` alongside real roadmap builds.
`RunResult` had no failure counter, so nothing in the output showed it.

The harness now records `built` (calls that returned a path) and
`accumulated_ms_built` beside the existing fields, which keep their meaning.

## Measured, n=200 paired, obstacles 1.0 m/s, geometric trigger

| scenario | planner | recalc (as published) | built | failed | ms/build |
|---|---|---|---|---|---|
| 1 | PRM | 6.49 | 3.79 | 41.6% | 7.93 |
| 1 | VisibilityGraph | 13.37 | 5.46 | 59.1% | 5.49 |
| 1 | Voronoi | 3.09 | 3.09 | 0.0% | 2.03 |
| 2 | PRM | 8.74 | 4.44 | 49.2% | 5.34 |
| 2 | VisibilityGraph | 11.15 | 5.16 | 53.8% | 2.40 |
| 2 | Voronoi | 4.20 | 4.20 | 0.0% | 1.39 |

The `recalc` column reproduces the committed table exactly, so this is the
published condition with a decomposition added, not a different experiment.

## What changes for the paper

**The replan-count advantage is much smaller than reported.** Counting only
calls that produced a path, scenario 1 gives 3.79, 5.46 and 3.09 rather than
6.49, 13.37 and 3.09. Voronoi against the visibility graph falls from 4.3 times
to 1.77; against PRM, from 2.1 times to 1.23.

**The comparison to the source study needs the same care.** Our visibility graph
reproduces their reported 13.09 closely at 13.37, but only 5.46 of those calls
return a path. Whether their 13.09 counts retries is not something we can read
off their paper, so the honest statement is that the two numbers are not known
to be measuring the same thing.

**A cleaner claim is available and it is stronger.** Voronoi failed 0 of roughly
600 calls in each scenario, where PRM failed 42 to 49 percent and the
visibility graph 54 to 59. A roadmap that always connects is a better headline
than one that replans less often, and it is not sensitive to how retries are
counted.

**Per-call cost is unaffected and still favours Voronoi**, by 2.7 and 3.9 times
in scenario 1. Separating failures actually strengthens it, because a failed
call returns in about 0.15 ms and was dragging every planner's mean toward zero,
the ones that fail most by the most.

## Clearance, and why Voronoi does not need to replan

Recorded alongside: the smallest distance from each successful plan to any
obstacle centre, at the moment it was made.

| scenario | planner | mean clearance | min |
|---|---|---|---|
| 1 | PRM | 291 mm | 210 |
| 1 | VisibilityGraph | 217 mm | 210 |
| 1 | Voronoi | 387 mm | 16 |
| 2 | PRM | 285 mm | 210 |
| 2 | VisibilityGraph | 223 mm | 210 |
| 2 | Voronoi | 326 mm | 27 |

This replicates a closed-loop observation from grSim on a full Division B field,
where Voronoi paths sat 674 mm from the nearest obstacle and were never
invalidated, while the visibility graph sat at 122 mm and was invalidated 9.33
times per run. Different absolute values, same ordering, and here with obstacle
speed as a controlled variable at n=200.

The mechanism this supports: a path with room to spare survives an obstacle
moving into its neighbourhood; a path that hugs an obstacle does not. That links
the committed finding that Voronoi carries 8.6 percent redundant length at
equal clearance to its replanning behaviour. The redundancy buys trigger
immunity.

## Open, do not write this one down yet

Voronoi's **minimum** clearance is 16 mm in scenario 1 and 27 mm in scenario 2,
below the 210 mm inflation radius, where the other two never go under 210. That
is the single worst plan out of roughly 600 per scenario.

Three candidate explanations, untested:

1. An outlier. The distribution, not the extremum, settles this.
2. An artefact of rooting the path at the robot's current position: if an
   obstacle has moved onto the robot, the first leg starts close to it.
3. Voronoi returning a path in situations where PRM and the visibility graph
   fail, which is consistent with its 0 percent failure rate. That would
   reframe "never fails" as "always returns something, sometimes tight".

Voronoi still records the fewest collisions in both scenarios, 0.61 and 0.85, so
there is no evidence of it being less safe overall. Measure the clearance
distribution before claiming either way.
