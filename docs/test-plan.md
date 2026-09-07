# Test plan: replanning in the SSL

Derived from the four papers in [papers/](papers/README.md), not invented. Each
phase says what it measures, why, whose precedent it follows, and what result
would kill it.

The framing follows the reviewer's note: this work is not a new path-planning
algorithm, it is **deciding when replanning is necessary** in an environment
that is fully known but highly dynamic. That is a different problem from
planning in unknown terrain (D\* Lite's motivating case) and from dexterous-arm
motion planning.

## Phase 0 — the measurement that decides whether the paper is viable

**Run this before writing any planner code.**

Lim et al. change "a randomly chosen 10% of the edges" between consecutive
searches, and their whole result rests on the change being small enough that
reusing the previous search pays. Nobody has measured the SSL equivalent.

**Question.** Between consecutive vision frames at 60 Hz, what fraction of the
visibility graph actually changes?

**Measure**, per frame, against the live grSim loop:

| Quantity | Why |
|---|---|
| fraction of graph edges that flip validity | the direct analogue of Lim's 10% |
| number of obstacles that moved more than the 90 mm invalidation threshold | da Silva Costa & Tonidandel's trigger, so this counts their replans |
| whether the *current path* was invalidated | the replan rate a robot actually experiences |

**Decision rule.** If a small fraction of edges changes per frame, incremental
repair should beat from-scratch and the paper has its motivation in one plot.
If most of the graph churns every frame, the premise fails, incremental search
buys nothing, and we should say so and pivot to trigger policy alone. **Either
outcome is publishable; not knowing is not.**

## Phase 1 — static baseline, comparable to prior work

Adopt da Silva Costa & Tonidandel's static metric set verbatim so the numbers
are comparable, then add the operation counts that explain them.

| Metric | Source |
|---|---|
| computational time, ms | da Silva Costa & Tonidandel |
| path length, m | " |
| path safety (sum of perpendicular obstacle-to-path distances) | " |
| **number of edge evaluations** | Lim et al. |
| **number of vertex expansions** | Lim et al. |

The counts are what turn "Voronoi is faster" into "Voronoi is faster *because*".
Our harness records wall-clock only today.

**Instances.** Both, because they answer different questions:

- their four hand-designed scenarios (worst case per algorithm, mixed, game
  stoppage) as named case studies;
- our randomised paired sweep (`scripts/benchmark_planner_sweep.py`) for the
  statistics.

**Weight the sweep toward sparse.** They report that in real matches robots are
"practically alone or with a second robot within a radius of 500 to 1500 mm".
That independently explains why our success rate saturates at 200/200 — SSL is
not cluttered, so completeness never discriminates. Extending the density sweep
upward measures a situation that does not occur.

**Reporting**, per Moll et al.: distributions rather than means (box plots, and
the empirical CDF), with **n printed for every cell**. Their warning is
concrete: if a planner fails 99 of 100 runs, its mean path length comes from a
single sample.

## Phase 2 — the budget question

The shipping constraint is the **50 ms control tick**
(`ui/execution/page.py:380`), not the 16 ms figure quoted in
`visibility_graph.py:33`, which is the vision frame budget and a different
thing. `ResearchRuntime.plan()` loops robots serially, so a team shares one
tick.

**Plot.** Empirical CDF of planning time per call, with vertical lines at
50 ms and at 50/N ms for a team of N. Read off P(solved within budget)
directly. This replaces the current `fits/tick` integer, which throws away the
distribution.

**Precedent.** da Silva Costa & Tonidandel already do budget-constrained
evaluation: their Table 12 re-runs both planners under a strict **5 ms** limit
and reports *rate of failure* and *average distance from target*. So the
framing is citable rather than invented. Report the same two quantities.

## Phase 3 — replanning triggers, the actual contribution

The gap, in the prior work's own words:

> "even though a new path would only need a minor change in the invalid path,
> the whole algorithm needs to run again."

**Baseline trigger.** Theirs: a path is invalid whenever an obstacle crosses
it, threshold 90 mm (the robot radius). Purely geometric, no prediction.

**Triggers to compare against it:**

1. **Periodic** — replan every k control ticks regardless. The trivial control;
   without it we cannot claim any trigger is smart.
2. **Predictive horizon** — ignore a predicted crossing beyond T seconds. Prior
   art exists in the league: they cite an algorithm using a 3 s horizon with
   paths regenerated in a 10 ms window. The SDK already has `predict_motion`
   for the obstacle-motion half of this.
3. **Lookahead / commitment depth** — Lim et al.'s `l`, how far the search tree
   is repaired before edges are evaluated, swept at `1, 2, 3, 4, inf`. Their
   stated trade: higher lookahead delays edge evaluation but causes more vertex
   expansions. This is the same shape as "how much do you commit to before
   re-checking the world", with prior art behind it.

**Metrics**, from their dynamic perspective, one sample = robot leaves start and
arrives:

| Metric | Definition |
|---|---|
| accumulated computational time | sum of cost of all paths generated until the goal, ms |
| paths recalculated | times a previous path was invalid and replanned |
| navigation time | start to goal, s |
| number of collisions | contacts with any obstacle |

All four already exist in `docs/results_metrics.md` Format B, measured by
`ExecutionController` against grSim. This phase is mostly wiring, not new
measurement code.

**Their benchmark to beat** (Table 9, scenario 1): DVG+A\* 13.09 recalculations
mean, RRT 34.08.

## Phase 4 — incremental replanning, only where the premise holds

Lim et al.'s argument requires **edge evaluation to cost more than vertex
expansion**. That is a property of the planner, not of the domain:

| Planner | Edge evaluation is | Incremental/lazy applies? |
|---|---|---|
| VisibilityGraph | segment vs inflated polygon, O(n²) of them | **yes, strongly** |
| PRM | collision check between milestones | **yes** |
| Voronoi | cost is building the tessellation, not testing edges | **not obviously** |

So implement LPA\*-style incremental repair for the visibility graph, compare
against its own from-scratch baseline, and state plainly that the third planner
is outside the premise. That split is itself a result: the incremental-lazy
literature targets two of our three planners and largely misses the third.

## Known-invalid metric, and an opportunity

Our `min clear mm`, and their path safety, are **static** measures. Their own
conclusion:

> "the path safety parameter, defined initially for the static experiments,
> doesn't measure at all how safe a path actually is when there are fast, and
> unpredictable, moving obstacles ... another metric should be implemented ...
> would need some particular research."

Either restrict clearance to Phase 1 and drop it from the dynamic phases, or
propose the dynamic safety metric they say is missing. The second is a
contribution in its own right.

Related: they conclude a path planner alone cannot avoid collisions and a
separate obstacle-avoidance layer is required, conceding that **all their
experiments would need redoing** once one exists. Worth deciding early whether
we are testing planners alone or a planner plus avoidance layer, because it
determines whether our collision counts are comparable to theirs.

## Settings that must be fixed before any number is generated

1. **grSim Robots Count is 11**, inherited from the Division A default, so
   scenes carry 22 obstacles instead of Division B's 12.
2. **`predict_motion` defaults `False`** — every current number assumes static
   obstacles. Ablate it or caveat it.
3. **`parallel_planning` defaults `True`** but ADR 0005 measured the thread pool
   as sometimes slower than serial. Report what we ship.
4. **The SDK ignores grSim's geometry packet** — planning scenes come back
   9000x6000 from a hardcoded default regardless of what vision reports. Correct
   in Division B by coincidence.
5. **Planner tunables** (`prm_num_samples=20`, `k=6`,
   `voronoi_max_density_nodes=140`) are three arbitrary operating points. Per
   Moll et al., either tune each to equal path quality and compare time, or
   equal time and compare quality. Pick one and say which.
6. **Voronoi's worst-case clearance falls below its own 120 mm inflation** (97 mm
   at 9 obstacles, 104 mm at 15, n=200). Bug or accepted behaviour?
