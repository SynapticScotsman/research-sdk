# Shared D* Lite across the three planners: what it costs and what it buys

All three backends now search through `src/research_sdk/planners/search.py`, so
the search is genuinely held constant and the comparison is about the mapping.
This file records what that measured, so nobody has to re-derive it.

Reproduce with `scripts/measure_search_reuse.py`. Run under the project venv
with `PYTHONPATH` pointing at this worktree's `src`.

## Correctness first

Every run replays the identical frame sequence twice, once with D* Lite and
once with `networkx.dijkstra_path`, and compares path length frame by frame.

**2418 paired comparisons, 2418 agreements, maximum difference 0.0000 mm.**
The incremental search returns the same optimal path as the exhaustive one on
all three roadmap types. This is the claim that has to hold before any timing
number means anything.

## Positive control: obstacles frozen, robot moving

400 frames, quantum 5 mm, 5 alternating A/B repeats, medians.

| planner          | searches | reuse | nodes | churn | exp/reuse | exp/fresh | D* ms | Dijkstra ms |
|------------------|---------:|------:|------:|------:|----------:|----------:|------:|------------:|
| visibility graph |      399 | 99.7% |  50.0 |  4.0% |       1.0 |       5.0 | 1.304 |       1.600 |
| PRM              |      400 | 99.8% |  20.0 | 10.0% |       1.0 |      10.0 | 0.335 |       0.327 |
| Voronoi          |      400 | 99.8% |  36.0 |  5.5% |       1.0 |      20.0 | 0.348 |       0.311 |

`churn` is the share of the vertex set replaced per call. Here it is only the
start vertex being spliced in and the previous one dropped: 2 nodes out of 50,
20 and 36 respectively. A repair expands **1** vertex where a full search
expands 5, 10 or 20.

This control exists because the first version of the wiring reported **0%
reuse on this scene**. `DStarLite.plan` reinitialised whenever the vertex set
differed, and the robot's own position is a vertex of the roadmap in all three
planners, so the vertex set differs on every single frame. The fix was to treat
an added vertex as one entering locally consistent and a removed vertex as its
edges going to infinity, both of which reduce to the edge-cost changes Koenig
and Likhachev's `UpdateVertex` already handles. Only a changed goal now forces
a rebuild.

## Real match: 2024-07-19 TurtleRabbit vs NAMeC, Division B

421 frames from the recorded game log, replayed through `VisionWorldPipeline`.
Same 5 alternating repeats.

| planner          | searches | reuse | nodes |  churn | exp/reuse | exp/fresh | D* ms | Dijkstra ms | ratio |
|------------------|---------:|------:|------:|-------:|----------:|----------:|------:|------------:|------:|
| visibility graph |      404 | 99.8% |  66.0 | 192.5% |       2.2 |       4.0 | 2.226 |       1.135 | 1.96x |
| PRM              |      406 | 99.8% |  18.5 |  14.7% |       1.3 |       6.0 | 0.479 |       0.371 | 1.29x |
| Voronoi          |      376 | 91.5% |  41.3 | 116.0% |       7.7 |      10.4 | 0.754 |       0.446 | 1.69x |

**D* Lite is 1.29x to 1.96x slower than Dijkstra on real match motion.**

Churn says why. The visibility graph replaces 192.5% of its vertex set per
frame: every inflated obstacle corner moves, so the previous search tree
describes a graph that no longer exists and the repair has to diff all of it.
Voronoi is at 116.0% for the same reason. Only PRM's roadmap survives (14.7%),
because its milestones are drawn from a fixed seed and so do not move with the
obstacles at all, and even there the graph is 18.5 nodes: the bookkeeping costs
more than the 18-node search it replaces.

## The ceiling this was always working against

Independently measured on an eight-obstacle scene, roadmap construction is
**98.0%** of a visibility-graph call (29.94 ms of 30.54 ms) and **92.8%** of a
Voronoi call (7.67 ms of 8.26 ms). Search is the remaining 2.0% and 7.2%. Even
a search made free could not move a planning call by more than that.

## What this supports and what it does not

Supported: *the three planners share one search, so any difference between them
is a difference in mapping.* That is now literally true and testable.

Supported: *incremental search is applicable in this domain but does not pay,
and the reason is measurable rather than rhetorical.* Roadmap churn is
116-193% per frame for the two geometric planners.

Not supported, do not claim: any speedup from D*. The measurement says the
opposite, on real data, five repeats, paired.

## Reproducing

```
python scripts/measure_search_reuse.py --selftest
python scripts/measure_search_reuse.py --source static --frames 400 --repeats 5
python scripts/measure_search_reuse.py --source drift --frames 100 --drift-mm 20
python scripts/measure_search_reuse.py --source log --log MATCH.log.gz --seconds 6 --repeats 5
```

The `drift` source sweeps obstacle motion per frame against the 5 mm quantum in
`search.py` and is the cheapest way to see churn cross from 4% to 190%.

## Postscript: the fixed site lattice, measured

The paper proposes a Voronoi roadmap on a fixed backbone of virtual sites as
the one structure incremental repair could target. The planner ships
`placement_mode="density_grid"`, whose sites move with the obstacles, and every
number above is that configuration. Exposing `placement_mode` and replaying the
same 421 match frames under `"grid"` gives:

| placement_mode | nodes | churn | reached D* | D* ms | Dijkstra ms |
|---|---:|---:|---:|---:|---:|
| `density_grid` | 41.3 | 116.0% | 376 of 421 | 0.274 | 0.143 |
| `grid` | 957.3 | n/a | **0 of 421** | 3.143 | 3.398 |

Two separate problems, either of which rules it out as a drop-in swap.

**Build cost.** At the default spacing of twice the clearance radius, 240 mm,
the lattice covers a 9 x 6 m pitch with 966 nodes and 1841 edges and takes
4478 ms to build, against 34 nodes, 53 edges and 2.2 ms for density placement.
That is 2000 times the cost, against a 50 ms control tick. Sweeping the spacing
(`scripts/probe_voronoi_sites.py`) finds it affordable only from about 900 mm:
116 nodes at 27.5 ms, or 83 nodes at 11.3 ms for 1200 mm.

**Vertex density against identity tolerance.** The incremental search identifies
a vertex by its position quantised to 5 mm. At 957 vertices the lattice puts
pairs of them inside the same 5 mm cell on 410 of 421 frames, so the search
refused to claim reuse and fell back to a full rebuild every time. A finer
quantum would reduce that, so this is a statement about the roadmap's density
rather than a hard failure, but it says the fixed lattice is far denser than the
problem needs: 957 vertices where 34 sufficed.

## Does a coarse lattice still find the gaps?

Measured, with `scripts/measure_gap_resolution.py`. A wall of robots spans the
pitch with one gap in it, so a route across either goes through that gap or does
not exist. Sweep the gap width, and sweep where the gap sits, because a lattice
has a period and one gap position measures one alignment. Ground truth is
geometric: a gap admits a robot exactly when its two flanking robots are 420 mm
apart or more.

14 widths x 6 positions per roadmap, 84 passable trials each.

| roadmap | solved | misses | faults | narrowest solved everywhere |
|---|---:|---:|---:|---:|
| visibility graph | 78/84 | 6 | 0 | 430 mm |
| `density_grid` | 84/84 | 0 | 0 | 420 mm |
| `grid` @ 1200 mm | 84/84 | 0 | 0 | 420 mm |
| `grid` @ 900 mm | 84/84 | 0 | 0 | 420 mm |
| `grid` @ 600 mm | 68/84 | 16 | 0 | 780 mm |

**A 1200 mm lattice resolves 420 mm gaps.** Every passable gap, every position,
down to the exact geometric limit, and nothing returned through a gap too narrow.
Clearance came out at exactly half the gap width at every width tested, which is
the Voronoi construction doing what it claims: running down the centre line.

**Coarse is better here, not merely cheaper.** The 600 mm lattice missed 16 of 84,
and missed only at gap positions that are not multiples of 600 mm: positions 0
and 600 solved, positions 200, 400, 800 and 1000 failed. That is aliasing. It
also puts 10 to 14 roadmap vertices within 700 mm of the gap against 4 to 6 for
the 1200 mm lattice, so the diagram near the gap is being shaped by virtual sites
instead of by the two robots forming it. Consistent with the aliasing; not
established as its cause.

**The visibility graph refuses 420 mm and solves everything from 430 mm.** At
exactly 420 the corridor has zero width. Voronoi routes through it with exactly
210.0 mm of clearance, which is legal and has no margin whatever. Against the
23 mm of camera noise measured on a real match, that is not a route to take. The
visibility graph's refusal is the safer behaviour, and the earlier reading of
"first solid 480 mm" was an artefact of sweeping in 60 mm steps.

So the fixed backbone is viable at 1200 mm on both counts, cost and resolution.

## Does the fixed lattice survive obstacle motion?

This is the property the whole incremental-repair argument rests on. Same 421
match frames, three site placements.

| placement | nodes | churn | vertices changed per frame | D* ms | rebuild ms | ratio |
|---|---:|---:|---:|---:|---:|---:|
| `density_grid` | 41.3 | 116.0% | **47.9** | 0.249 | 0.135 | 1.84x |
| `grid` @ 1200 mm | 83.9 | 73.9% | **62.0** | 0.533 | 0.265 | 2.01x |
| `grid` @ 900 mm | 118.5 | 54.2% | **64.2** | 0.627 | 0.375 | 1.67x |

**The churn ratio falls exactly as predicted, and it means nothing.** Churn is
`(added + removed) / total`. Going from density placement to a 900 mm lattice
halves it, 116% to 54%, which reads as the roadmap becoming twice as stable. The
count of vertices that actually change per frame goes the other way: 47.9, then
62.0, then 64.2. The lattice does not stabilise the roadmap. It surrounds an
equally unstable core with a larger number of stable virtual vertices, and only
the denominator moves.

Nor is the count merely flat. It rises, because each obstacle now has more
neighbouring sites to form Voronoi vertices against, so an obstacle that moves
displaces more vertices than before.

**Every other cost moves the wrong way too.** The search itself gets slower in
absolute terms, 0.249 to 0.627 ms, because the graph is three times larger.
Building it goes from 2.2 ms to 27.5 ms on an eight-obstacle scene. And D* Lite
stays slower than a full rebuild throughout, between 1.67x and 2.01x, with no
trend toward crossing 1.0 as the lattice tightens.

So the precondition fails. A fixed site backbone was the one structure that
might have made incremental repair worth attempting here, and it does not hold
the roadmap still. Incremental tessellation is not future work blocked on
engineering effort; it is blocked on a property this domain does not have.
