# Clearance by cause: where each planner's tightest moment comes from

The recount (`docs/replan-count-recount.md`) left one number unexplained:
Voronoi's minimum clearance of 16 and 27 mm against 210 and 223 for the
visibility graph. An obstacle centre 16 mm from the path means the robot was
inside the obstacle. This note splits that minimum by cause, with n = 200 per
cell, and traces the residual to one line of code.

Harness: `scripts/dynamic_scenario.py --sweep-speeds 0.5 1.0 1.5 2.0 2.5 3.0
--samples 200`, the da Silva Costa and Tonidandel loops, their geometric
trigger (obstacle within 90 mm of the path), robot at 1.5 m/s, 50 ms tick,
same phase draws at every speed. Data: `results/speed-sweep/scenario_1.json`
and `scenario_2.json`, 20 September 2026, harness at e6f638f plus the 1 mm
tolerance below.

## Three clearances, all at plan time

For every successful plan the harness records, against the obstacles as
they stood when the plan was made (`scripts/dynamic_scenario.py`, `plan_from`):

- rooted: nearest obstacle centre to the plan rooted at the robot,
  `point_to_path_mm(o.pos_mm, (p, *waypoints))`;
- start: distance from the robot itself to the nearest obstacle centre,
  `math.dist(p, o.pos_mm)`;
- route: nearest obstacle centre to the plan beyond its first leg,
  `_rooted_at(p, waypoints)[1:]`, the part the roadmap chose; absent when
  the plan was a single leg.

A run's minimum of each is kept. "At robot" below is the share of runs whose
rooted minimum equals the start minimum: the tight spot was where the robot
already stood when it replanned, not where the planner sent it.

## Measured

Geometric trigger, n = 200 per cell. Clearances in mm, obstacle centre to
path. `route < 209` allows 1 mm below the 210 mm inflation disc, for the
reason given under the visibility graph.

Scenario 1, 7 obstacles:

| planner | obst m/s | rooted min, median | rooted min, min | at robot | route min, median | route min, min | route < 209 |
|---|---|---|---|---|---|---|---|
| PRM | 0.5 | 227 | 210 | 17% | 260 | 210.3 | 0.0% |
| PRM | 1.0 | 224 | 210 | 16% | 239 | 210.3 | 0.0% |
| PRM | 2.0 | 219 | 210 | 12% | 229 | 210.3 | 0.0% |
| PRM | 3.0 | 218 | 210 | 12% | 224 | 210.2 | 0.0% |
| visibility | 0.5 | 210 | 210 | 0% | 210 | 210.0 | 0.0% |
| visibility | 1.0 | 210 | 210 | 0% | 210 | 210.0 | 0.0% |
| visibility | 2.0 | 210 | 210 | 0% | 210 | 210.0 | 0.0% |
| visibility | 3.0 | 210 | 210 | 0% | 210 | 210.0 | 0.0% |
| Voronoi | 0.5 | 278 | 75 | 31% | 441 | 183.5 | 6.5% |
| Voronoi | 1.0 | 235 | 16 | 40% | 343 | 181.0 | 10.0% |
| Voronoi | 2.0 | 208 | 2 | 47% | 245 | 180.5 | 17.5% |
| Voronoi | 3.0 | 183 | 4 | 55% | 241 | 180.5 | 19.5% |

Scenario 2, 5 obstacles:

| planner | obst m/s | rooted min, median | rooted min, min | at robot | route min, median | route min, min | route < 209 |
|---|---|---|---|---|---|---|---|
| PRM | 0.5 | 225 | 210 | 18% | 239 | 210.0 | 0.0% |
| PRM | 1.0 | 217 | 210 | 20% | 222 | 210.2 | 0.0% |
| PRM | 2.0 | 216 | 210 | 14% | 219 | 210.2 | 0.0% |
| PRM | 3.0 | 214 | 210 | 12% | 217 | 210.0 | 0.0% |
| visibility | 0.5 | 210 | 210 | 0% | 210 | 210.0 | 0.0% |
| visibility | 1.0 | 210 | 210 | 0% | 210 | 210.0 | 0.0% |
| visibility | 2.0 | 210 | 210 | 0% | 210 | 210.0 | 0.0% |
| visibility | 3.0 | 210 | 210 | 0% | 210 | 210.0 | 0.0% |
| Voronoi | 0.5 | 216 | 75 | 31% | 269 | 180.5 | 27.0% |
| Voronoi | 1.0 | 193 | 27 | 42% | 210 | 180.1 | 48.5% |
| Voronoi | 2.0 | 141 | 4 | 57% | 200 | 180.3 | 66.0% |
| Voronoi | 3.0 | 78 | 1 | 74% | 197 | 180.5 | 73.0% |

The 1.5 and 2.5 m/s rows are left out for width; the full tables print with
`--from-speed-json results/speed-sweep/scenario_1.json`.

## Reading

**The 16 and 27 mm minima were the robot, not the route.** In 31 to 74
percent of Voronoi runs the rooted minimum is the robot's own distance to an
obstacle at the moment it replanned. This harness counts contact and does not
prevent it, so a robot that has been overrun replans from inside the obstacle
and the rooted clearance records the overlap. The other two planners never
show this because they refuse the call: PRM returns "start or goal lies
inside an inflated obstacle" and the visibility graph fails the same way, and
a failed call records no clearance. Voronoi instead escapes the containing
obstacle (`_escape_waypoint_from_containing_obstacles`) and plans. The rooted
minimum therefore compares failure handling, not routing, and should not be
quoted as a planner's clearance.

**On the route itself, PRM never comes inside the inflation disc and the
visibility graph runs exactly along it.** PRM tests every segment against
obstacles inflated to 210 mm (`planners/PRM/prm_dijkstra.py`, `radii`), so
its route minimum is 210.0 to 210.3 in every cell. The visibility graph
routes along the vertices and edges of a circumscribed hexagon whose apothem
is 210 mm (`planners/VisibilityGraph/visibility_graph.py`,
`_inflate_circle_to_polygon`), so its route minimum is 210.0 in every run,
and floating point puts it a fraction of a millimetre under 210 in 42 to 88
percent of runs. Without the 1 mm tolerance that read as corner cutting; it
is arithmetic.

**Voronoi's route minimum is 180 mm, in every cell, and that is one line.**
The roadmap edges do keep 210 mm: the generator inflates each obstacle by
`VORONOI_MIN_CLEARANCE_MM` = robot radius 90 + safe margin 30 on top of the
90 mm obstacle radius and promises every edge at least that
(`world/map/voronoi/voronoi_generator.py`, lines 100 and 136). The legs that
attach the start and the goal to the roadmap are checked separately, by
`PlanningScene.is_path_free` (`world/scene.py`):

```python
    def is_path_free(self, start_pos, end_pos, ignore_robots=None, clearance: float = 0.0, horizon_ms=None) -> bool:
        ...
            or distance_2_segment(obstacle.pos_mm, start_pos, end_pos)
            > obstacle.radius_mm + ROBOT_RADIUS_MM + clearance
```

`VoronoiDijkstraPlanner._connect_temporary_node` calls it without
`clearance`, so those legs pass at 90 + 90 + 0 = 180 mm, the body-contact
distance, 30 mm inside what its own roadmap and both other planners require.
The measured minimum of 180.0 to 180.5 in all twelve Voronoi cells is that
leg. It is also why the share of Voronoi routes inside the disc grows with
obstacle speed and in the tighter scenario: faster obstacles make more of
the replans start and finish close to something.

**What this changes and what it does not.** The clearance means already
reported (Voronoi 387 and 326 mm against 217 and 223 for the visibility graph
and 291 and 285 for PRM) are means over plans and stand; the route medians
above rank the same way. Voronoi's *minimum* is not comparable to the other
two until its connection legs are checked with the same 30 mm margin. That
is a one-argument change (`clearance=SAFE_MARGIN` in the two
`is_path_free` calls) and it would move every Voronoi result, so it is not
made here; the experiments stand as run and the method is documented. A run
with the margin applied is the experiment that would say how much of
Voronoi's clearance advantage survives, and no number for that is claimed.

`horizon_ms` is accepted and discarded by `is_path_free` (`del horizon_ms`),
so the prediction horizon does not reach the connection test either. Noted,
not measured.
