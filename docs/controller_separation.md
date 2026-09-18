# Closed-loop measurement separation

The demo now distinguishes planner outcomes, planned-path geometry, path following and observation error. The previous adapter discarded the distinction between a failed plan and a direct route. An empty result could command motion towards the goal through obstacles. Failed and exhausted paths now command zero velocity and retry; direct routes retain an explicit goal waypoint. Direct, planned and failed calls have separate counters.

## What is measured

`--grsim-pixel-truth` is specific to the inspected local grSim source at revision `fe2bd2915a46f9f11ea6cb48dc426b8047952073`. In `src/sslworld.cpp`, `generatePacket` assigns the robot's unnoised world position in millimetres to `pixel_x` and `pixel_y`, then adds noise to `x` and `y`. Ordinary SSL-Vision pixel coordinates do not have this meaning. The option is off by default.

The reader assembles four camera packets sharing the same simulator capture timestamp. It does not compare robot positions taken at different capture times. Each controlled robot's JSON includes:

- Returned-path geometry against the request's observed obstacles and the latest synchronized unnoised obstacle positions.
- Distance of the unnoised robot position to the saved planned polyline. Its start is fixed at planning time, not replaced by the live position each tick.
- Observation error between the noisy and unnoised position in the same snapshot.
- Observed and unnoised centre separation, with a complete-snapshot flag.
- Sampled geometric overlap episodes for 90 mm circular bodies at or below 180 mm separation, counted on pairwise entry.
- In the final instrument, commanded velocities, pursued waypoints and their segment separation, linked to the originating plan.

These quantities separate evidence, not causes automatically. Tracking distance includes the effect of an inaccurately localized planning start. A returned path can be clear in the observed scene and unsafe in the unnoised scene. A later-moving obstacle can invalidate a previously safe path. Read the timeline before assigning a failure to a component.

Sampled overlap is not the simulator's physics contact event count. Contacts between samples can be missed. Missing snapshots limit coverage. Recorded replacement packets can move obstacles into other bodies. Do not label these counts as verified physical collisions.

## Reproduction

Run grSim first, then use WSL and the shared-search checkout. This diagnostic resets a six-robot-per-team scene, parks and disables the other robots, and places one stationary yellow obstacle at the centre. It is not recorded-match replay.

```bash
/home/paulk/rsdk-venv/bin/python scripts/run_controller_check.py \
  --planner visibility --duration 20 --out results/controller-separation/new-run.json

/home/paulk/rsdk-venv/bin/python scripts/plot_controller_diagnostics.py \
  results/controller-separation/new-run.json --out results/controller-separation/new-run.png
```

Use a fresh output filename. The driver records the configuration, code revision where available, and source hashes at import. The on-disk grSim configuration is recorded; that alone does not establish which settings a running simulator loaded.

The waypoint acceptance radius remains 150 mm by default, preserving the existing pursuit controller. `--waypoint-tolerance-mm` exposes it for controlled tests. No local-avoidance layer has been added, because that would confound diagnosing the existing controller. The unsafe no-path fallback was corrected separately.

The geometric trigger also differs between tiers: the historical grSim default is 180 mm centre separation, whereas the offline experiment uses 90 mm. The new `--trigger-centre-threshold-mm` option makes this choice explicit and records it. Selecting 90 reproduces that distance test; it does not by itself equalize every other implementation setting.

## Diagnostic observations

The three `*-static-150-parked.json` files are individual 20-second runs, not paired statistical comparisons. The running setup's configuration file recorded 23 mm position noise and 9 ms delay. All sampled snapshots were complete.

| Planner | Complete snapshots | Minimum observed separation (mm) | Minimum unnoised separation (mm) | Maximum tracking distance (mm) | Maximum observation error (mm) |
|---|---:|---:|---:|---:|---:|
| Visibility | 395 | 178.6 | 228.2 | 63.4 | 82.6 |
| Voronoi | 396 | 1528.2 | 1567.3 | 67.1 | 101.9 |
| PRM | 395 | 562.3 | 602.3 | 53.7 | 85.0 |

Each had zero sampled geometric overlap episodes. The visibility run demonstrates why noisy closest approach is not a contact counter: it crossed the observed 180 mm threshold while the unnoised sampled separation remained above 228 mm.

The initial `visibility-static-150.json` is excluded: the fixture stacked disabled bodies at the active robots' positions. Parking them separately restored the intended experiment. Keep this failed fixture run as a record of instrument debugging, not evidence about the planner.

`visibility-final.json` exercises the final instrument, including source hashes and commands. Its results are separate from the table above.

## What the grSim tier can now support

New runs can report plan failures, tracking distances, observation errors and explicitly defined sampled overlap, with their source and coverage. Historical closest-distance outputs cannot be retroactively relabelled as collision counts. A three-planner study still needs matched settings, repeated runs and a declared contact definition. Direct physics contacts require a contact-event feed or equivalent simulator instrumentation beyond these vision packets.
