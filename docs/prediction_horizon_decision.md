# Prediction horizon decision

The supplied manuscript's 50 ms prediction horizon is incorrect for the saved experiments. Retain those experiments and correct the methods, rather than changing the implementation to match the draft.

| Quantity | Value | Evidence |
|---|---|---|
| Control interval | 0.05 s | `scripts/dynamic_scenario.py`, `CONTROL_TICK_S` |
| Trigger prediction horizons | 0.25, 0.50, 1.00 s | `TRIGGERS` and both `trigger_sweep_n200_scenario_*.json` result keys |
| Velocity-estimation interval | 0.05 s normally; 0.25 s for smooth5 | `TriggerContext.velocity_of` |
| Predictive horizon in saved speed sweeps | 0.25 s, including smooth5 where present | `speed_sweep*.json` result keys |
| Future positions tested per decision | H/4, H/2, 3H/4, H | `trigger_predictive`, default `steps=4` |

No 50 ms predictive result is present in the saved sweeps inspected. The separate Voronoi world-model configuration also has a horizon; it must not be substituted for the trigger's horizon. The grSim demo currently offers geometric and periodic triggers, not the predictive sweep policy.

## Replacement for the supplied §3.3

At each 50 ms control step, we estimate obstacle velocity from consecutive position observations. The predictive trigger checks the current remaining path against positions extrapolated at four equally spaced times up to a horizon H. We compare H = 0.25, 0.50 and 1.00 s in the trigger sweep. The speed sweeps use H = 0.25 s. A smoothed variant estimates velocity over five control steps, or 0.25 s, while retaining the 0.25 s prediction horizon. These are constant-velocity trigger checks, not model predictive control or collision-free trajectory guarantees.

## Consequence for §4.4

Report the horizon attached to each saved condition. Use the trigger sweeps for comparisons among horizons and the speed sweeps for the 0.25 s policy across speeds. Do not claim that 50 ms prediction was evaluated. No new sweep is required to repair the reporting inconsistency. Testing 50 ms would be a new ablation, not a reinterpretation of old results.

At an obstacle speed of 1.5 m/s, the projection distances are 75, 375, 750 and 1500 mm for horizons of 0.05, 0.25, 0.50 and 1.00 s. The obstacle's estimated velocity determines that projection, not the controlled robot's speed. A 75 mm displacement can still cross a trigger threshold; being smaller than a robot radius does not make its effect negligible. The saved results, not this distance argument, determine which horizon was measured.

One separate comparability issue remains explicit: the offline geometric trigger tests obstacle-centre distance against 90 mm, whereas the grSim demo adds the obstacle radius to its 90 mm threshold, yielding 180 mm for these obstacles. Closed-loop runs using that default are not an exact replication of the offline policy.
