# What each evaluation tier can actually measure

Written after three attempts to make one recorded clip serve both purposes.
It cannot, and the reason is a property of the recording rather than the setup.

## The two requirements pull against each other

A closed-loop run needs endpoints that are never inside an inflated obstacle, or
no path exists and the run measures how fast each planner reports failure
(`grsim-scenario-validity.md`, fault 1). It also needs the route to meet
traffic, or the planner is never exercised.

Optimising only the first picks the emptiest lane on the field, which is empty
precisely because the recorded robots avoid it. Measured on the y=1500 lane at
x=+/-3200: closest approach 465 to 905 mm against a 210 mm inflation radius,
zero failed calls, and replans equal to legs travelled plus zero. Nothing
happened.

`scripts/choose_traverse.py` scores both together and picked
`--traverse-x 2400 --lane-centre-mm 750`: endpoints 708 mm clear, an obstacle
within 400 mm of the straight line in 100 percent of frames, nearest approach
to the line 104 mm.

**That proxy is weaker than it looks.** Obstacle-to-line distance asks whether
traffic is near the route. The robot occupies one point on that route at a time,
and a working planner routes around a cluster before reaching it, so a corridor
full of traffic still gave 712 mm closest approach and 3 replans over 2 laps.
Distance to the line is a necessary condition for an encounter, not a sufficient
one.

## Why this log cannot test dynamic replanning

The trigger fires when an obstacle moves into a plan that was valid when made.
In the 180 to 210 s clip, nine of eleven robots hold position: spans of 81 to
753 mm over 30 s, against 8450 mm and 7784 mm for the two that move. A plan made
at the start of a leg is still valid at the end, so there is nothing to replan
around.

This is the recording. The replay reproduces it to within grSim's 23 mm vision
noise, verified by sampling the log the same way the replay steps it.

Nothing in the log is fast. Sampled at 4 Hz, every 30 s window runs 0.05 to
0.56 m/s per robot, where competitive play is 1 to 3 m/s. The 180 s window is
the most active one available.

Beware measuring this at the log's native 70 Hz: 23 mm of vision noise
differentiated over 14 ms is about 1.6 m/s of pure noise, which made every
window look like a scramble on a first pass.

## The division that follows

| tier | measures | why it is the right tool |
|---|---|---|
| offline `dynamic_scenario.py` | trigger policy, path stability, replan counts | obstacle speed is a controlled variable, swept 0.5 to 3.0 m/s, n=200 paired |
| grSim on recorded data | per-call planning cost, roadmap behaviour under a real field | real geometry, 11 robots, vision noise 23 mm, 9 ms latency, closed-loop control |

Roadmap cost is driven by obstacle count and layout, so a realistic but slow
field is still a fair test of construction cost. Trigger behaviour is driven by
obstacle motion, so it is not a fair test of that.

Presenting the two tiers this way is stronger than apologising for a quiet clip:
each measures what it can hold constant.

## If dynamic behaviour is wanted in grSim

It needs a livelier recording, not a different traverse. `fetch_match_log.sh`
pulls others. Check one before committing to runs:

    python scripts/choose_traverse.py --log NEW.log.gz --replay-team both --obstacles 11

A log worth using should show per-robot spans in the metres over 30 s for most
robots, not two of eleven.
