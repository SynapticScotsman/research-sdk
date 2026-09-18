"""Separate plan geometry, tracking and observation error; all distances in mm.

Pixel fields are world coordinates ONLY for the inspected grSim implementation.
They are never treated as truth unless the driver explicitly enables that mode.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


def point_segment(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((point[0]-a[0])*dx + (point[1]-a[1])*dy)/length2)) if length2 else 0.0
    return math.dist(point, (a[0]+t*dx, a[1]+t*dy))


def distance_to_path(point, path):
    return min((point_segment(point, a, b) for a, b in zip(path, path[1:])), default=None)


def path_separation(path, obstacles):
    values = [distance_to_path(p, path) for p in obstacles]
    return min((v for v in values if v is not None), default=None)


@dataclass
class Diagnostics:
    reference: tuple = ()
    plans: list = field(default_factory=list)
    samples: list = field(default_factory=list)
    commands: list = field(default_factory=list)
    active_pairs: set = field(default_factory=set)
    overlap_episodes: int = 0
    last_t: float | None = None

    def planned(self, *, t, start, path, observed_obstacles, truth_obstacles, status, ms):
        # Preserve the actual planned start: prepending the live robot position
        # each tick would make tracking error identically zero.
        self.reference = (start, *path) if path else ()
        self.plans.append(dict(t_capture_s=t, status=status, planning_ms=ms,
                               reference_path_mm=self.reference,
                               observed_obstacles_mm=observed_obstacles,
                               truth_obstacles_mm=truth_obstacles,
                               observed_plan_separation_mm=path_separation(self.reference, observed_obstacles),
                               truth_plan_separation_mm=path_separation(self.reference, truth_obstacles or [])))

    def sample(self, *, t, observed, truth, robot_key, expected_keys):
        if t is None or (self.last_t is not None and t <= self.last_t):
            return
        self.last_t = t
        here = observed.get(robot_key)
        actual = truth.get(robot_key)
        peers = {k: p for k, p in truth.items() if k != robot_key}
        observed_peers = {k: p for k, p in observed.items() if k != robot_key}
        complete = actual is not None and expected_keys.issubset(truth)
        pairs = {k for k,p in peers.items() if actual is not None and math.dist(actual,p) <= 180.0}
        if complete:
            self.overlap_episodes += len(pairs - self.active_pairs)
            self.active_pairs = pairs
        self.samples.append(dict(t_capture_s=t, observed_mm=here, truth_mm=actual,
            truth_complete=complete, observed_obstacles_mm=list(observed_peers.values()),
            truth_obstacles_mm=list(peers.values()),
            observed_separation_mm=min((math.dist(here,p) for p in observed_peers.values()),default=None) if here else None,
            truth_separation_mm=min((math.dist(actual,p) for p in peers.values()),default=None) if complete else None,
            observation_error_mm=math.dist(here,actual) if here is not None and actual is not None else None,
            tracking_error_mm=distance_to_path(actual,self.reference) if actual is not None else None,
            observed_tracking_error_mm=distance_to_path(here,self.reference) if here is not None else None,
            reference_plan_index=len(self.plans)-1 if self.plans else None,
            sampled_overlap_pairs=[list(k) for k in pairs] if complete else None))

    def report(self):
        def values(key):
            return [s[key] for s in self.samples if s[key] is not None]
        def maximum(key):
            return max(values(key),default=None)
        def minimum(key):
            return min(values(key),default=None)
        return dict(samples=len(self.samples), truth_complete_samples=sum(s['truth_complete'] for s in self.samples),
            sampled_geometric_overlap_episodes=self.overlap_episodes if any(s['truth_complete'] for s in self.samples) else None,
            min_truth_separation_mm=minimum('truth_separation_mm'),
            min_observed_separation_mm=minimum('observed_separation_mm'),
            max_tracking_error_mm=maximum('tracking_error_mm'),
            max_observation_error_mm=maximum('observation_error_mm'),
            interpretation='Sampled geometric overlap of 90 mm circles, not physics contact events. Missing frames and between-sample contacts can be missed. Recorded replacement can cause overlaps.',
            plan_events=self.plans, telemetry=self.samples, commands=self.commands)

    def commanded(self, *, t, here, path, velocity, truth_here, truth_obstacles):
        segment = (here,path[0]) if path else ()
        actual_segment = (truth_here,path[0]) if path and truth_here is not None else ()
        self.commands.append(dict(t_capture_s=t, reference_plan_index=len(self.plans)-1 if self.plans else None,
            remaining_waypoints_mm=path, observed_start_mm=here, velocity_robot_mps=velocity,
            truth_command_segment_separation_mm=path_separation(actual_segment,truth_obstacles),
            observed_start_command_segment_truth_separation_mm=path_separation(segment,truth_obstacles)))
