"""How big and how slow is a Voronoi roadmap under each site placement mode?

The paper recommends "a Voronoi roadmap on a fixed backbone of virtual sites"
as the one structure incremental repair could target. The planner ships
placement_mode="density_grid", whose sites move with the obstacles. Before
recommending "grid", check what it costs: the full match replay under grid ran
past 15 minutes without emitting a row, which suggests the fixed lattice is not
a drop-in swap.

One 8-obstacle scene, one build per mode, node and edge counts plus wall time.
"""

import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = Path(sys.argv[1])
sys.path.insert(0, str(ROOT / "src"))

from research_sdk.planners.common import DEFAULT_ROBOT_RADIUS_MM
from research_sdk.world.map.voronoi.voronoi_generator import (
    generate_voronoi_map_from_scene,
)
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene

OBSTACLES = [
    (-1000, 400), (500, -700), (1500, 900), (-2000, -1200),
    (2500, 300), (0, 1500), (-500, -1800), (3000, -900),
]

scene = PlanningScene(
    timestamp=0.0,
    obstacles=tuple(
        PlanningObstacle(robot_id=i, isYellow=False, pos_mm=(float(x), float(y)),
                         radius_mm=DEFAULT_ROBOT_RADIUS_MM)
        for i, (x, y) in enumerate(OBSTACLES)
    ),
    field=FieldDimensions(),
)

print(f"{'placement_mode':<16}{'nodes':>8}{'edges':>9}{'build ms':>11}")
print("-" * 44)
for mode in ("density_grid", "grid", "random"):
    t0 = time.perf_counter()
    try:
        m = generate_voronoi_map_from_scene(scene, placement_mode=mode)
    except Exception as exc:  # noqa: BLE001
        print(f"{mode:<16}  failed: {type(exc).__name__}: {exc}")
        continue
    ms = (time.perf_counter() - t0) * 1000.0
    print(f"{mode:<16}{len(m.nodes):>8}{len(m.edges):>9}{ms:>11.1f}")

print("\nEvery number in the study used density_grid, the planner's default.")

# The default grid spacing is min_clearance_mm * 2.0, which is why "grid" is
# unusable above. Sweep the spacing to find where a fixed lattice becomes
# affordable against the 50 ms control tick.
print(f"\n{'grid spacing mm':<17}{'nodes':>8}{'edges':>9}{'build ms':>11}")
print("-" * 45)
for spacing in (240.0, 400.0, 600.0, 900.0, 1200.0, 1800.0):
    t0 = time.perf_counter()
    m = generate_voronoi_map_from_scene(
        scene, placement_mode="grid",
        grid_spacing_x_mm=spacing, grid_spacing_y_mm=spacing,
    )
    ms = (time.perf_counter() - t0) * 1000.0
    print(f"{spacing:<17.0f}{len(m.nodes):>8}{len(m.edges):>9}{ms:>11.1f}")
print("\n50 ms is one control tick; a build above that cannot run at 20 Hz.")
