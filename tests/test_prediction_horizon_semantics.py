import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamic_scenario import TriggerContext, trigger_predictive, trigger_geometric, TRIGGERS
from research_sdk.planners.common import Obstacle


def context(y):
    current=(Obstacle(pos_mm=(500,y),robot_id=0),)
    previous=(Obstacle(pos_mm=(500,y+75),robot_id=0),)
    return TriggerContext(remaining_path=((0,0),(1000,0)),obstacles=current,
                          history=(previous,current),ticks_since_plan=1,dt_s=.05)


def test_small_projection_can_cross_threshold():
    ctx=context(120)
    assert not trigger_geometric(ctx)
    assert trigger_predictive(.05)(ctx)


def test_horizon_not_velocity_estimation_interval():
    ctx=context(400)
    assert not trigger_predictive(.05)(ctx)
    assert trigger_predictive(.25)(ctx)
    assert [t.__name__ for t in TRIGGERS if t.__name__.startswith('predictive')] == [
        'predictive_0.25s','predictive_0.50s','predictive_1.00s','predictive_0.25s_smooth5']
