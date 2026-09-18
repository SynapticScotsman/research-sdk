import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import drive_grsim as driver
from closed_loop_diagnostics import Diagnostics


def test_failed_plan_is_not_a_direct_route(monkeypatch):
    monkeypatch.setitem(driver.PLANNERS, 'prm', lambda *a: SimpleNamespace(success=False, waypoints_mm=(), message='no path'))
    call = driver.plan_for('prm', (0, 0), (1000, 0), (), key='failure')
    assert call.status == 'failed'
    assert call.waypoints == ()


def test_voronoi_direct_retains_goal(monkeypatch):
    monkeypatch.setattr(driver, 'VoronoiDijkstraPlanner', lambda: SimpleNamespace(plan=lambda *a, **k: SimpleNamespace(used_direct_path=True, waypoints_mm=(), target_mm=(1000,0))))
    call = driver.plan_for('voronoi', (0,0), (1000,0), (), key='direct')
    assert call.status == 'direct'
    assert call.waypoints == ((1000,0),)


def test_voronoi_failed_plan_is_not_direct(monkeypatch):
    monkeypatch.setattr(driver, 'VoronoiDijkstraPlanner', lambda: SimpleNamespace(plan=lambda *a, **k: SimpleNamespace(used_direct_path=False, waypoints_mm=(), target_mm=(1000,0))))
    assert driver.plan_for('voronoi', (0,0), (1000,0), (), key='failed').status == 'failed'


def test_tracking_uses_fixed_reference_and_distinguishes_noise():
    d=Diagnostics()
    d.planned(t=0,start=(0,0),path=((1000,0),),observed_obstacles=[(500,300)],truth_obstacles=[(500,300)],status='path',ms=1)
    d.sample(t=1,observed={(False,0):(500,115),(True,0):(500,300)},truth={(False,0):(500,100),(True,0):(500,300)},robot_key=(False,0),expected_keys={(False,0),(True,0)})
    sample=d.samples[-1]
    assert sample['tracking_error_mm']==100
    assert sample['observation_error_mm']==15
    assert sample['truth_separation_mm']==200
    assert d.report()['sampled_geometric_overlap_episodes']==0


def test_overlap_episode_not_counted_once_per_tick_or_with_missing_truth():
    d=Diagnostics()
    for t,y in enumerate((170,170,200,170)):
        positions={(False,0):(0,0),(True,0):(0,y)}
        d.sample(t=t,observed=positions,truth=positions,robot_key=(False,0),expected_keys=set(positions))
    assert d.report()['sampled_geometric_overlap_episodes']==2
    d.sample(t=4,observed={},truth={(False,0):(0,0)},robot_key=(False,0),expected_keys={(False,0),(True,0)})
    assert d.samples[-1]['truth_separation_mm'] is None
    assert d.report()['sampled_geometric_overlap_episodes']==2


def test_controller_stops_on_missing_or_exhausted_path():
    assert driver.follow_waypoints((0,0),0,(),1.2,150)==((),0,0)
    assert driver.follow_waypoints((0,0),0,((10,0),),1.2,150)==((),0,0)
    path,vx,vy=driver.follow_waypoints((0,0),0,((1000,0),),1.2,150)
    assert path==((1000,0),)
    assert vx==1.2 and vy==0


def test_pixel_truth_requires_opt_in_and_matching_four_camera_timestamps(monkeypatch):
    import threading
    queue=[]
    def listen():
        if not queue:
            raise BlockingIOError
        return queue.pop(0)
    sock=SimpleNamespace(is_running=threading.Event(),listen=listen)
    sock.sock=SimpleNamespace(setblocking=lambda _:None)
    monkeypatch.setattr(driver,'grSimVision',lambda _:sock)
    def packet(camera,t):
        robot=SimpleNamespace(robot_id=0,x=110,y=0,orientation=0,pixel_x=100,pixel_y=0,HasField=lambda _:True)
        detection=SimpleNamespace(t_capture=t,camera_id=camera,robots_blue=[robot],robots_yellow=[])
        return SimpleNamespace(detection=detection,HasField=lambda _:True)
    reader=driver.Vision(pixel_truth=True)
    queue.extend(packet(i,i) for i in range(4))
    reader.update()
    assert reader.capture_s is None
    queue.extend(packet(i,10) for i in range(4))
    reader.update()
    assert reader.capture_s==10
    assert reader.truth_snapshot[(False,0)]==(100,0)
    assert reader.observed_snapshot[(False,0)]==(110,0)
    reader=driver.Vision()
    queue.extend(packet(i,11) for i in range(4))
    reader.update()
    assert reader.truth_snapshot=={}
