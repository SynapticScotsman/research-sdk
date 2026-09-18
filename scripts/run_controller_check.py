"""One robot, one stationary central obstacle; run inside the grSim WSL host.

This resets the six-robot-per-team simulator scene. It leaves all robots stopped.
It is a diagnostic fixture, not a planner benchmark or recorded-match replay.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from drive_grsim import grSimSender, grSimPacketFactory, RobotCommand


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner',choices=['prm','visibility','voronoi'],default='voronoi')
    parser.add_argument('--duration',type=float,default=20)
    parser.add_argument('--waypoint-tolerance-mm',type=float,default=150)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if args.out.exists():
        parser.error('output already exists; use a new filename')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    sender=grSimSender()
    robots=[]
    for yellow in (False,True):
        for rid in range(6):
            sender.send_robot_command(RobotCommand(robot_id=rid,isYellow=yellow))
            # Park disabled bodies separately as well. Some grSim builds keep
            # their collision geometry active after hiding their vision output.
            x = (0 if yellow else -3.2) if rid == 0 else (-2.5 + rid*.5)
            y = 0 if rid == 0 else (2.8 if yellow else -2.8)
            robots.append(dict(robot_id=rid,isYellow=yellow,x=x,y=y,orientation=0))
    packet=grSimPacketFactory.scenario_replacement_command(robots)
    for r in packet.replacement.robots:
        r.turnon = r.id == 0
    sender.send_packet(packet)
    time.sleep(.5)
    command=[sys.executable,str(Path(__file__).with_name('drive_grsim.py')),
             '--planner',args.planner,'--robots','1','--obstacles','1',
             '--opponents','static','--no-place','--grsim-pixel-truth',
             '--duration',str(args.duration),'--waypoint-tolerance-mm',str(args.waypoint_tolerance_mm),
             '--out-json',str(args.out)]
    result=subprocess.run(command,check=False)
    if result.returncode:
        return result.returncode
    data=json.loads(args.out.read_text())
    data['meta']['fixture']='one blue at (-3200,0), one static yellow at (0,0), other ten robots disabled'
    args.out.write_text(json.dumps(data,indent=1))
    for r in data['robots']:
        d=r['diagnostics']
        print(json.dumps({k:v for k,v in d.items() if k not in ('telemetry','plan_events','commands')},indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
