"""Plot saved telemetry without rerunning grSim."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('run',type=Path)
parser.add_argument('--out',type=Path,required=True)
args=parser.parse_args()
data=json.loads(args.run.read_text())
diagnostic=data['robots'][0]['diagnostics']
samples=diagnostic['telemetry']
origin=samples[0]['t_capture_s']
plt.rcParams.update({'figure.facecolor':'#16181b','axes.facecolor':'#16181b',
    'text.color':'#e6e4df','axes.labelcolor':'#e6e4df','xtick.color':'#b2b6bf',
    'ytick.color':'#b2b6bf','axes.edgecolor':'#727883','font.size':11})
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
for ax in axes:
    ax.grid(alpha=.18)
    ax.spines[['top','right']].set_visible(False)
def line(ax,key,label,color):
    valid=[s for s in samples if s[key] is not None]
    ax.plot([s['t_capture_s']-origin for s in valid],[s[key] for s in valid],label=label,color=color,lw=1.4)
line(axes[0],'observed_separation_mm','Observed','#80baff')
line(axes[0],'truth_separation_mm','Unnoised grSim coordinates','#ff87b6')
axes[0].axhline(180,color='#ffd277',linestyle='--',label='180 mm geometric contact')
axes[0].set_ylabel('Centre separation (mm)')
axes[0].legend(facecolor='#16181b',labelcolor='#e6e4df',loc='upper right')
line(axes[1],'tracking_error_mm','Unnoised distance to saved planned polyline','#ff87b6')
axes[1].set_ylabel('Tracking distance (mm)')
axes[1].set_ylim(bottom=0)
line(axes[2],'observation_error_mm','Observed versus unnoised position','#80baff')
axes[2].set_ylabel('Observation error (mm)')
axes[2].set_ylim(bottom=0)
axes[2].set_xlabel('Elapsed simulator capture time (s)')
fig.suptitle(f"{data['meta']['planner'].capitalize()} · one stationary obstacle · one diagnostic run\nTracking distance is not a causal attribution; overlap is sampled geometry, not physics contact",fontsize=13)
fig.savefig(args.out,dpi=160)
print(args.out)
