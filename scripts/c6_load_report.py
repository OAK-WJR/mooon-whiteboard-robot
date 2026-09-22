#!/usr/bin/env python3
"""Publish heavy-load evidence and English portfolio figures, no hardware."""
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

ROOT=Path(__file__).resolve().parents[1]; BOARD=ROOT/'c6_drive'
source=BOARD/'reports/heavy-load-01.json'
report=json.loads(source.read_text())
for name,digest in report['source_sha256'].items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest, name
rows=report['results']
profiles=['free','bearings','pad','high_drag']
labels=['No added drag','Bearing drag','Bearings + 2 N pad','High drag + 5 N pad']
groups=[]
for profile in profiles:
    selected=[r for r in rows if r['case']['profile']==profile and r['case']['motor_force_N']==4]
    groups.append(dict(profile=profile,n=len(selected),passed=sum(r['metrics']['within_motion_bounds'] for r in selected)))
summary=dict(scope=report['scope'],assumptions=report['assumptions'],profiles=report['profiles'],
             acceptance=report['acceptance'],source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
             n=len(rows),passed=sum(r['metrics']['within_motion_bounds'] for r in rows),groups=groups,
             solver_warnings=sum(r['metrics']['solver_warnings'] for r in rows),
             cases=[dict(case=r['case'],resistance=r['resistance'],metrics=r['metrics'],
                         trace=[{key:f[key] for key in ('t','truth','estimate','task','goal','pwm')} for f in r['trace']]) for r in rows])
(BOARD/'reports/heavy-load-summary.json').write_text(json.dumps(summary,allow_nan=False,separators=(',',':')))

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':14,'svg.fonttype':'none'})
fig,ax=plt.subplots(figsize=(11,6.6),layout='constrained')
grid=np.zeros((3,4))
for mi,mass in enumerate((.5,1.,2.)):
    for pi,profile in enumerate(profiles):
        selected=[r for r in rows if r['case']['mass']==mass and r['case']['profile']==profile and r['case']['motor_force_N']==4]
        grid[mi,pi]=sum(r['metrics']['within_motion_bounds'] for r in selected)
ax.imshow(grid,vmin=0,vmax=4,cmap=ListedColormap(['#df927c','#edb399','#eecfaa','#b1d5c9','#69b29e']),aspect='auto')
for i in range(3):
    for j in range(4): ax.text(j,i,f'{int(grid[i,j])} / 4',ha='center',va='center',fontsize=28,weight='bold',color='#202e36')
ax.set(xticks=range(4),xticklabels=['No added\ndrag','Bearing\ndrag','Bearings +\n2 N pad','High drag +\n5 N pad'],
       yticks=range(3),yticklabels=['0.5 kg','1.0 kg','2.0 kg'],ylabel='Assumed total mass')
ax.tick_params(length=0,pad=12)
fig.suptitle('Heavy-load stress test | Passed / tested',fontsize=24,weight='bold')
fig.supxlabel('Two directions × two requested speeds (0.04 / 0.08 m/s). Same unchanged C++ controller.\n4 N per-wheel motor authority is assumed, not measured. Pad force is nominal spring preload.\nNo added drag still includes the archived motor/contact model. Simulation only.',fontsize=12)
fig.savefig(BOARD/'portfolio/heavy-load-grid.png',dpi=150);fig.savefig(BOARD/'portfolio/heavy-load-grid.svg');plt.close(fig)

fig,axes=plt.subplots(2,1,figsize=(11,9),layout='constrained')
for ax,name,title in zip(axes,['1kg_high_drag_x_0.08','2kg_high_drag_x_0.08'],
                       ['1 kg + high drag: wheels turn, body barely travels','2 kg + high drag: output limit and reference slowdown']):
    r=next(r for r in rows if r['case']['name']==name);t=[f['t'] for f in r['trace']]
    for key,label,color,style in [('task','Original request','#596e7c',':'),('goal','Governed goal','#8963a8','-.'),
                                  ('estimate','Encoder estimate','#007e79','--'),('truth','Simulated body','#b45330','-')]:
        ax.plot(t,[f[key][0]*1000 for f in r['trace']],label=label,color=color,ls=style,lw=2.5)
    ax.set(title=title,xlabel='Time (s)',ylabel='Forward travel (mm)');ax.grid(alpha=.18)
    ax.legend(fontsize=11,loc='upper left')
fig.suptitle('Two different failures under resistance',fontsize=23,weight='bold')
fig.supxlabel('Assumed wheel/pad friction and 4 N motor force per wheel. No robot validation.\nBody truth is available to the scorer only. Encoder feedback cannot observe all sliding.',fontsize=12)
fig.savefig(BOARD/'portfolio/heavy-load-failures.png',dpi=150);fig.savefig(BOARD/'portfolio/heavy-load-failures.svg');plt.close(fig)
print(json.dumps({k:summary[k] for k in ('n','passed','groups','solver_warnings')},indent=2))
