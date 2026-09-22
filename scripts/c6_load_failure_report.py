#!/usr/bin/env python3
"""Explain retained independent failures without changing scores or evidence."""
import hashlib
import json
from collections import Counter
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1];BOARD=ROOT/'c6_drive'
source=BOARD/'reports/load-servo-holdout02.json'
report=json.loads(source.read_text());limits=report['acceptance']
keys=('final_task_error_mm','cross_track_peak_mm','final_heading_error_deg','tail_position_range_mm')
cases=[];bounds=Counter();signatures=Counter()
for row in report['results']:
    if row['family']!='load' or row['variant']!='load_servo':continue
    metrics=row['metrics'];last=row['trace'][-1]
    goal_shortfall=float(np.linalg.norm(np.array(last['goal'][:2])-last['task'][:2])*1000)
    failed=[k for k in keys if metrics[k]>limits[k]]
    tags=[]
    if not metrics['within_motion_bounds']:
        bounds.update(failed)
        if metrics['max_odometry_gap_mm']>10:tags.append('odometry_body_gap')
        if goal_shortfall>10:tags.append('reference_shortfall')
        if metrics['tail_position_range_mm']>limits['tail_position_range_mm']:tags.append('settling_motion')
        signatures.update(tags)
    cases.append(dict(name=row['case']['name'],passed=metrics['within_motion_bounds'],failed_bounds=failed,
                      evidence_tags=tags,reference_shortfall_mm=goal_shortfall,metrics=metrics))
summary=dict(source=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
             scope='Retained simulation evidence only; categories overlap, no hardware diagnosis',
             attempted=len(cases),passed=sum(c['passed'] for c in cases),
             failed_bounds=dict(bounds),signatures=dict(signatures),cases=cases)
(BOARD/'reports/load-servo-failure-analysis.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12,'svg.fonttype':'none'})
fig,axes=plt.subplots(3,1,figsize=(11,12),layout='constrained')
for ax,name,title in zip(axes,['load_unseen_18103','load_unseen_18124','load_unseen_18113'],[
    'Wheel odometry reaches the request; the body does not',
    'Load derating shortens the reference; requested travel is not completed',
    'Final position is close; residual settling motion still fails the bound']):
    row=next(r for r in report['results'] if r['variant']=='load_servo' and r['case']['name']==name)
    axis=row['case']['axis'];sign=np.sign(row['case']['speed']);trace=row['trace'];t=[f['t'] for f in trace]
    for key,label,color,style in [('task','Original request','#74808b','--'),('goal','Governed reference','#ae702c',':'),
                                 ('estimate','Encoder estimate','#3b79c4','-'),('truth','Simulated body','#007e79','-')]:
        ax.plot(t,[f[key][axis]*1000*sign for f in trace],color=color,ls=style,label=label,lw=2)
    m=row['metrics'];ax.set(title=title,xlabel='Time (s)',ylabel='Travel along requested direction (mm)')
    ax.text(.02,.95,f"{name} | final error {m['final_task_error_mm']:.1f} mm | settling range {m['tail_position_range_mm']:.1f} mm",
            transform=ax.transAxes,va='top',fontsize=10)
    ax.grid(alpha=.18);ax.legend(loc='lower right',fontsize=10)
    if name.endswith('18113'):ax.set_ylim(111,130);ax.set_xlim(5,8)
fig.suptitle('Why the load servo is not 36/36',fontsize=23,weight='bold')
fig.supxlabel('17/36 passed unchanged bounds. Assumed physics; body truth is never supplied to the controller.\nThese cases cannot establish the cause of the real robot’s slow start.',fontsize=11)
for ext in ('png','svg'):fig.savefig(BOARD/f'portfolio/load-servo-failures.{ext}',dpi=150)
print(json.dumps({k:summary[k] for k in ('attempted','passed','failed_bounds','signatures')},indent=2))
