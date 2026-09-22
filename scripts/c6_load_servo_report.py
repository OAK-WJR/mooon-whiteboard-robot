#!/usr/bin/env python3
"""Publish paired successor-controller results, preserving negative validation."""
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1];BOARD=ROOT/'c6_drive'
development=json.loads((BOARD/'reports/load-servo-dev06.json').read_text())
holdout=json.loads((BOARD/'reports/load-servo-holdout02.json').read_text())
for report in (development,holdout):
    for name,digest in report['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name

def group(rows,predicate):
    return {variant:dict(n=len(selected),passed=sum(x['metrics']['within_motion_bounds'] for x in selected),
                        faults=sum(bool(x['metrics']['faults']) for x in selected))
            for variant in ('fixed','load_servo')
            for selected in [[x for x in rows if x['variant']==variant and predicate(x)]]}

summary=dict(revision='encoder-load-servo-1',ready_for_hardware_use=False,
             status='Not promoted: independent heavy-load pass count regresses; wall hold unqualified',
             source_sha256=holdout['source_sha256'],
             development=group(development['results'],lambda r:r['family']=='load'),
             validation={g:group(holdout['results'],lambda r,g=g:r['family']=='load' if g=='load' else r['case']['name'].startswith(g+'_')) for g in ('load','wheel','floor','wall')},
             regressions=[x['case']['name'] for x,y in zip(holdout['results'][::2],holdout['results'][1::2]) if x['metrics']['within_motion_bounds'] and not y['metrics']['within_motion_bounds']],
             candidate_cases=[dict(case=x['case'],resistance=x['resistance'],metrics=x['metrics'],
                             trace=[{k:f[k] for k in ('t','truth','estimate','task','goal','pwm')} for f in x['trace']])
                             for x in development['results'] if x['family']=='load' and x['variant']=='load_servo'])
(BOARD/'reports/load-servo-summary.json').write_text(json.dumps(summary,separators=(',',':'),allow_nan=False))

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':14,'svg.fonttype':'none'})
fig,axes=plt.subplots(2,1,figsize=(10,9),layout='constrained')
for ax,name,title in zip(axes,['2kg_high_drag_y_0.04','2kg_high_drag_x_0.08'],
                        ['Low-speed loaded start: a development case improves','Higher-speed loaded travel: improvement still misses the target']):
    for variant,color,label in [('fixed','#ad663f','Previous servo'),('load_servo','#007e79','Load servo candidate')]:
        x=next(x for x in development['results'] if x['case']['name']==name and x['variant']==variant)
        t=[f['t'] for f in x['trace']];axis=x['case']['axis']
        ax.plot(t,[f['truth'][axis]*1000 for f in x['trace']],color=color,lw=3,label=label)
    ax.plot(t,[f['task'][axis]*1000 for f in x['trace']],ls='--',color='#647684',label='Original request')
    ax.set(title=title,xlabel='Time (s)',ylabel='Simulated body travel (mm)');ax.grid(alpha=.18);ax.legend(fontsize=12)
fig.suptitle('Load compensation | Gains and remaining limits',fontsize=23,weight='bold')
fig.supxlabel('Assumed 2 kg, high drag and 4 N/wheel. Development cases, not hardware tests.\nIndependent heavy-load validation: 18/36 → 17/36. Candidate not promoted.',fontsize=12)
fig.savefig(BOARD/'portfolio/load-servo-comparison.png',dpi=150);fig.savefig(BOARD/'portfolio/load-servo-comparison.svg');plt.close(fig)
print(json.dumps({k:summary[k] for k in ('development','validation','regressions')},indent=2))
