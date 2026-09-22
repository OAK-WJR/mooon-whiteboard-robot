#!/usr/bin/env python3
"""Summarize the bounded startup experiment without changing baseline evidence."""
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1];BOARD=ROOT/'c6_drive'
source=BOARD/'reports/startup-ramp-dev01.json';report=json.loads(source.read_text())
for name,digest in report['source_sha256'].items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
results=report['results']
pair=[next(x for x in results if x['family']=='load' and x['case']['name']=='2kg_high_drag_y_0.04' and x['variant']==v)
      for v in ('fixed','ramp')]
summary=dict(scope=report['scope'],mass_kg=None,reported_symptom='wheel_start_difficult_and_slow_under_load',
             disposition='Experimental only; not promoted or flashed',
             source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
             load={v:dict(n=54,passed=sum(x['metrics']['within_motion_bounds'] for x in results if x['family']=='load' and x['variant']==v),
                          faulted=sum(bool(x['metrics']['faults']) for x in results if x['family']=='load' and x['variant']==v)) for v in ('fixed','ramp')},
             example=[dict(case=x['case'],variant=x['variant'],metrics=x['metrics']) for x in pair],
             warnings=sum(x['metrics']['solver_warnings'] for x in results),
             post_fault_nonzero=sum(x['metrics']['post_fault_nonzero'] for x in results))
(BOARD/'reports/startup-ramp-summary.json').write_text(json.dumps(summary,indent=2))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':14,'svg.fonttype':'none'})
fig,axes=plt.subplots(2,1,figsize=(10,8),layout='constrained')
for x,color,title in zip(pair,['#a45a32','#007e79'],['Fixed breakaway pulse','Rising pulse, same deadline']):
    frames=x['trace'];t=[f['t'] for f in frames]
    axes[0].plot(t,[f['truth'][1]*1000 for f in frames],lw=2.8,color=color,label=title)
    axes[1].plot(t,[max(abs(v) for v in f['pwm']) for f in frames],lw=2,color=color,label=title)
axes[0].plot(t,[f['task'][1]*1000 for f in frames],ls='--',lw=1.8,color='#617586',label='Original request')
axes[0].set(ylabel='Simulated body travel (mm)',title='Starts moving, but still falls short of the requested travel')
axes[0].legend(loc='upper left',fontsize=12)
axes[1].set(xlabel='Time (s)',ylabel='Highest |PWM| of the 3 wheels',ylim=(-5,265))
for ax in axes:ax.grid(alpha=.18);ax.set_xlim(0,8)
fig.suptitle('Loaded startup | Development experiment',fontsize=22,weight='bold')
fig.supxlabel('Assumed 2 kg, high drag, Y travel at 0.04 m/s. Same 250 ms recovery deadline.\nAll 54 load cases: 29 → 30 pass; high-drag group still fails. Not hardware validated.',fontsize=12)
fig.savefig(BOARD/'portfolio/startup-ramp.png',dpi=150);fig.savefig(BOARD/'portfolio/startup-ramp.svg');plt.close(fig)
print(json.dumps(summary['load'],indent=2))
