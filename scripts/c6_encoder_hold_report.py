#!/usr/bin/env python3
"""English portfolio figure from actual, suspended encoder-hold trials."""
import hashlib
import json
from pathlib import Path
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
reports=ROOT/'c6_drive/reports'
names=['encoder-hold-r6-six-directions','encoder-hold-r4-default-speed']
records=[json.loads((reports/(name+'.json')).read_text()) for name in names]
for record in records[:1]: # Final build must match; the retained failure is an archived earlier build.
    for path,digest in record['source_sha256'].items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==digest, 'Firmware changed: '+path

summary={'simulation_only':False,'body_position_measured':False,'suspended':True,
         'revision':'encoder-hold-6','released_envelope_passed':records[0].get('passed',False),
         'higher_speed_prototype_passed':records[1].get('passed',False),'trials':[],
         'limitations':['Position and heading are wheel-odometry estimates, not measured chassis pose.',
          'Wheels suspended: no loaded surface, slip, thermal or wall qualification.',
          'Static-friction corrections can take seconds to settle; see settling times.',
          'Startup/control parameters were developed using these tests; not independent prospective validation.'],
         'retained_failure_reports':['encoder-hold-smoke-01.json','encoder-hold-smoke-02.json',
                                    'encoder-hold-six-directions-01.json','encoder-hold-six-directions-02.json',
                                    'encoder-hold-r4-default-speed.json','encoder-hold-r5-six-directions.json']}
for index,record in enumerate(records):
    for trial in record['results']:
        frames=[r['status'] for r in trial['frames'] if r['phase']=='hold' and r['status']['ms']>=trial['release_ms']]
        def settled(f):
            return f['odometry'].get('within_hold_band') and all(m['pwm']==0 and abs(m['speed'])<150 for m in f['motors'])
        bad=[i for i,f in enumerate(frames) if not settled(f)]
        start=bad[-1]+1 if bad else 0
        settle_s=(frames[start]['ms']-trial['release_ms'])/1000 if start<len(frames) else None
        summary['trials'].append({k:v for k,v in trial.items() if k!='frames'}|
            {'speed_mps':record['speed_mps'],'yaw_rate':record.get('yaw_rate',.6),'settled_after_release_s':settle_s,
             'released_envelope':index==0})

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':14,'svg.fonttype':'none'})
fig,axes=plt.subplots(2,1,figsize=(10,9),layout='constrained',sharex=True)
for index,(ax,record) in enumerate(zip(axes,records)):
    for trial,color in zip(record['results'],['#2674b6','#cf652b','#008a70','#af509a','#b08809','#635ab6']):
        frames=[r['status'] for r in trial['frames'] if r['phase']=='hold' and r['status']['ms']>=trial['release_ms']]
        ax.plot([(f['ms']-trial['release_ms'])/1000 for f in frames],[f['odometry']['error_m']*1000 for f in frames],label=trial['name'],color=color,lw=2)
    ax.axhspan(0,4,color='#008a70',alpha=.1);ax.axhline(4,color='#5d766f',ls='--',lw=1)
    ax.set_title(('Released limit: ' if index==0 else 'FAILED prototype; this speed is disabled: ')+f"{record['speed_mps']:.2f} m/s",loc='left',fontsize=15)
    ax.set(xlim=(0,6),ylabel='Odometry target error (mm)');ax.set_ylim(bottom=0)
    ax.spines[['top','right']].set_visible(False);ax.grid(alpha=.15)
axes[0].legend(ncol=3,fontsize=11)
axes[-1].set_xlabel('Seconds after key-release command')
fig.suptitle('Encoder-relative hold | Real hardware, wheels suspended',fontsize=19,weight='bold')
fig.supxlabel('Green band = 4 mm encoder-estimated tolerance. Body position was not measured.\nCorrection can take seconds; these trials do not establish wall holding.',fontsize=12)
for ext in ['png','svg']:fig.savefig(ROOT/f'c6_drive/portfolio/encoder-hold-response.{ext}',dpi=145)
plt.close(fig)
(reports/'encoder-hold-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
lines=['# Encoder-relative hold: suspended hardware','',
       '**These are actual encoder measurements, not chassis-position accuracy or wall-hold evidence.**','',
       '| Command | Translation / yaw | Settled after release | Final-window position error | Heading error | Pass |',
       '|---|---|---:|---:|---:|---|']
for r in summary['trials']:
    settled=f"{r['settled_after_release_s']:.2f} s" if r['settled_after_release_s'] is not None else 'not settled'
    lines.append(f"| {r['name']} | {r['speed_mps']:.2f} m/s / {r['yaw_rate']:.1f} rad/s | {settled} | {1000*r['hold_tail_max_error_m']:.2f} mm | {math.degrees(r['hold_tail_max_heading_error_rad']):.2f} deg | {r['passed']} |")
lines+=['','Settled time is the first retained sample after the last non-quiet sample, with zero PWM and each filtered wheel speed below 150 counts/s. Acceptance additionally requires the last 0.7 s of a six-second hold to remain inside 4 mm / 3 degrees of encoder-derived error.','',
        '[English figure](../portfolio/encoder-hold-response.png) · [Controller details and limitations](../ENCODER_HOLD.md)','',
        'The earlier missing-breakaway and late-correction failures remain in these reports:','']
lines += [f'- [{n}]({n})' for n in summary['retained_failure_reports']]
(reports/'encoder-hold-summary.md').write_text('\n'.join(lines)+'\n')
print(json.dumps({'released_envelope_passed':summary['released_envelope_passed'],'trials_including_failure':len(summary['trials']),
      'worst_settle_s':max((r['settled_after_release_s'] or 0) for r in summary['trials']),
      'max_encoder_error_mm':max(r['hold_tail_max_error_m']*1000 for r in summary['trials'])},indent=2))
