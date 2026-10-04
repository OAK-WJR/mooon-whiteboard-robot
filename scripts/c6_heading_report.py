#!/usr/bin/env python3
"""Offline encoder-relative heading assessment of same-firmware floor A/B trials."""
import argparse
import json
import math
from pathlib import Path
from c6_floor_response_report import summarize


def metrics(path):
    raw=json.loads(path.read_text());r=summarize(path)
    if r.get('motion_started') is False:return r
    origin=raw['experiment_status']['odometry'];x,y,a=origin['x_m'],origin['y_m'],origin['yaw_rad']
    vectors={'right':(0,-1),'left':(0,1),'forward':(1,0),'backward':(-1,0)}
    ux,uy=vectors[raw['plan']['direction']]
    frames=raw['frames']+raw['stop_frames']+[raw['final_status']]
    frames=sorted({f['ms']:f for f in frames if f['ms']>=raw['command_ms']}.values(),key=lambda f:f['ms'])
    samples=[]
    for f in frames:
        od=f['odometry'];dx,dy=od['x_m']-x,od['y_m']-y
        bx=math.cos(a)*dx+math.sin(a)*dy;by=-math.sin(a)*dx+math.cos(a)*dy
        yaw=math.atan2(math.sin(od['yaw_rad']-a),math.cos(od['yaw_rad']-a))*180/math.pi
        samples.append([(f['ms']-raw['command_ms'])/1000,yaw,(bx*ux+by*uy)*1000,(-bx*uy+by*ux)*1000])
    out={k:r[k] for k in ('source','source_sha256','raw_sha256','controller_revision','capture_complete',
                          'capture_error','motion_quality_passed','stop_settle_ms','wheels')}
    release=(raw.get('release_command_ms',raw['command_ms']+1000)-raw['command_ms'])/1000
    before_release=[s for s in samples if s[0]<release]
    out.update(direction=raw['plan']['direction'],assist=raw['plan']['heading_assist'],
        direct_heading=raw['plan'].get('direct_heading',False),
        encoder_startup_peak_heading_deg=max(abs(s[1]) for s in samples if s[0]<=.6),
        encoder_release_heading_deg=before_release[-1][1] if before_release else None,
        encoder_stop_heading_change_deg=samples[-1][1]-before_release[-1][1] if before_release else None,
        encoder_final_heading_deg=samples[-1][1],encoder_peak_heading_deg=max(abs(s[1]) for s in samples),
        encoder_final_progress_mm=samples[-1][2],encoder_peak_cross_track_mm=max(abs(s[3]) for s in samples),
        encoder_samples=samples,body_position_measured=False)
    return out


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('directory',type=Path)
    ap.add_argument('--portfolio',type=Path);a=ap.parse_args()
    m=json.loads((a.directory/'manifest.json').read_text());criteria=m['criteria']
    variant='direct_heading' if 'direct_heading' in m else 'assist'
    trials=[metrics(p) for p in sorted(a.directory.glob('[0-9]*.json')) if not p.name.endswith('.annotation.json')]
    baselines={t['direction']:t for t in trials if t.get(variant) is False}
    for direction,relative in m.get('baseline_sources',{}).items():
        if direction in baselines:raise ValueError('Ambiguous baseline for '+direction)
        baseline=metrics((a.directory/relative).resolve())
        if baseline.get(variant) is not False or baseline.get('direction')!=direction:
            raise ValueError('Mismatched reference baseline')
        baselines[direction]=baseline
    for t in trials:
        if t.get(variant) is True:
            baseline=baselines.get(t['direction'])
            t['baseline_source']=baseline['source'] if baseline else None
            t['heading_criteria_passed']=bool(t['motion_quality_passed'] and baseline and
                abs(t['encoder_final_heading_deg'])<=criteria['candidate_final_encoder_heading_deg'] and
                t['encoder_peak_heading_deg']<=criteria['candidate_peak_encoder_heading_deg'] and
                t['encoder_peak_heading_deg']<=baseline['encoder_peak_heading_deg']+criteria['no_worse_than_paired_baseline_peak_by_deg'])
    result=dict(trials=trials,body_position_measured=False,calibration_flags_written=False,
                complete=len(trials)==len(m['directions']))
    candidates=[t for t in trials if t.get(variant) is True]
    if variant=='direct_heading' and candidates and len(baselines)==2:
        mean=lambda rows,key,absolute=False:sum(abs(t[key]) if absolute else t[key] for t in rows)/len(rows)
        b=list(baselines.values())
        stats=dict(baseline_mean_peak_deg=mean(b,'encoder_peak_heading_deg'),candidate_mean_peak_deg=mean(candidates,'encoder_peak_heading_deg'),
                   baseline_mean_final_deg=mean(b,'encoder_final_heading_deg',True),candidate_mean_final_deg=mean(candidates,'encoder_final_heading_deg',True))
        result['comparison']=stats
        result['initial_promotion_criteria_passed']=bool(result['complete'] and all(t.get('heading_criteria_passed') for t in candidates) and
            stats['candidate_mean_peak_deg']<stats['baseline_mean_peak_deg'] and
            stats['candidate_mean_final_deg']<=stats['baseline_mean_final_deg']+criteria['promotion_max_mean_final_regression_deg'])
    (a.directory/'heading-results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps([dict(source=Path(t['source']).name,**{k:v for k,v in t.items() if k in
        ('assist','motion_quality_passed','encoder_final_heading_deg','encoder_peak_heading_deg',
         'encoder_final_progress_mm','encoder_peak_cross_track_mm','heading_criteria_passed')}) for t in trials],indent=2))
    if a.portfolio:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        a.portfolio.mkdir(parents=True,exist_ok=True)
        directions=list(dict.fromkeys(t['direction'] for t in trials if 'direction' in t))
        fig,axes=plt.subplots(len(directions),1,figsize=(10,3.1*len(directions)+1.3),sharex=True,layout='constrained',squeeze=False)
        for direction,ax in zip(directions,axes[:,0]):
            baseline=baselines.get(direction)
            if baseline and not any(t['source']==baseline['source'] for t in trials):
                samples=baseline['encoder_samples']
                ax.plot([s[0] for s in samples],[s[1] for s in samples],lw=2,color='#7a718c',ls='--',label='Earlier OFF reference')
            for t in trials:
                if t.get('direction')!=direction:continue
                samples=t['encoder_samples'];on=t[variant]
                label=('Direct feedback' if on else 'Filtered feedback') if variant=='direct_heading' else (
                    ('Assist + deceleration ON' if m.get('controlled_release') else 'Heading assist ON') if on else 'Assist OFF')
                ax.plot([s[0] for s in samples],[s[1] for s in samples],lw=2.5,
                        color='#008a70' if on else '#7a718c',label=label)
            ax.axhline(0,color='#566475',ls='--',lw=1);ax.axvline(1,color='#999',ls=':',lw=1)
            ax.set(title=direction.title()+' | 0.08 m/s, 1 s request',ylabel='Encoder heading change (deg)')
            ax.grid(alpha=.2);ax.legend();ax.spines[['top','right']].set_visible(False)
        axes[-1,0].set_xlabel('Time after accepted command (s)')
        title='Heading feedback placement' if variant=='direct_heading' else ('Coordinated normal stop' if m.get('controlled_release') else 'Moving heading correction')
        fig.suptitle('WhiteBoardBot | '+title+'\nSame firmware, unchanged inner wheel PI',fontsize=17,weight='bold')
        fig.supxlabel('Real wood-floor wheel odometry. Includes stopping coast.\nEncoder heading is not independent chassis heading; isolated trials are exploratory.',fontsize=10)
        fig.savefig(a.portfolio/'heading-comparison.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
