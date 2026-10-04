#!/usr/bin/env python3
"""Offline moving-range floor model screening. Never opens a hardware transport.

Freeze fits with --train before collecting independent validation. The report
loads those frozen fits; it does not refit from validation data or write gains.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from c6_drive_sysid import fit_model,simulate,metrics,delta32


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def extract(path):
    r=json.loads(path.read_text())
    if not r.get('capture_complete'):raise ValueError('Incomplete capture: '+str(r.get('error')))
    f=r['final_status']
    if f['armed'] or any(m['pwm'] or abs(m['speed'])>=30 for m in f['motors']):raise ValueError('Not stopped')
    motor=r['plan']['motor']-1;sign=1 if r['baseline_pwm'][motor]>0 else -1
    start=r['phases'][1]['accepted_ms']+20 # exclude the one-control-tick raw handover
    rows={}
    for f in r['frames']:
        for row in f.get('control_samples',[]):
            if row[0]>=start:
                if row[0] in rows and rows[row[0]]!=row:raise ValueError('Conflicting samples')
                rows[row[0]]=row
    data=np.array([rows[k] for k in sorted(rows)],dtype=float)
    if len(data)<40 or data.shape[1]!=11 or not np.isfinite(data).all():raise ValueError('Insufficient finite raw-phase samples')
    t=(data[:,0]-data[0,0])/1000;dt=np.diff(t)
    if np.min(dt)<=0 or np.max(dt)>.025:raise ValueError('Control trace gap')
    for i in range(3):
        if i!=motor and np.any(data[:,8+i]!=r['baseline_pwm'][i]):raise ValueError('Other input not fixed')
    u=sign*data[:,8+motor];y=sign*data[:,2+motor]
    if np.min(u)<=0 or np.max(u)>130 or np.ptp(u)<16 or np.count_nonzero(np.diff(u))<4:raise ValueError('Input not sufficiently varied')
    if np.min(y)<100:raise ValueError('Target wheel left the continuously moving range')
    frames=[f for f in r['frames'] if f['ms']>=start]
    first,last=(f['motors'][motor] for f in (frames[0],frames[-1]))
    efficiency=abs(delta32(last['count'],first['count']))/max(1,(last['edges']-first['edges'])&0xffffffff)
    if not .8<=efficiency<=1.05:raise ValueError('Poor quadrature efficiency')
    quality=dict(samples=len(t),median_dt_s=float(np.median(dt)),maximum_gap_s=float(np.max(dt)),
                 quadrature_efficiency=efficiency,pwm_span=float(np.ptp(u)),speed_span_cps=float(np.ptp(y)),
                 raw_duration_s=float(t[-1]))
    return r,t,u,y,quality,data


def train(folder):
    target=folder/'frozen-models.json'
    if target.exists():raise ValueError('Training model already frozen; use a new experiment')
    models=[]
    for p in sorted(folder.glob('[0-9]*.json')):
        raw=json.loads(p.read_text());i=raw['plan']['motor']-1
        item=dict(source=p.name,source_sha256=digest(p),raw_sha256=digest(p.with_suffix('.jsonl')),
                  motor=i+1,direction=raw['plan']['direction'],role=raw['plan']['role'])
        try:
            r,t,u,y,q,_=extract(p)
            if r['plan']['role']!='train':raise ValueError('Not training data')
            item.update(model=fit_model(t,u,y),quality=q,status='FIT_ONLY_NOT_VALIDATED')
        except ValueError as e:item.update(status='REJECTED_TRAINING_DATA',error=str(e))
        models.append(item)
    target.write_text(json.dumps(dict(models=models,simulation_only=False,gains_written=False,
        method='Positive-gain first-order PWM-to-filtered-speed model; fixed delay/tau search from the existing sysid tool',
        limitations=['Short single-input perturbations do not identify a full coupled chassis model.',
                     'No voltage/current, independent chassis motion, mass or slip measurement.',
                     'The fitted bias is moving-range friction/load, not breakaway duty.']),indent=2)+'\n')
    print(json.dumps([{k:v for k,v in m.items() if k in ('motor','direction','status','error','model')} for m in models],indent=2))


def report(training,validation,out):
    frozen=json.loads((training/'frozen-models.json').read_text())
    manifest=json.loads((validation/'manifest.json').read_text());criteria=manifest['criteria'];results=[]
    for m in frozen['models']:
        path=training/m['source']
        if digest(path)!=m['source_sha256'] or digest(path.with_suffix('.jsonl'))!=m['raw_sha256']:raise ValueError('Training source changed')
        candidates=[p for p in validation.glob('[0-9]*.json') if (lambda r:r['plan']['motor']==m['motor'] and r['plan']['direction']==m['direction'])(json.loads(p.read_text()))]
        item=dict(motor=m['motor'],direction=m['direction'],training_source=str(path),passed=False)
        try:
            if len(candidates)!=1:raise ValueError('Need exactly one independent validation trial')
            vp=candidates[0];r,t,u,y,q,_=extract(vp)
            if r['plan']['role']!='validation':raise ValueError('Wrong validation role')
            if 'model' not in m:raise ValueError('No accepted training data')
            predicted=simulate(t,u,y[0],m['model']);v=metrics(y,predicted)
            resolved=(criteria['minimum_tau_samples']*max(q['median_dt_s'],m['quality']['median_dt_s'])<=m['model']['tau_s']<criteria['maximum_tau_s'] and m['model']['delay_s']<criteria['maximum_delay_s'])
            item.update(validation_source=str(vp),source_sha256=digest(vp),raw_sha256=digest(vp.with_suffix('.jsonl')),
                        validation=v,quality=q,model=m['model'],dynamics_resolved=resolved,
                        passed=resolved and v['r_squared']>=criteria['minimum_validation_r_squared'] and v['rmse_fraction_of_speed_span']<=criteria['maximum_validation_rmse_fraction_of_span'])
        except ValueError as e:item['error']=str(e)
        results.append(item)
    result=dict(results=results,all_models_qualified=len(results)==6 and all(r['passed'] for r in results),
                validation_attempt_count=len(list(validation.glob('[0-9]*.json'))),
                validation_completed_count=sum('validation' in r for r in results),
                frozen_training_sha256=digest(training/'frozen-models.json'),gains_written=False,body_position_measured=False)
    (validation/'model-validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));draw(training,validation,out,frozen,result)


def draw(training,validation,out,frozen,result):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out.mkdir(parents=True,exist_ok=True);figures=[]
    for m,v in zip(frozen['models'],result['results']):
        if 'model' not in m:continue
        validated='validation' in v
        source=Path(v['validation_source']) if validated else training/m['source']
        r,t,u,y,q,data=extract(source);prediction=simulate(t,u,y[0],m['model'])
        fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True,layout='constrained')
        axes[0].plot(t,y,label='Measured '+('validation' if validated else 'training')+' speed',lw=2.2,color='#008a70')
        axes[0].plot(t,prediction,label='Frozen model prediction' if validated else 'Training fit (not independent)',lw=2,color='#b64f59')
        axes[0].set_ylabel('Speed magnitude (counts/s)');axes[0].legend();axes[0].grid(alpha=.2)
        axes[1].step(t,u,where='post',lw=2,color='#2767a0');axes[1].set(ylabel='Applied PWM magnitude / 255',xlabel='Time in raw input phase (s)');axes[1].grid(alpha=.2)
        status=('Independent validation: '+('PASS' if v['passed'] else 'FAIL')) if validated else 'Training fit only | validation incomplete'
        score=v['validation'] if validated else m['model']['train']
        fig.suptitle(f"WhiteBoardBot | Motor {m['motor']} · {m['direction'].title()}\n{status}",fontsize=18,weight='bold')
        fig.supxlabel(f"{'Validation' if validated else 'Training'} R² {score['r_squared']:.2f} · RMSE {score['rmse_cps']:.0f} counts/s\nReal wood-floor data. Other motor duties fixed. No chassis-motion or voltage measurement.",fontsize=11)
        name=f"motor-{m['motor']}-{m['direction']}.png";fig.savefig(out/name,dpi=150);plt.close(fig);figures.append((m['motor'],m['direction'],name))
    links=''.join(f'<option value="{name}">Motor {i} · {d}</option>' for i,d,name in figures)
    first=figures[0][2] if figures else ''
    page=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Floor model validation</title><style>body{{max-width:1100px;margin:30px auto;padding:0 20px;font:18px/1.6 system-ui;color:#183245;background:#f7fafb}}img{{width:100%}}select{{font:inherit;padding:8px}}a{{color:#086e84}}</style><h1>Real floor model validation</h1><p>Completed independent validation: {result['validation_completed_count']}/6, from {result['validation_attempt_count']} attempted captures. Qualified models: {sum(x['passed'] for x in result['results'])}. Missing or aborted validation is not counted as a completed test.</p><p>No gains were written. This is model testing, not a smoothness or wall-hold qualification. Plots without completed validation explicitly show the training fit only.</p><p><a href="../">Back to all floor results</a></p><select id="pick">{links}</select><a id="full" href="{first}"><img id="plot" src="{first}" alt="Motor model screening, training or independent validation as labelled"></a><script>document.getElementById('pick').onchange=e=>{{document.getElementById('plot').src=e.target.value;document.getElementById('full').href=e.target.value;}};</script></html>'''
    (out/'index.html').write_text(page)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('training',type=Path);ap.add_argument('--train',action='store_true');ap.add_argument('--validation',type=Path);ap.add_argument('--portfolio',type=Path)
    a=ap.parse_args()
    if a.train:train(a.training)
    elif a.validation and a.portfolio:report(a.training,a.validation,a.portfolio)
    else:ap.error('Choose --train, or --validation and --portfolio')


if __name__=='__main__':main()
