#!/usr/bin/env python3
"""Bounded suspended-wheel comparison; records board-clock 100 Hz response.

No serial access with --plan. Motion requires the explicit --suspended flag.
Always restores the legacy controller and stops; no gain/NVS writes or automatic
promotion. This measures wheel dynamics, not ground tracking or wall hold.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from c6_drive_tune import Client

ROOT=Path(__file__).resolve().parents[1]


def plan(smoke=False):
    tests=[]
    for i in range(3):
        for sign in ([1] if smoke else [1,-1]):
            v=[0,0,0];v[i]=sign*(1800 if smoke else 3000)
            tests.append(dict(name=f'motor-{i+1}-{sign:+d}',values=v,seconds=1.2 if smoke else 2.2))
    if not smoke:
        for name,v in [('forward',[-2600,0,2600]),('lateral',[1500,-3000,1500]),('same-speed',[3000,3000,3000])]:
            tests.append(dict(name=name,values=v,seconds=2.2))
    return tests


def signed_delta(after,before):
    return ((after-before+0x80000000)&0xffffffff)-0x80000000


def analyze(test,samples,frames,command_ms):
    # Compact rows: control timestamp, command timestamp, 3 measured speeds,
    # 3 ramp references, 3 actual LEDC duties. Deduplicate overlapping packets.
    samples=sorted({int(s[0]):s for s in samples if command_ms<=s[0]<=command_ms+test['seconds']*1000}.values())
    if len(samples)<30:raise RuntimeError('Too few board-clock samples')
    rows=np.array(samples,dtype=float);t=(rows[:,0]-command_ms)/1000
    if max(np.diff(t))>.04:raise RuntimeError('Control trace gap exceeds 40 ms')
    result={'name':test['name'],'values':test['values'],'command_ms':command_ms,
            'samples':samples,'sample_count':len(samples),'max_gap_ms':float(max(np.diff(t))*1000),'motors':[]}
    for i,target in enumerate(test['values']):
        speed=rows[:,2+i];pwm=rows[:,8+i]
        if not target:
            if np.max(np.abs(speed))>80 or np.max(np.abs(pwm))>0:raise RuntimeError(f'Uncommanded motor {i+1}')
            continue
        sign=1 if target>0 else -1;normalized=speed/target
        crossings={}
        for fraction in [.1,.5,.9]:
            indices=np.where(normalized>=fraction)[0]
            crossings[f't{int(fraction*100)}_s']=float(t[indices[0]]) if len(indices) else None
        within=np.abs(normalized-1)<.10;settle=None
        for n in range(len(t)):
            window=(t>=t[n])&(t<=t[n]+.2)
            if t[-1]-t[n]>=.2 and np.all(within[window]):settle=float(t[n]);break
        tail=speed[t>=max(.6,test['seconds']-.6)]
        active=[f['motors'][i] for f in frames if f['armed']]
        first,last=active[0],active[-1]
        edges=(last['edges']-first['edges'])&0xffffffff
        travel=abs(signed_delta(last['count'],first['count']))
        efficiency=travel/edges if edges else 0
        r=dict(motor=i+1,target=target,mean_cps=float(np.mean(tail)),std_cps=float(np.std(tail)),
               relative_error=float(abs(np.mean(tail)-target)/abs(target)),
               peak_pwm=float(np.max(np.abs(pwm))),overshoot_fraction=float(max(0,np.max(normalized)-1)),
               settle_10percent_for_200ms_s=settle,quadrature_efficiency=efficiency,**crossings)
        r['tracking_pass']=r['relative_error']<.10 and r['std_cps']<abs(target)*.20 and efficiency>.85 and r['t90_s'] is not None
        result['motors'].append(r)
    times=[m['t90_s'] for m in result['motors']]
    result['t90_spread_s']=max(times)-min(times) if all(x is not None for x in times) else None
    result['tracking_pass']=all(m['tracking_pass'] for m in result['motors'])
    return result


def stopped(c):
    c.request('stop');deadline=time.monotonic()+2.5
    while time.monotonic()<deadline:
        c.pump(.08);s=c.latest
        if not s['ready']:raise RuntimeError('Output hardware fault: '+s['reason'])
        if not s['armed'] and all(abs(m['speed'])<30 and m['pwm']==0 and
                m['fi_duty']==m['bi_duty']==m['fi_level']==m['bi_level']==0 for m in s['motors']):return
    raise RuntimeError('Stopped output/encoder state not verified')


def trial(c,test):
    stopped(c);c.pump(.3)
    before=[m['count'] for m in c.latest['motors']]
    lease=c.request('arm')['lease'];seq=0;frames=[];samples=[];accepted=None
    end=time.monotonic()+test['seconds']
    try:
        while time.monotonic()<end:
            mark=len(c.frames)
            ack=c.request('wheels',lease=lease,seq=seq+1,values=test['values']);seq+=1
            if accepted is None:accepted=ack['command_ms']
            c.pump(.045)
            if time.monotonic()-c.last_status>.20:raise RuntimeError('Telemetry stale')
            for f in c.frames[mark:]:
                if f['ms']<accepted:continue
                if not f['armed'] or f['lease']!=lease:raise RuntimeError('Firmware stopped: '+f['reason'])
                if not f['ready'] or f['control_age_ms']>40:raise RuntimeError('Hardware/loop fault')
                frames.append(f);samples.extend(f.get('control_samples',[]))
                for i,value in enumerate(test['values']):
                    if value==0 and abs(signed_delta(f['motors'][i]['count'],before[i]))>10:
                        raise RuntimeError(f'Uncommanded motion on motor {i+1}')
    finally:stopped(c)
    r=analyze(test,samples,frames,accepted);r['stopped']=c.latest
    return r


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--plan',action='store_true');ap.add_argument('--suspended',action='store_true')
    ap.add_argument('--smoke',action='store_true')
    ap.add_argument('--controller',choices=['legacy','robust_wheels','both'],default='both')
    ap.add_argument('--case',help='Run one named fixture from the plan')
    ap.add_argument('--repeat',type=int,default=1)
    ap.add_argument('--reverse',action='store_true',help='Invert all fixture wheel targets')
    ap.add_argument('--port',default='/dev/cu.usbmodem1101')
    ap.add_argument('--report',type=Path,default=ROOT/'c6_drive/reports/robust-hardware-bench-01.json')
    args=ap.parse_args();tests=plan(args.smoke)
    if args.case:tests=[t for t in tests if t['name']==args.case]
    if not tests:ap.error('Unknown fixture name')
    if not 1<=args.repeat<=5:ap.error('--repeat must be between 1 and 5')
    if args.reverse:
        for test in tests:test['values']=[-v for v in test['values']];test['name']+='-reverse'
    if args.plan:print(json.dumps(tests,indent=2));return
    if not args.suspended:ap.error('Confirm the robot is secured off the board with wheels suspended; then use --suspended')
    if args.report.exists() or args.report.with_suffix('.jsonl').exists():ap.error('Report/log already exists; choose a new path')
    args.report.parent.mkdir(parents=True,exist_ok=True)
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'suspended':True,'simulation_only':False,
            'units':'encoder counts/s; board-clock seconds','results':[],
            'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                [ROOT/'c6_drive/include/robust_control.h',ROOT/'c6_drive/include/robust_wheels.h',ROOT/'c6_drive/include/control.h',ROOT/'c6_drive/src/main.cpp']}}
    c=None
    def save():args.report.write_text(json.dumps(report,indent=2)+'\n')
    with args.report.with_suffix('.jsonl').open('x') as log:
        try:
            c=Client(args.port,log);report['initial']=c.latest
            if c.latest.get('controller_revision')!='robust-wheels-4':raise RuntimeError('Integrated test firmware is not installed')
            if c.latest.get('clients',0):raise RuntimeError('Disconnect the browser/other Wi-Fi controller before the bench comparison')
            stopped(c);c.request('trace',enabled=True);c.pump(.2)
            modes=['legacy','robust_wheels'] if args.controller=='both' else [args.controller]
            for repetition in range(args.repeat):
                # Alternate order across repetitions to reveal warm-up/order effects.
                for mode in (modes if repetition%2==0 else list(reversed(modes))):
                    stopped(c);c.request('controller',mode=mode);c.pump(.1)
                    if c.latest['controller']!=mode:raise RuntimeError('Controller selection not confirmed')
                    for test in tests:
                        r=trial(c,test);r.update(controller=mode,repetition=repetition);report['results'].append(r);save()
                        brief={k:v for k,v in r.items() if k not in ['samples','stopped']}
                        print(json.dumps(brief),flush=True)
            report['passed']=all(r['tracking_pass'] for r in report['results'])
        except BaseException as e:
            report['error']=str(e);save();raise
        finally:
            if c:
                try:
                    stopped(c);c.request('trace',enabled=False);c.request('controller',mode='legacy');c.pump(.1)
                    report['final']=c.latest
                finally:c.close()
            save()
    print('Saved',args.report,flush=True)


if __name__=='__main__':main()
