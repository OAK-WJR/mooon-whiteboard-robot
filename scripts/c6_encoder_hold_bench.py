#!/usr/bin/env python3
"""Bounded encoder-hold test with wheels suspended; not chassis-pose evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import time
from c6_drive_tune import Client
from c6_robust_bench import stopped

ROOT=Path(__file__).resolve().parents[1]
CASES={'forward':[.08,0,0],'backward':[-.08,0,0],
       'left':[0,.08,0],'right':[0,-.08,0],'ccw':[0,0,.6],'cw':[0,0,-.6]}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--suspended',action='store_true');ap.add_argument('--plan',action='store_true')
    ap.add_argument('--case',choices=CASES);ap.add_argument('--port',default='/dev/cu.usbmodem1101')
    ap.add_argument('--hold-seconds',type=float,default=6)
    ap.add_argument('--speed',type=float,default=.08)
    ap.add_argument('--yaw-rate',type=float,default=.6)
    ap.add_argument('--repeat',type=int,default=1)
    ap.add_argument('--report',default='c6_drive/reports/encoder-hold-bench-01.json')
    args=ap.parse_args()
    if not 3<=args.hold_seconds<=10 or not .04<=args.speed<=.08:ap.error('Hold duration must be 3–10 s; released hold speed 0.04–0.08 m/s')
    if not .2<=args.yaw_rate<=.6:ap.error('Released hold yaw rate must be 0.2–0.6 rad/s')
    if not 1<=args.repeat<=3:ap.error('Repeat count must be 1–3')
    cases={name:[v*args.speed/.08 if i<2 else v*args.yaw_rate/.6 for i,v in enumerate(values)]
           for name,values in CASES.items() if not args.case or name==args.case}
    if args.plan:print(json.dumps({'cases':cases,'motion_s':1.3,'hold_s':args.hold_seconds}));return
    if not args.suspended:ap.error('Motion requires --suspended and the user-confirmed powered, secured bench setup')
    path=Path(args.report)
    if path.exists() or path.with_suffix('.jsonl').exists():ap.error('Use a new report path; evidence is not overwritten')
    report={'simulation_only':False,'wheels_suspended':True,'body_position_measured':False,
            'hold_seconds':args.hold_seconds,'speed_mps':args.speed,'yaw_rate':args.yaw_rate,
            'criteria':{'tail_seconds':.7,'position_m':.004,'heading_rad':.05236,'speed_cps':150},
            'results':[],'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in [ROOT/'c6_drive/src/main.cpp',*[ROOT/f'c6_drive/include/{n}.h' for n in ['control','robust_wheels','robust_control','encoder_hold']]]}}
    client=None
    with path.with_suffix('.jsonl').open('x') as log:
      try:
        client=Client(args.port,log);c=client;stopped(c)
        if c.latest.get('hold_revision')!='encoder-hold-6':raise RuntimeError('Hold firmware revision mismatch')
        if c.latest.get('clients'):raise RuntimeError('Wi-Fi client connected; no concurrent controllers during bench')
        c.request('controller',mode='encoder_hold')
        for name,values in list(cases.items())*args.repeat:
            stopped(c);c.request('zero_odometry');c.pump(.12)
            lease=c.request('arm')['lease'];seq=0;frames=[];released=None
            try:
                for phase,command,seconds in [('move',values,1.3),('hold',[0,0,0],args.hold_seconds)]:
                    end=time.monotonic()+seconds
                    while time.monotonic()<end:
                        mark=len(c.frames);seq+=1
                        ack=c.request('drive',lease=lease,seq=seq,values=command)
                        if phase=='hold' and released is None:released=ack['command_ms']
                        c.pump(.04)
                        if time.monotonic()-c.last_status>.20:raise RuntimeError('Telemetry stale')
                        for f in c.frames[mark:]:
                            if not f['armed'] or f['lease']!=lease:raise RuntimeError('Firmware stopped: '+f['reason'])
                            if not f['ready'] or f['control_age_ms']>40:raise RuntimeError('Output/loop fault')
                            if not f['odometry']['valid'] or f['odometry']['error_m']>.04:raise RuntimeError('Odometry correction outside bench bound')
                            frames.append({'phase':phase,'status':f})
                tail=[r['status'] for r in frames if r['phase']=='hold' and r['status']['ms']>released+(args.hold_seconds-.7)*1000]
                if len(tail)<6:raise RuntimeError('Insufficient hold samples')
                max_error=max(f['odometry']['error_m'] for f in tail)
                max_yaw=max(abs(f['odometry']['heading_error_rad']) for f in tail)
                max_speed=max(abs(m['speed']) for f in tail for m in f['motors'])
                quiet=all(f['odometry'].get('within_hold_band') and all(m['pwm']==0 for m in f['motors']) for f in tail)
                moved=[r['status'] for r in frames if r['phase']=='move'][-1]['odometry']
                axis=next(i for i,v in enumerate(values) if v)
                progress=moved[['x_m','y_m','yaw_rad'][axis]]*(1 if values[axis]>0 else -1)
                passed=quiet and max_error<.004 and max_yaw<.05236 and max_speed<150 and progress>(.2 if axis==2 else .04)
                result={'name':name,'values':values,'release_ms':released,'hold_tail_max_error_m':max_error,
                        'hold_tail_max_heading_error_rad':max_yaw,'hold_tail_max_speed_cps':max_speed,
                        'movement_progress':progress,'quiet_zero_output':quiet,'passed':passed,'frames':frames}
                report['results'].append(result)
                print(json.dumps({k:v for k,v in result.items() if k!='frames'}),flush=True)
                if not passed:raise RuntimeError(f'{name}: hold acceptance failed')
            finally:stopped(c)
        # A hold heartbeat must expire; expired hold is never re-armed automatically.
        lease=c.request('arm')['lease'];ack=c.request('drive',lease=lease,seq=1,values=[0,0,0])
        c.pump(.45)
        assert not c.latest['armed'] and c.latest['reason']=='LINK_TIMEOUT'
        assert all(m['pwm']==m['fi_duty']==m['bi_duty']==0 for m in c.latest['motors'])
        report['watchdog']={'last_command_ms':ack['command_ms'],'status':c.latest}
        report['passed']=True
      except Exception as e:
        report['error']=str(e);report['passed']=False
        print('FAILED:',e,flush=True)
      finally:
        if client:
            try:
                stopped(client);client.request('controller',mode='robust_wheels');client.pump(.1)
                report['final']=client.latest
            finally:client.close()
        path.write_text(json.dumps(report,indent=2)+'\n')
    if not report.get('passed'):raise SystemExit(1)


if __name__=='__main__':main()
