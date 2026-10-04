#!/usr/bin/env python3
"""Short independent-input floor experiment. Default is offline, no serial.

After a bounded PI launch, hold the other two motor duties fixed and perturb
one motor by +/-4 PWM. Preserve raw data, failures and the stop transient.
This is identification data, not a chassis-smoothness or control qualification.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time
from c6_loaded_capture import plan as floor_plan, check_frame


def plan(direction='left',motor=1,role='train',moving_range=False):
    if direction not in ('left','right') or motor not in (1,2,3) or role not in ('train','validation'):
        raise ValueError('Invalid experiment')
    p=floor_plan(direction,2)
    pattern=[1,-1,1,-1] if role=='train' else [-1,1,-1,1]
    p.update(stage='loaded_independent_input',motor=motor,role=role,pi_seconds=1.2,
             fixed_baseline_seconds=.12,step_seconds=.16,pwm_perturbations=[4*s for s in pattern],
             maximum_raw_pwm=130,maximum_raw_speed_cps=2200,
             expected_revision='robust-wheels-10-continuous-pi')
    if moving_range:
        # Same bounded independent-input method in the region where loaded
        # wheel tracking has been repeated. No fitted model is assumed here.
        speed_plan=floor_plan(direction,1,.08)
        p.update(wheel_targets_cps=speed_plan['wheel_targets_cps'],body_request_mps=.08,
                 pi_seconds=.8,fixed_baseline_seconds=.12,step_seconds=.12,
                 pwm_perturbations=[8*s for s in pattern],maximum_raw_speed_cps=3500,
                 encoder_excursion_limit_m=.14,expected_revision='robust-wheels-14-coordinated-stop')
    p['maximum_total_command_seconds']=p['pi_seconds']+p['fixed_baseline_seconds']+len(p['pwm_perturbations'])*p['step_seconds']
    return p


def base_duties(frames,targets,now):
    rows=[f for f in frames if now-240<=f['ms']<=now]
    if len(rows)<4:raise RuntimeError('Insufficient launch data')
    duty=[]
    for i,t in enumerate(targets):
        v=[f['motors'][i]['speed']/t for f in rows]
        if abs(statistics.mean(v)-1)>.30 or statistics.pstdev(v)>.25:
            raise RuntimeError(f'Motor {i+1} not sufficiently settled for independent-input measurement')
        u=round(statistics.mean(f['motors'][i]['pwm'] for f in rows))
        if u*t<=0 or not 12<=abs(u)<=126:raise RuntimeError('Baseline duty outside experiment bounds')
        duty.append(u)
    return duty


def perturbed(base,motor,delta):
    out=list(base);out[motor-1]+=delta*(1 if base[motor-1]>0 else -1)
    if any(abs(v)>130 or v*b<=0 for v,b in zip(out,base)):raise RuntimeError('Perturbation outside bounds')
    return out


def check_raw(frame,values,maximum_speed=2200):
    for m,v in zip(frame['motors'],values):
        if not math.isfinite(m['speed']) or abs(m['speed'])>maximum_speed:raise RuntimeError('Raw-phase speed bound')
        if m['speed']*v<0 and abs(m['speed'])>200:raise RuntimeError('Opposite feedback in raw phase')
        if abs(m['pwm'])>130:raise RuntimeError('Raw-phase duty bound')


def capture(client,p,r):
    from c6_drive_sysid import stop_and_wait
    stop_and_wait(client)
    r['initial_status']=client.latest;original=client.latest['controller']
    if client.latest['controller_revision']!=p['expected_revision']:raise RuntimeError('Unexpected firmware')
    if not client.latest['ready'] or client.latest['clients']:raise RuntimeError('Preflight failed')
    if client.latest.get('heading_assist',{}).get('enabled'):raise RuntimeError('Independent-input capture requires heading assist OFF')
    origin=None
    try:
        # Allow boot reconnection to finish while stopped; never bypass a busy radio.
        deadline=time.monotonic()+20;stable=0
        while time.monotonic()<deadline:
            client.pump(.1)
            stable=0 if client.latest['network']['busy'] else stable+1
            if stable>=10:break
        if stable<10:raise RuntimeError('Radio busy; no motion attempted')
        client.request('controller',mode='robust_wheels');client.request('trace',enabled=True);client.pump(.1)
        f=client.latest;od=f['odometry'];age=f.get('telemetry_build_ms',0)
        origin=[od['x_m'],od['y_m'],od['yaw_rad']]
        if (time.monotonic()-client.last_status>.2 or not od['valid'] or od['age_ms']+age>25 or
                f['control_age_ms']+age>40 or f['clients'] or not all(map(math.isfinite,origin))):
            raise RuntimeError('Invalid pre-arm telemetry')
        lease=client.request('arm')['lease'];seq=0

        def stage(op,values,seconds):
            nonlocal seq
            phase=dict(op=op,values=values,seconds=seconds);r['phases'].append(phase)
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                mark=len(client.frames);seq+=1
                ack=client.request(op,lease=lease,seq=seq,values=values)
                phase.setdefault('accepted_ms',ack['command_ms'])
                client.pump(.025)
                if time.monotonic()-client.last_status>.2:raise RuntimeError('Stale telemetry')
                for frame in client.frames[mark:]:
                    r['frames'].append(frame)
                    if frame['ms']>=phase['accepted_ms']:
                        check_frame(frame,lease,origin,p)
                        if op=='pwm':check_raw(frame,values,p['maximum_raw_speed_cps'])

        stage('wheels',p['wheel_targets_cps'],p['pi_seconds'])
        base=base_duties(r['frames'],p['wheel_targets_cps'],client.latest['ms']);r['baseline_pwm']=base
        for d in p['pwm_perturbations']:perturbed(base,p['motor'],d)
        stage('pwm',base,p['fixed_baseline_seconds'])
        for d in p['pwm_perturbations']:
            stage('pwm',perturbed(base,p['motor'],d),p['step_seconds'])
        r['capture_complete']=True
    finally:
        mark=len(client.frames)
        try:stop_and_wait(client)
        finally:
            r['stop_frames']=client.frames[mark:];r['final_status']=client.latest
        client.request('trace',enabled=False);client.request('controller',mode=original)
        client.pump(.1);r['final_status']=client.latest
        if origin is not None:
            od=client.latest['odometry']
            if math.hypot(od['x_m']-origin[0],od['y_m']-origin[1])>p['encoder_excursion_limit_m']:raise RuntimeError('Post-stop travel bound')
            if abs(math.atan2(math.sin(od['yaw_rad']-origin[2]),math.cos(od['yaw_rad']-origin[2])))>.25:
                raise RuntimeError('Post-stop heading bound')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true');ap.add_argument('--floor-confirmed',action='store_true')
    ap.add_argument('--direction',choices=['left','right'],default='left')
    ap.add_argument('--motor',type=int,choices=[1,2,3],default=1)
    ap.add_argument('--role',choices=['train','validation'],default='train')
    ap.add_argument('--moving-range',action='store_true',help='0.08 m/s launch, +/-8 PWM, 1.4 s total request')
    ap.add_argument('--port');ap.add_argument('--report',type=Path)
    a=ap.parse_args();p=plan(a.direction,a.motor,a.role,a.moving_range)
    if not a.execute:print(json.dumps(p,indent=2));return
    if not a.floor_confirmed or not a.port or not a.report:ap.error('Need existing floor authorization, port and report')
    if a.report.exists() or a.report.with_suffix('.jsonl').exists():ap.error('Use a new evidence path')
    from c6_drive_tune import Client
    a.report.parent.mkdir(parents=True,exist_ok=True)
    r=dict(schema='wbb-floor-excitation-1',started_utc=datetime.now(timezone.utc).isoformat(),plan=p,
        floor='wood',frames=[],phases=[],capture_complete=False,body_position_measured=False,
        voltage_measured=False,current_measured=False,calibration_flags_written=False)
    c=None
    with a.report.with_suffix('.jsonl').open('x') as log:
        try:
            c=Client(a.port,log);capture(c,p,r)
        except Exception as e:r.update(capture_complete=False,error=str(e))
        finally:
            if c:
                try:c.close()
                except Exception as e:r.update(capture_complete=False,final_stop_error=str(e))
            a.report.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('frames','stop_frames','initial_status','final_status')},indent=2))
    if not r['capture_complete']:raise SystemExit(1)


if __name__=='__main__':main()
