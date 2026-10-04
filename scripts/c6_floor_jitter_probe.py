#!/usr/bin/env python3
"""Bounded PI-versus-fixed-duty floor diagnostic; defaults to an offline plan.

Execution needs existing explicit ground/clear-space authorization. Fixed duty
retains MCU stop, feedback and lease protection. The host additionally checks
direction during the raw phase (the MCU reference-direction guard is PI-only).
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import time
from datetime import datetime,timezone
from c6_loaded_capture import plan,check_frame


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true');ap.add_argument('--floor-confirmed',action='store_true')
    ap.add_argument('--port');ap.add_argument('--report',type=Path)
    args=ap.parse_args()
    experiment=plan('left',2)
    experiment.update(stage='pi_then_fixed_duty_diagnostic',pi_seconds=1.2,fixed_seconds=.7,
                      maximum_fixed_pwm=130,expected_revision='robust-wheels-10-continuous-pi')
    if not args.execute:print(json.dumps(experiment,indent=2));return
    if not args.floor_confirmed or not args.port or not args.report:ap.error('Need current floor authorization, port and report')
    if args.report.exists() or args.report.with_suffix('.jsonl').exists():ap.error('Use a new evidence path')
    from c6_drive_tune import Client
    from c6_drive_sysid import stop_and_wait
    record=dict(schema='wbb-loaded-jitter-diagnostic-1',started_utc=datetime.now(timezone.utc).isoformat(),
                plan=experiment,body_position_measured=False,chassis_acceleration_measured=False,
                frames=[],phases=[],capture_complete=False)
    args.report.parent.mkdir(parents=True,exist_ok=True);client=None;original=None
    with args.report.with_suffix('.jsonl').open('x') as log:
        try:
            client=Client(args.port,log);stop_and_wait(client)
            record['initial_status']=client.latest;original=client.latest['controller']
            assert client.latest['controller_revision']==experiment['expected_revision'],'Unexpected firmware'
            assert client.latest['ready'] and not client.latest['clients'],'Invalid preflight'
            client.request('controller',mode='robust_wheels');client.request('trace',enabled=True);client.pump(.1)
            od=client.latest['odometry'];origin=[od['x_m'],od['y_m'],od['yaw_rad']]
            age=client.latest.get('telemetry_build_ms',0)
            assert (time.monotonic()-client.last_status<=.20 and od['valid'] and
                    od['age_ms']+age<=25 and client.latest['control_age_ms']+age<=40 and
                    all(math.isfinite(v) for v in origin)), 'Invalid pre-arm telemetry'
            lease=client.request('arm')['lease'];seq=0
            for op,seconds in [('wheels',1.2),('pwm',.7)]:
                values=experiment['wheel_targets_cps']
                if op=='pwm':
                    recent=[f for f in record['frames'] if client.latest['ms']-200<=f['ms']]
                    assert len(recent)>=3,'Insufficient pre-transition samples'
                    values=[round(statistics.mean(f['motors'][i]['pwm'] for f in recent)) for i in range(3)]
                    assert all(abs(v)<=130 and v*t>0 for v,t in zip(values,experiment['wheel_targets_cps'])),'Fixed duty outside bounds'
                phase=dict(op=op,values=values,seconds=seconds);record['phases'].append(phase)
                end=time.monotonic()+seconds
                while time.monotonic()<end:
                    mark=len(client.frames);seq+=1
                    ack=client.request(op,lease=lease,seq=seq,values=values)
                    phase.setdefault('accepted_ms',ack['command_ms'])
                    client.pump(.035)
                    if time.monotonic()-client.last_status>.20:raise RuntimeError('Telemetry stale')
                    for f in client.frames[mark:]:
                        record['frames'].append(f)
                        if f['ms']>=phase['accepted_ms']:
                            check_frame(f,lease,origin,experiment)
                            if op=='pwm' and any(m['speed']*v<0 and abs(m['speed'])>200
                                               for m,v in zip(f['motors'],values)):
                                raise RuntimeError('Opposite feedback during fixed-duty phase')
            record['capture_complete']=True
        except Exception as error:
            record.update(capture_complete=False,error=str(error))
        finally:
            if client:
                mark=len(client.frames)
                try:
                    stop_and_wait(client);record['stop_frames']=client.frames[mark:]
                    client.request('trace',enabled=False)
                    if original:client.request('controller',mode=original)
                    client.pump(.1);record['final_status']=client.latest
                    if record['capture_complete']:
                        od=client.latest['odometry']
                        assert math.hypot(od['x_m']-origin[0],od['y_m']-origin[1])<=.1,'Post-stop encoder travel bound'
                        assert abs(math.atan2(math.sin(od['yaw_rad']-origin[2]),math.cos(od['yaw_rad']-origin[2])))<=.25,'Post-stop heading bound'
                except Exception as error:record.update(capture_complete=False,stop_error=str(error))
                finally:
                    try:client.close()
                    except Exception as error:record.update(capture_complete=False,close_error=str(error))
            with args.report.open('x') as f:json.dump(record,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in record.items() if k not in ['frames','stop_frames','initial_status','final_status']},indent=2))
    if not record['capture_complete']:raise SystemExit(1)


if __name__=='__main__':main()
