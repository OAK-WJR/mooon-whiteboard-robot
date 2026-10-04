#!/usr/bin/env python3
"""Repeated floor startup pulses; not a steady-state motor calibration.

Uses the existing bounded polarity collector, one USB session and a fixed
batch origin. Any fault, lack of motion, wrong sign or exceeded bound stops the
entire batch. No gains, validation flags or firmware are written.
"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path


def plan():
    return [dict(repeat=repeat,role='validation' if repeat==3 else 'development',
                 pwm=pwm,motor=motor,sign=sign,duration_s=duration)
            for repeat in range(1,4) for pwm,duration in [(140,.12),(160,.10),(180,.08)]
            for motor in [1,2,3] for sign in [1,-1]]


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true');ap.add_argument('--floor-confirmed',action='store_true')
    ap.add_argument('--port');ap.add_argument('--directory',type=Path);ap.add_argument('--hardware-label')
    ap.add_argument('--max-trials',type=int,default=54)
    args=ap.parse_args();trials=plan()
    if not args.execute:
        print(json.dumps({'trials':trials,'trial_count':54,'nominal_driven_seconds':sum(t['duration_s'] for t in trials),
                          'purpose':'Startup polarity and repeatability at three PWM levels; not steady-state identification'},indent=2));return
    if not args.floor_confirmed or not args.port or not args.directory or not args.hardware_label:
        ap.error('Requires confirmed floor conditions, hardware label, port and new directory')
    if not 1<=args.max_trials<=54:ap.error('max-trials must be 1–54')
    args.directory.mkdir(parents=True,exist_ok=False)
    from c6_drive_tune import Client
    from c6_drive_sysid import stop_and_wait
    from c6_floor_polarity import PLAN,pose,pulse
    PLAN['pulse_seconds']=.12
    report=dict(hardware_label=args.hardware_label,created_utc=datetime.now(timezone.utc).isoformat(),
                requested_trials=trials[:args.max_trials],runs=[],status='COLLECTING',firmware_or_gain_changes=False)
    manifest=args.directory/'manifest.json'
    def save():manifest.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    save();client=None
    with (args.directory/'telemetry.jsonl').open('x') as log:
        try:
            client=Client(args.port,log);stop_and_wait(client)
            report['initial']=client.latest;origin=pose(client.latest)
            client.request('trace',enabled=True)
            for index,trial in enumerate(report['requested_trials'],1):
                PLAN['pwm']=trial['pwm']
                PLAN['pulse_seconds']=trial['duration_s']
                run=dict(trial=trial);report['runs'].append(run)
                print(f'Pulse {index}/{args.max_trials}: M{trial["motor"]} {trial["sign"]:+d} {trial["pwm"]}/255',flush=True)
                pulse(client,trial['motor']-1,trial['sign'],run,origin)
                print(json.dumps(run['result']),flush=True)
                save()
            report['status']='CAPTURED'
        except Exception as error:
            report['status']='STOPPED_FOR_REVIEW';report['error']=str(error)
        finally:
            if client:
                try:
                    stop_and_wait(client);client.request('trace',enabled=False)
                    report['final']=client.latest;client.close()
                except Exception as error:
                    report['stop_error']=str(error);report['status']='STOPPED_FOR_REVIEW'
            save()
    print(json.dumps({'status':report['status'],'runs':len(report['runs']),'error':report.get('error')}))
    if report['status']!='CAPTURED':raise SystemExit(1)


if __name__=='__main__':main()
