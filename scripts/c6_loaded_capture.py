#!/usr/bin/env python3
"""Short loaded-floor response capture using existing firmware, not a new flash.

Default/--plan never import serial or contact a robot. Execution additionally
requires a current user-confirmed off-wall, powered floor setup and --execute
--floor-confirmed. Flags document that confirmation; they do not replace it.
One direction per invocation, no automatic retry/re-arm after faults.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

MPC=.11938052/2800
DIRECTIONS={'forward':(1,0),'backward':(-1,0),'left':(0,1),'right':(0,-1)}


def plan(direction='forward',seconds=.6,speed=.04):
    if seconds not in (.6,1.,1.2,2.):raise ValueError('Floor step must be 0.6, 1, 1.2 or 2 seconds')
    if speed not in (.04,.06,.08) or seconds*speed>.080001:
        raise ValueError('Floor speed must be 0.04/0.06/0.08 m/s, nominal travel at most 8 cm')
    x,y=DIRECTIONS[direction];x*=speed;y*=speed
    return dict(stage='short_loaded_floor_capture',direction=direction,
                wheel_targets_cps=[(-math.sqrt(3)/2*x+.5*y)/MPC,-y/MPC,(math.sqrt(3)/2*x+.5*y)/MPC],
                body_request_mps=speed,motion_seconds=seconds,
                encoder_excursion_limit_m=.10 if seconds==2 or seconds*speed>.06 else .06,encoder_heading_limit_rad=.25,
                telemetry_timeout_s=.20,maximum_existing_pwm=255,
                controller='robust_wheels',firmware_or_gain_changes=False,
                warning='Encoder limits are not independent chassis-position limits. Clear floor required.')


def check_frame(frame,lease,origin,experiment=None):
    build_ms=frame.get('telemetry_build_ms',0)
    if not math.isfinite(build_ms) or build_ms<0:raise RuntimeError('Invalid telemetry age')
    if not frame.get('ready') or frame.get('control_age_ms',1000)+build_ms>40:
        raise RuntimeError('Output fault or stale control loop')
    if not frame.get('armed') or frame.get('lease')!=lease:
        raise RuntimeError('Firmware stopped: '+frame.get('reason','unknown'))
    if frame.get('clients',0):raise RuntimeError('Wi-Fi client connected during USB capture')
    if frame.get('controller')!='robust_wheels':raise RuntimeError('Controller changed during capture')
    od=frame['odometry']
    if not od.get('valid') or od.get('age_ms',1000)+build_ms>25:raise RuntimeError('Invalid odometry')
    if not all(math.isfinite(v) for v in [*origin,od['x_m'],od['y_m'],od['yaw_rad']]):
        raise RuntimeError('Non-finite odometry')
    distance=experiment['encoder_excursion_limit_m'] if experiment else .06
    if math.hypot(od['x_m']-origin[0],od['y_m']-origin[1])>distance:
        raise RuntimeError('Encoder travel limit reached')
    angle=math.atan2(math.sin(od['yaw_rad']-origin[2]),math.cos(od['yaw_rad']-origin[2]))
    if abs(angle)>.25:raise RuntimeError('Encoder heading limit reached')
    if len(frame['motors'])!=3:raise RuntimeError('Missing motor telemetry')
    for m in frame['motors']:
        if not math.isfinite(m['pwm']) or abs(m['pwm'])>255:raise RuntimeError('Invalid PWM report')


def analyze(samples,command_ms,targets,seconds=.6):
    samples=sorted({int(row[0]):row for row in samples if command_ms<=row[0]<=command_ms+round(seconds*1000)}.values())
    if len(samples)<10:raise RuntimeError('Insufficient board-clock samples')
    gaps=[b[0]-a[0] for a,b in zip(samples,samples[1:])]
    if max(gaps)>40:raise RuntimeError('Board-clock sample gap exceeds 40 ms')
    motors=[]
    for i,target in enumerate(targets):
        sign=1 if target>0 else -1 if target<0 else 0
        onset=None
        if sign:
            for k in range(len(samples)-2):
                if all(row[2+i]*sign>50 for row in samples[k:k+3]):
                    onset=(samples[k][0]-command_ms)/1000;break
        motors.append(dict(motor=i+1,target_cps=target,motion_onset_s=onset,
                           peak_applied_pwm=max(abs(row[8+i]) for row in samples),
                           tail_mean_cps=statistics.mean(row[2+i] for row in samples[-10:])))
    return dict(samples=samples,motors=motors,maximum_gap_ms=max(gaps),
                classification='Loaded wheel response only; no body-slip, voltage, current or mass inference')


def capture(client,experiment,record):
    from c6_drive_sysid import stop_and_wait
    stop_and_wait(client)
    if client.latest.get('clients'):raise RuntimeError('Disconnect Wi-Fi controllers before USB capture')
    original=client.latest.get('controller')
    original_assist=client.latest.get('heading_assist',{}).get('enabled',False)
    original_direct=client.latest.get('heading_assist',{}).get('direct_feedback',False)
    original_fast=client.latest.get('heading_assist',{}).get('fast_release',False)
    if original not in ('legacy','robust_wheels','encoder_hold'):raise RuntimeError('Unrecognized existing controller')
    record['initial_status']=client.latest
    origin=None
    try:
        client.request('controller',mode='robust_wheels')
        if 'heading_assist' in experiment:
            kwargs={'direct':experiment['direct_heading']} if 'direct_heading' in experiment else {}
            if 'fast_release' in experiment:kwargs['fast_release']=experiment['fast_release']
            client.request('heading_assist',enabled=experiment['heading_assist'],**kwargs)
        client.request('trace',enabled=True);client.pump(.1)
        record['experiment_status']=client.latest
        if experiment.get('controlled_release') and 'braking' not in client.latest.get('heading_assist',{}):
            raise RuntimeError('Firmware does not report coordinated release state')
        if ('heading_assist' in experiment and
                client.latest.get('heading_assist',{}).get('enabled')!=experiment['heading_assist']):
            raise RuntimeError('Heading-assist setting not confirmed')
        if ('direct_heading' in experiment and
                client.latest.get('heading_assist',{}).get('direct_feedback')!=experiment['direct_heading']):
            raise RuntimeError('Direct heading setting not confirmed')
        if ('fast_release' in experiment and
                client.latest.get('heading_assist',{}).get('fast_release')!=experiment['fast_release']):
            raise RuntimeError('Duty-reduction setting not confirmed')
        od=client.latest['odometry'];origin=[od['x_m'],od['y_m'],od['yaw_rad']]
        if (time.monotonic()-client.last_status>.20 or not client.latest.get('ready') or
                client.latest.get('control_age_ms',1000)+client.latest.get('telemetry_build_ms',0)>40 or client.latest.get('clients',0) or
                client.latest.get('controller')!='robust_wheels' or not od.get('valid') or
                od.get('age_ms',1000)+client.latest.get('telemetry_build_ms',0)>25 or not all(math.isfinite(v) for v in origin)):
            raise RuntimeError('Invalid pre-arm telemetry; no motion command sent')
        lease=client.request('arm')['lease'];sequence=0;accepted=None
        deadline=time.monotonic()+experiment['motion_seconds']
        while time.monotonic()<deadline:
            mark=len(client.frames);sequence+=1
            ack=client.request('wheels',lease=lease,seq=sequence,values=experiment['wheel_targets_cps'])
            if accepted is None:accepted=ack['command_ms'];record['command_ms']=accepted
            client.pump(.035)
            if time.monotonic()-client.last_status>.20:raise RuntimeError('Telemetry stale')
            for frame in client.frames[mark:]:
                record['frames'].append(frame);record['samples'].extend(frame.get('control_samples',[]))
                # Ignore only pre-command packets; never ignore a post-command stop.
                if frame['ms']>=accepted:check_frame(frame,lease,origin,experiment)
        if experiment.get('controlled_release'):
            release_deadline=time.monotonic()+.65;quiet=False
            while time.monotonic()<release_deadline:
                mark=len(client.frames);sequence+=1
                ack=client.request('wheels',lease=lease,seq=sequence,values=[0,0,0])
                record.setdefault('release_command_ms',ack['command_ms'])
                client.pump(.025)
                if time.monotonic()-client.last_status>.20:raise RuntimeError('Release telemetry stale')
                for frame in client.frames[mark:]:
                    record['frames'].append(frame);record['samples'].extend(frame.get('control_samples',[]))
                    if frame['ms']>=record['release_command_ms']:
                        check_frame(frame,lease,origin,experiment)
                        quiet=(not frame.get('heading_assist',{}).get('braking',True) and
                               all(m['pwm']==0 and abs(m['speed'])<30 for m in frame['motors']))
                if quiet:break
            if not quiet:raise RuntimeError('Controlled release did not settle')
            record['controlled_release_complete']=True
    finally:
        # Any failed stop verification propagates, preventing mode restoration,
        # another direction or an automatic repeat. No NVS/validation writes.
        stop_mark=len(client.frames)
        record['stop_requested_host']=time.monotonic()
        try:
            stop_and_wait(client)
        finally:
            # Keep the stop transient even when confirmation fails. The raw
            # JSONL already contains it; expose it in the report as well.
            record['stop_frames']=client.frames[stop_mark:]
            record['final_status']=client.latest
        client.request('trace',enabled=False)
        if 'heading_assist' in experiment:
            kwargs={'direct':original_direct} if 'direct_heading' in experiment else {}
            if 'fast_release' in experiment:kwargs['fast_release']=original_fast
            client.request('heading_assist',enabled=original_assist,**kwargs)
        client.request('controller',mode=original)
        client.pump(.1);record['final_status']=client.latest
        od=client.latest['odometry']
        if origin is not None and math.hypot(od['x_m']-origin[0],od['y_m']-origin[1])>experiment['encoder_excursion_limit_m']:
            raise RuntimeError('Post-stop encoder travel limit reached')
        if origin is not None and abs(math.atan2(math.sin(od['yaw_rad']-origin[2]),math.cos(od['yaw_rad']-origin[2])))>.25:
            raise RuntimeError('Post-stop encoder heading limit reached')
    # Stop and verify before analysis; a failed stop cannot produce a completed capture.
    record['analysis']=analyze(record['samples'],accepted,experiment['wheel_targets_cps'],experiment['motion_seconds'])
    record['capture_complete']=True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',action='store_true');parser.add_argument('--execute',action='store_true')
    parser.add_argument('--floor-confirmed',action='store_true')
    parser.add_argument('--direction',choices=DIRECTIONS,default='forward')
    parser.add_argument('--seconds',type=float,choices=[.6,1.,1.2,2.],default=.6)
    parser.add_argument('--speed',type=float,choices=[.04,.06,.08],default=.04)
    parser.add_argument('--heading-assist',choices=['on','off'],help='USB-only stopped development setting')
    parser.add_argument('--direct-heading',choices=['on','off'],help='USB-only comparison of feedback placement')
    parser.add_argument('--fast-release',choices=['on','off'],help='USB-only comparison of duty reduction rate')
    parser.add_argument('--controlled-release',action='store_true',help='Send zero velocity before the final hard STOP')
    transport=parser.add_mutually_exclusive_group()
    transport.add_argument('--port');transport.add_argument('--url',help='Home-network robot URL; uses the existing web validation gates')
    parser.add_argument('--key-file',type=Path,help='Wi-Fi device key file; not a network password')
    parser.add_argument('--report',type=Path)
    parser.add_argument('--hardware-label',default='Unspecified hardware',
                        help='Human-supplied hardware identity for this capture, not a measured device ID')
    args=parser.parse_args();experiment=plan(args.direction,args.seconds,args.speed)
    if args.heading_assist is not None:
        if args.url:parser.error('Heading-assist trials require USB')
        experiment['heading_assist']=args.heading_assist=='on'
    if args.direct_heading is not None:
        if args.url or args.heading_assist!='on':parser.error('Direct feedback comparison needs USB and heading assist on')
        experiment['direct_heading']=args.direct_heading=='on'
    if args.fast_release is not None:
        if args.url or args.heading_assist!='on' or args.direct_heading is not None:
            parser.error('Duty reduction comparison requires USB, assist on, and no direct-feedback experiment')
        experiment['fast_release']=args.fast_release=='on'
    if args.controlled_release:experiment['controlled_release']=True
    if args.plan or not args.execute:print(json.dumps(experiment,indent=2));return
    if not args.floor_confirmed or not (args.port or args.url) or not args.report:
        parser.error('Execution requires current user confirmation, --floor-confirmed, --port or --url, and --report')
    if args.report.exists() or args.report.with_suffix('.jsonl').exists():parser.error('Use a new evidence path')
    if args.url:
        from c6_wifi_client import WifiClient as Client
    else:
        from c6_drive_tune import Client  # Lazy: planning/analysis cannot open serial.
    args.report.parent.mkdir(parents=True,exist_ok=True)
    record=dict(schema='wbb-loaded-capture-1',started_utc=datetime.now(timezone.utc).isoformat(),
                hardware_label=args.hardware_label,condition='floor',
                plan=experiment,transport='wifi' if args.url else 'usb',simulation_only=False,mass_kg=None,body_position_measured=False,
                voltage_measured=False,current_measured=False,frames=[],samples=[],capture_complete=False)
    client=None
    with args.report.with_suffix('.jsonl').open('x') as log:
        try:
            kwargs={'device_key':args.key_file.read_text().strip() if args.key_file else None} if args.url else {}
            client=Client(args.url or args.port,log,**kwargs);capture(client,experiment,record)
        except Exception as error:
            record['error']=str(error);record['capture_complete']=False
        finally:
            if client:
                try:client.close()
                except Exception as error:record['final_stop_error']=str(error);record['capture_complete']=False
            with args.report.open('x') as report:json.dump(record,report,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in record.items() if k not in ('frames','samples','stop_frames','initial_status','final_status','analysis')},indent=2))
    if not record['capture_complete']:raise SystemExit(1)


if __name__=='__main__':main()
