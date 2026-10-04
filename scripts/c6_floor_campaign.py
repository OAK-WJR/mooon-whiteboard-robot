#!/usr/bin/env python3
"""Plan or collect repeated floor traces, stopping on any failed run.

Planning and analysis are offline. Execution requires the correct deployed
firmware, current user-authorized floor conditions and an explicit --execute.
Each invocation creates a new batch; no automatic fault retry or firmware write.
"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import sys


def plan():
    trials=[]
    for repeat in range(1,4):
        role='validation' if repeat==3 else 'development'
        for speed,duration in [(.04,1.5),(.06,1.0),(.08,.75),(.10,.6),(.12,.5)]:
            directions=['W','S','A','D'] if repeat%2 else ['A','D','W','S']
            for direction in directions:
                trials.append(dict(pair=direction*2,speed=speed,duration=duration,repeat=repeat,role=role))
        for pair in ['WA','SD','AD','DA','WS','SW','AS','DW']:
            trials.append(dict(pair=pair,speed=.08,duration=None,repeat=repeat,role=role))
    return dict(trials=trials,trial_count=len(trials),nominal_driven_seconds=sum(t['duration'] or .85 for t in trials),
        development_count=sum(t['role']=='development' for t in trials),validation_count=sum(t['role']=='validation' for t in trials),
        per_run_encoder_excursion_m=.075,per_run_heading_rad=.30,
        batch_encoder_excursion_m=.15,batch_heading_rad=.40,monitor_pwm_ceiling=180,
        full_batch_is_conditional_on_usable_motion=True,
        note='Encoder bounds cannot detect chassis slip. No automatic retry after faults or observed loss of motion.')


def endpoint_ok(status,origin):
    from c6_floor_polarity import bounds
    bounds(status,origin,.15,.40)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true');ap.add_argument('--floor-confirmed',action='store_true')
    ap.add_argument('--port');ap.add_argument('--directory',type=Path)
    ap.add_argument('--hardware-label',help='User-confirmed board and wiring identity')
    ap.add_argument('--max-trials',type=int,default=84)
    args=ap.parse_args();experiment=plan()
    if not args.execute:
        print(json.dumps(experiment,indent=2));return
    if not args.floor_confirmed or not args.port or not args.directory or not args.hardware_label:
        ap.error('Requires confirmed hardware label, floor setup, port and new output directory')
    if not 1<=args.max_trials<=84:ap.error('max-trials must be 1–84')
    args.directory.mkdir(parents=True,exist_ok=False)
    from c6_floor_polarity import pose
    from c6_floor_response_report import summarize
    result=dict(plan=experiment,hardware_label=args.hardware_label,created_utc=datetime.now(timezone.utc).isoformat(),
                runs=[],firmware_or_gain_changes=False,status='COLLECTING',origin=None,session=None)
    manifest=args.directory/'manifest.json'
    def save():manifest.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    save()
    for index,trial in enumerate(experiment['trials'][:args.max_trials],1):
        path=args.directory/f'{index:03d}-{trial["pair"]}-{round(trial["speed"]*100):02d}.json'
        args_run=[sys.executable,str(Path(__file__).with_name('c6_floor_reversal.py')),
                  '--execute','--floor-confirmed','--port',args.port,'--pair',trial['pair'],
                  '--speed',str(trial['speed']),'--report',str(path)]
        if trial['duration'] is not None:args_run+=['--duration',str(trial['duration'])]
        if result['session'] is not None:
            args_run+=['--expected-session',str(result['session']),'--batch-origin',*[str(x) for x in result['origin']]]
        print(f'Capture {index}/{args.max_trials}: {trial}',flush=True)
        done=subprocess.run(args_run,capture_output=True,text=True)
        record=json.loads(path.read_text()) if path.exists() else {}
        entry=dict(trial=trial,report=str(path),capture_complete=record.get('passed',False),
                   error=record.get('error'),returncode=done.returncode,stderr=done.stderr)
        result['runs'].append(entry)
        try:
            if done.returncode or not entry['capture_complete']:
                raise RuntimeError(entry['error'] or 'Capture failed; see raw report')
            initial,final=record['initial'],record['final']
            if result['origin'] is None:
                result['origin']=pose(initial);result['session']=initial['session']
            if initial['session']!=result['session'] or final['session']!=result['session']:
                raise RuntimeError('Device session changed; do not mix hardware sessions')
            endpoint_ok(initial,result['origin']);endpoint_ok(final,result['origin'])
            if trial['pair'][0]==trial['pair'][1]:
                quality=summarize(path)
                entry['motion_quality_passed']=quality['motion_quality_passed']
                entry['wheels']=quality['wheels']
                # Poor tracking alone remains useful data; inability to start or
                # sustained low speed requires a focused diagnosis before more trials.
                if any(w['onset_ms'] is None or w['longest_low_speed_after_300ms']>=200 for w in quality['wheels']):
                    raise RuntimeError('Observed loss of wheel motion; retain trace and diagnose before continuing')
        except Exception as error:
            result['status']='STOPPED_FOR_REVIEW';entry['stop_reason']=str(error);save()
            print(json.dumps(entry,indent=2),flush=True);raise SystemExit(1)
        save();print('Captured and stopped.',flush=True)
    result['status']='CAPTURED';save()
    print(json.dumps({'status':result['status'],'trials':len(result['runs']),'manifest':str(manifest)}))


if __name__=='__main__':main()
