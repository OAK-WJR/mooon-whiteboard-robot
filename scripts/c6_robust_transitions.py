#!/usr/bin/env python3
"""Explicit suspended test of reversal, zero command and command expiry."""
import argparse
import json
from pathlib import Path
import time
from c6_drive_tune import Client
from c6_robust_bench import stopped


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--suspended',action='store_true')
    ap.add_argument('--report',type=Path,required=True)
    args=ap.parse_args()
    if not args.suspended:ap.error('Requires the confirmed suspended robot')
    if args.report.exists() or args.report.with_suffix('.jsonl').exists():ap.error('Report already exists')
    report={'simulation_only':False,'suspended':True,'results':[]};c=None
    with args.report.with_suffix('.jsonl').open('x') as log:
        try:
            c=Client('/dev/cu.usbmodem1101',log);stopped(c)
            if c.latest.get('controller_revision')!='robust-wheels-4':raise RuntimeError('Wrong controller revision')
            if c.latest.get('clients'):raise RuntimeError('Wi-Fi client connected')
            c.request('controller',mode='robust_wheels');c.request('trace',enabled=True);c.pump(.15)
            report['initial']=c.latest
            lease=c.request('arm')['lease'];seq=0
            for phase,values,duration in [('forward',[3000,3000,3000],1.4),('reverse',[-3000,-3000,-3000],1.6),('release',[0,0,0],.65)]:
                frames=[];end=time.monotonic()+duration
                while time.monotonic()<end:
                    mark=len(c.frames);seq+=1;ack=c.request('wheels',lease=lease,seq=seq,values=values);c.pump(.045)
                    for f in c.frames[mark:]:
                        if not f['armed'] or f['lease']!=lease:raise RuntimeError('Transition fault: '+f['reason'])
                        frames.append(f)
                if not frames:raise RuntimeError('No trace')
                last=frames[-1]
                if phase=='release':
                    if any(abs(m['speed'])>30 or m['pwm']!=0 for m in last['motors']):raise RuntimeError('Release failed')
                elif any(abs(m['speed']-values[i])>abs(values[i])*.10 for i,m in enumerate(last['motors'])):
                    raise RuntimeError('Transition did not settle')
                report['results'].append({'phase':phase,'target':values,'frames':frames});print(phase,'PASS',flush=True)
            stopped(c)
            lease=c.request('arm')['lease'];ack=c.request('wheels',lease=lease,seq=1,values=[3000,0,0]);mark=len(c.frames)
            c.pump(.75) # Deliberately no further command heartbeat; MCU must stop.
            frames=c.frames[mark:];faults=[f for f in frames if f['reason']=='LINK_TIMEOUT']
            if not faults:raise RuntimeError('Command timeout did not latch')
            first=faults[0]
            if first['armed'] or any(m['pwm']!=0 or m['fi_duty'] or m['bi_duty'] or m['fi_level'] or m['bi_level'] for m in first['motors']):
                raise RuntimeError('Nonzero drive after timeout')
            samples=sorted({row[0]:row for f in frames for row in f.get('control_samples',[])}.values())
            zero=[row[0]-ack['command_ms'] for row in samples if row[0]-ack['command_ms']>=300 and row[8:11]==[0,0,0]]
            if not zero or min(zero)>320:raise RuntimeError('Late command-timeout output shutdown')
            report['results'].append({'phase':'command_timeout','zero_after_ms':min(zero),'frames':frames})
            print('command timeout PASS; zero output after',min(zero),'ms',flush=True)
            report['passed']=True
        except BaseException as e:
            report['error']=str(e);raise
        finally:
            if c:
                try:
                    stopped(c);c.request('trace',enabled=False);c.request('controller',mode='legacy');c.pump(.1);report['final']=c.latest
                finally:c.close()
            args.report.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
