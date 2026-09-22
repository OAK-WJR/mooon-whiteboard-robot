#!/usr/bin/env python3
"""Measured C6 adapter calibration. All wheel indices follow physical order 1,2,3."""
import argparse
import json
from pathlib import Path
import time
import serial
import numpy as np

class Client:
    def __init__(self, port, logfile=None):
        self.s=serial.Serial(port=None,baudrate=115200,timeout=.01,write_timeout=.3)
        self.s.dtr=True; self.s.rts=False; self.s.port=port; self.s.open()
        self.logfile=logfile; self.latest=None; self.frames=[]; self.acks={}; self.id=0
        self.buffer=b''; self.session=None; self.last_status=0
        end=time.monotonic()+8
        while self.latest is None and time.monotonic()<end: self.pump(.05)
        if self.latest is None: self.s.close(); raise RuntimeError('No matching firmware')
        self.request('stop')
    def pump(self, seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.buffer+=self.s.read(self.s.in_waiting or 1)
            while b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1)
                try:d=json.loads(line)
                except (ValueError,UnicodeError):continue
                if not isinstance(d,dict):continue
                if self.logfile:self.logfile.write(json.dumps({'host':time.monotonic(),'frame':d})+'\n')
                if d.get('type')=='status':
                    if d.get('firmware')!='wbb-c6-drive-1':raise RuntimeError('Firmware mismatch')
                    if self.session is not None and self.session!=d['session']:raise RuntimeError('Board rebooted')
                    self.session=d['session'];self.latest=d;self.last_status=time.monotonic();self.frames.append(d)
                elif d.get('type')=='ack':self.acks[d.get('id')]=d
            if len(self.buffer)>8192:raise RuntimeError('RX overflow')
    def request(self,op,**args):
        self.id+=1;payload=(json.dumps({'id':self.id,'op':op,**args})+'\n').encode()
        if self.s.write(payload)!=len(payload):raise RuntimeError('Short USB write')
        end=time.monotonic()+.7
        while self.id not in self.acks and time.monotonic()<end:self.pump(.005)
        d=self.acks.pop(self.id,None)
        if not d or not d.get('ok'):raise RuntimeError(f'{op}: {d}')
        return d
    def run(self,op,values,seconds=1):
        self.request('stop');self.pump(.65)
        if not self.latest.get('ready'):raise RuntimeError('Output fault: '+self.latest['reason'])
        if any(abs(m['speed'])>50 for m in self.latest['motors']):
            raise RuntimeError('Wheel moving before command; test cancelled')
        before=[m['count'] for m in self.latest['motors']]
        lease=self.request('arm')['lease'];seq=0;frames=[];end=time.monotonic()+seconds
        try:
            while time.monotonic()<end:
                start=len(self.frames)
                self.request(op,lease=lease,seq=seq+1,values=values);seq+=1
                watermark=self.latest['ms']
                self.pump(.065)
                frames.extend(f for f in self.frames[start:] if f['armed'] and f['lease']==lease)
                if time.monotonic()-self.last_status>.25:raise RuntimeError('Stale telemetry')
                # A successful command ACK may arrive before the next status.
                # Do not mistake the previous stopped sample for a new stop.
                if self.latest['ms']==watermark:continue
                if self.latest['reason'] not in ('READY','BENCH','DRIVE'):raise RuntimeError(self.latest['reason'])
                for i,value in enumerate(values):
                    delta=((self.latest['motors'][i]['count']-before[i]+0x80000000)&0xffffffff)-0x80000000
                    if value==0 and abs(delta)>10:
                        raise RuntimeError(f'Uncommanded motion on physical motor {i+1}; test cancelled')
        finally:self.request('stop');self.pump(.5)
        return frames
    def close(self):
        try:self.request('stop')
        finally:self.s.close()

def summarize(frames,i,target):
    if len(frames)<4:raise RuntimeError('Insufficient live telemetry')
    values=np.array([f['motors'][i]['speed'] for f in frames])
    tail=values[len(values)//2:]
    first,last=frames[len(values)//2]['motors'][i],frames[-1]['motors'][i]
    travel=abs(((last['count']-first['count']+0x80000000)&0xffffffff)-0x80000000)
    edges=(last['edges']-first['edges'])&0xffffffff
    return {'target':target,'mean':float(np.mean(tail)),'std':float(np.std(tail)),
            'quadrature_efficiency':travel/edges if edges else 0,
            'relative_error':float(abs(np.mean(tail)-target)/max(abs(target),1)),
            'samples':len(values)}

def fit_gains(results):
    """Fit measured steady speed -> PWM separately for every wheel/direction.

    PI gains begin conservatively relative to measured inverse plant gain.
    They are candidates only; validate() must demonstrate tracking before save.
    """
    gains=[]
    for motor in range(1,4):
        pair=[]
        for sign in [1,-1]:
            rows=[r for r in results if r['motor']==motor and r['sign']==sign
                  and not r.get('error') and r['mean']*sign>100
                  and r.get('std',float('inf'))<abs(r['mean'])*.12
                  and r.get('quadrature_efficiency',0)>.8]
            if len(rows)<3:raise ValueError(f'Motor {motor}, direction {sign}: need 3 moving measurements')
            speeds=np.array([abs(r['mean']) for r in rows]);duties=np.array([r['duty'] for r in rows])
            if np.ptp(speeds)<500:raise ValueError('No usable variable-speed response')
            kv,ks=np.polyfit(speeds,duties,1)
            if not 0<kv<.2 or not -40<ks<180:raise ValueError('Invalid motor model')
            pair.append([float(.3*kv),float(kv),float(kv),float(max(0,ks))])
        gains.append(pair)
    return gains

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--port',default='/dev/cu.usbmodem1101')
    ap.add_argument('--mode',choices=['status','sweep','fit','validate','watchdog'],default='status')
    ap.add_argument('--report',default='c6_drive/reports/bench.json');ap.add_argument('--gains')
    ap.add_argument('--sweep',default='c6_drive/reports/sweep.json');ap.add_argument('--save',action='store_true')
    ap.add_argument('--duties',type=int,nargs='+',default=[80,120,160,200])
    ap.add_argument('--motors',type=int,nargs='+',choices=[1,2,3],default=[1,2,3])
    args=ap.parse_args();path=Path(args.report);path.parent.mkdir(parents=True,exist_ok=True)
    report={'mode':args.mode,'units':'encoder counts/s','wheel_order':'physical 1,2,3','results':[]}
    if args.mode=='fit':
        report['gains']=fit_gains(json.loads(Path(args.sweep).read_text())['results'])
        report['status']='CANDIDATE_REQUIRES_BENCH_VALIDATION';path.write_text(json.dumps(report,indent=2));return
    if args.save and args.mode!='validate':ap.error('--save requires --mode validate')
    with path.with_suffix('.jsonl').open('w') as log:
        c=Client(args.port,log)
        try:
            if args.gains:
                gains=json.loads(Path(args.gains).read_text())['gains']
                for i in range(3):
                    for direction in range(2):c.request('gains',motor=i,dir=direction,values=gains[i][direction])
            if args.mode=='status':print(json.dumps(c.latest,ensure_ascii=False,indent=2))
            elif args.mode=='sweep':
                for motor in args.motors:
                    i=motor-1
                    for sign in [1,-1]:
                        for duty in args.duties:
                            values=[0,0,0];values[i]=sign*duty
                            try:
                                frames=c.run('pwm',values,.9)
                                result=summarize(frames,i,0);result.update(motor=i+1,sign=sign,duty=duty)
                                if result['mean']*sign<-100:raise RuntimeError('Encoder direction mismatch')
                            except RuntimeError as e:
                                if 'NO_ENCODER' not in str(e):raise
                                result={'motor':i+1,'sign':sign,'duty':duty,'mean':0,'error':str(e)}
                            report['results'].append(result);print(json.dumps(result),flush=True)
                            path.write_text(json.dumps(report,indent=2))
            elif args.mode=='validate':
                for i in range(3):
                    for target in [1800,3000,-1800,-3000]:
                        values=[0,0,0];values[i]=target
                        frames=c.run('wheels',values,1.6)
                        result=summarize(frames,i,target);result['motor']=i+1
                        report['results'].append(result);print(json.dumps(result),flush=True)
                        path.write_text(json.dumps(report,indent=2))
                report['passed']=all(r['relative_error']<.10 and r['std']<abs(r['target'])*.20
                                     and r['quadrature_efficiency']>.8 for r in report['results'])
                if args.save:
                    if not report['passed']:raise RuntimeError('Tracking validation failed; gains NOT saved')
                    c.request('save');c.pump(.15)
                    if not c.latest['calibrated']:raise RuntimeError('Saved calibration not confirmed')
                    report['saved']=True
            elif args.mode=='watchdog':
                lease=c.request('arm')['lease'];c.request('wheels',lease=lease,seq=1,values=[1000,0,0])
                start=time.monotonic();c.pump(.6)
                assert not c.latest['armed'] and c.latest['reason']=='LINK_TIMEOUT'
                assert all(m['pwm']==0 for m in c.latest['motors'])
                assert all(m['fi_level']==0 and m['bi_level']==0 for m in c.latest['motors'])
                report['results'].append({'deadman':'PASS','observed_after_seconds':time.monotonic()-start})
                print(report['results'][-1])
            report['final']=c.latest;path.write_text(json.dumps(report,indent=2))
        finally:c.close()

if __name__=='__main__':main()
