#!/usr/bin/env python3
"""Encoder-only C++ controller against quantized wheel plants and free-body MuJoCo.

No hardware/network access. Plant truth is available only to the scorer.
All unmeasured loads/contacts/delays are declared assumptions, not robot data.
"""
import argparse
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from engine import Plant, ROOT, DT, MPC, RAD_PER_COUNT, MODELS


def build():
    sources=[ROOT/'include'/x for x in ('control.h','encoder_servo.h','encoder_hold.h','robust_wheels.h','robust_control.h')]
    sources.append(Path(__file__).with_name('encoder_bridge.cpp'))
    digest=hashlib.sha256(b''.join(p.read_bytes() for p in sources)).hexdigest()[:18]
    exe=Path(tempfile.gettempdir())/f'wbb-encoder-{digest}'
    if not exe.exists():
        subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',str(ROOT/'include'),str(sources[-1]),'-o',str(exe)],check=True)
    return exe,sources


class Native:
    def __init__(self,variant,exe):
        self.proc=subprocess.Popen([str(exe),variant],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)

    def step(self,n,command,delta,speed,applied,fresh=True,stop=False):
        values=[(n+1)*10,DT,*command,*delta,*speed,*applied,int(fresh),int(stop)]
        self.proc.stdin.write(' '.join('0' if abs(x)<1e-12 else format(float(x),'.10g') for x in values)+'\n')
        self.proc.stdin.flush()
        row=self.proc.stdout.readline().split()
        if len(row)!=18:raise RuntimeError(f'Native error: {row!r}, code={self.proc.poll()}')
        x=[float(v) for v in row[:15]]
        return dict(pwm=x[:3],reference=x[3:6],integral=x[6:9],estimate=x[9:12],goal=x[12:15],
                    reason=row[15],armed=bool(int(row[16])),scale=float(row[17]))

    def close(self):
        self.proc.stdin.close();self.proc.wait(timeout=5);self.proc.stdout.close()


@dataclass
class Case:
    name:str
    plant:str='wheels'
    axis:int=0
    speed:float=.08
    gain:tuple=(1.,1.,1.)
    tau:tuple=(1.,1.,1.)
    delay:tuple=(0,0,0)
    load:tuple=(0.,0.,0.) # equivalent effective PWM, wheel model only
    stiction:bool=False
    mass:float=.30
    incline:float=0.
    adhesion:float=0.
    mu:float=.65
    weak:float=1.
    event:str='none'
    seconds:float=10.
    seed:int=0


def command_at(c,t):
    out=np.zeros(3)
    if c.event=='hold_only':return out
    if 1<=t<3:out[c.axis]=c.speed
    if c.event=='reverse' and 4<=t<6:out[c.axis]=-c.speed
    return out


def run(job):
    c,variant,exe=job
    controller=Native(variant,exe)
    plant=Plant(mass=c.mass,incline=c.incline,mu=c.mu) if c.plant=='mujoco' else None
    position=np.zeros(3);velocity=np.zeros(3);counts=np.zeros(3);filt=np.zeros(3)
    truth=np.zeros(3);task=np.zeros(3);applied=np.zeros(3)
    delays=[deque([0.]*d) for d in c.delay]
    trace=[];ended=False;faults=set();post_fault=0;peak_pwm=0;max_integral=0;max_slew=0
    try:
        for n in range(round(c.seconds/DT)):
            t=n*DT;cmd=command_at(c,t);task+=cmd*DT
            if plant:
                newcounts=np.round(plant.d.qpos[plant.angles]/RAD_PER_COUNT)
            else:newcounts=np.round(position)
            delta=newcounts-counts;counts=newcounts
            if c.event=='encoder_stuck' and t>=1.5:delta[0]=0
            filt+=.4*(delta/DT-filt)
            if c.event=='encoder_invalid' and t>=4:delta[0]=100000
            fresh=not(c.event=='lease_loss' and t>=4)
            o=controller.step(n,cmd,delta,filt,applied,fresh,ended or (c.event=='stop' and t>=4))
            pwm=np.array(o['pwm'])
            if not o['armed']:
                faults.add(o['reason']);post_fault+=int(np.any(pwm))
            else:max_slew=max(max_slew,float(max(abs(pwm-applied))/DT))
            peak_pwm=max(peak_pwm,float(max(abs(pwm))))
            max_integral=max(max_integral,max(abs(x) for x in o['integral']))
            applied=pwm.copy() # PWM register readback, NOT delayed/hidden motor torque
            motor=np.zeros(3)
            for i,q in enumerate(delays):q.append(pwm[i]);motor[i]=q.popleft()
            if not o['armed']:motor[:]=0
            if not ended:
                if plant:
                    mu=c.mu*(.03 if c.event=='slip' and t>=4 else 1)
                    plant.set_mu(mu)
                    supply=.65 if c.event=='sag' and t>=4 else 1.
                    adhesion=c.adhesion*(.125 if c.event=='magnet' and t>=4 else 1.)
                    push=(-.8,0.) if c.event=='push' and 4<=t<6 else (0.,0.)
                    plant.step(motor,adhesion=adhesion,supply=supply,weak=c.weak,push=push,coast=not o['armed'])
                    world=plant.pose();truth=np.array([world[1],-world[0],math.atan2(math.sin(world[2]-math.pi/2),math.cos(world[2]-math.pi/2))])
                    diag=plant.diagnostics()
                    if abs(truth[0])>.6 or abs(truth[1])>.6 or diag['tilt_deg']>65 or diag['height_m']>.12:ended=True
                    if not np.isfinite(plant.d.qpos).all():raise RuntimeError('Nonfinite physics')
                else:
                    for i in range(3):
                        m=MODELS[i][int(motor[i]<0)]
                        k=m['gain_cps_per_pwm'];tau=m['tau_s']*c.tau[i];ks=max(0,-m['bias_cps']/k)
                        supply=.65 if c.event=='sag' and t>=4 else 1.
                        drive=np.sign(motor[i])*max(0,abs(motor[i])*supply-ks)*c.gain[i]
                        load=c.load[i] if t>=.5 else 0.
                        if c.event=='load_release' and t>=6:load=0
                        a=math.exp(-DT/tau)
                        if c.event=='jam' and t>=1.5 and i==0:velocity[i]=0
                        elif c.stiction and abs(velocity[i])<30 and abs(drive-load)<(155 if i==0 else 135)-ks:velocity[i]=0
                        else:velocity[i]=a*velocity[i]+(1-a)*k*(drive-load)
                    position+=velocity*DT
                    step=np.array([(velocity[2]-velocity[0])*MPC/math.sqrt(3),
                                   (velocity[0]-2*velocity[1]+velocity[2])*MPC/3,
                                   sum(velocity)*MPC/(3*.033738)])*DT
                    mid=truth[2]+step[2]/2;cs,sn=math.cos(mid),math.sin(mid)
                    truth+=np.array([cs*step[0]-sn*step[1],sn*step[0]+cs*step[1],step[2]])
            trace.append(dict(t=round(t+DT,4),truth=truth.tolist(),task=task.tolist(),speed=filt.tolist(),**o))
    finally:controller.close()
    tail=trace[-100:];travel=[f for f in trace if 1<=f['t']<3]
    # Orthogonal deviation is meaningful for pure translations only.
    cross=max(abs(f['truth'][1-c.axis]) for f in travel) if c.axis<2 else 0.
    metrics=dict(final_task_error_mm=float(np.linalg.norm(truth[:2]-task[:2])*1000),
                 final_encoder_error_mm=float(np.linalg.norm(np.array(o['estimate'][:2])-o['goal'][:2])*1000),
                 tail_task_error_mm=float(np.mean([np.linalg.norm(np.array(f['truth'][:2])-f['task'][:2])*1000 for f in tail])),
                 cross_track_peak_mm=float(cross*1000),peak_pwm=peak_pwm,max_integral=max_integral,max_slew=max_slew,
                 tail_speed_cps=max(abs(x) for f in tail for x in f['speed']),
                 tail_pwm_mean=float(np.mean([max(abs(x) for x in f['pwm']) for f in tail])),
                 final_heading_error_deg=math.degrees(abs(math.atan2(math.sin(truth[2]-task[2]),math.cos(truth[2]-task[2])))),
                 tail_position_range_mm=float(max(np.linalg.norm(np.array(f['truth'][:2])-truth[:2])*1000 for f in tail)),
                 faults=sorted(faults),post_fault_nonzero=post_fault,ended=ended,
                 solver_warnings=int(sum(w.number for w in plant.d.warning)) if plant else 0)
    return dict(case=asdict(c),variant=variant,metrics=metrics,trace=trace[::5])


def holdout_cases(offset=0):
    """Frozen before evaluation: randomized finite envelope, not all conditions."""
    out=[]
    for index in range(48):
        seed=9300+offset+index
        r=np.random.default_rng(seed);axis=int(r.integers(0,2))
        speed=float(r.choice([-.08,-.04,.04,.08]));delay=tuple(int(x) for x in r.integers(0,6,3))
        if index<24:
            out.append(Case(f'wheel_seed_{seed}',seed=seed,axis=axis,speed=speed,delay=delay,
                            gain=tuple(float(x) for x in r.uniform(.8,1.15,3)),
                            tau=tuple(float(x) for x in r.uniform(.85,1.3,3)),
                            load=tuple(float(x) for x in r.uniform(-20,20,3)),seconds=15))
        else:
            out.append(Case(f'floor_seed_{seed}',plant='mujoco',seed=seed,axis=axis,speed=speed,delay=delay,
                            mass=float(r.uniform(.25,.45)),mu=float(r.uniform(.55,.85)),weak=float(r.uniform(.8,1.)),seconds=12))
    for seed in range(9400+offset,9412+offset):
        r=np.random.default_rng(seed)
        out.append(Case(f'wall_seed_{seed}',plant='mujoco',seed=seed,incline=90,
                        mass=float(r.uniform(.25,.4)),mu=float(r.uniform(.55,.85)),adhesion=float(r.uniform(10,16)),
                        delay=tuple(int(x) for x in r.integers(0,4,3)),event='hold_only',seconds=15))
    return out


def cases(plant):
    if plant=='wheels':
        return [Case('nominal'),Case('lateral',axis=1),Case('yaw',axis=2,speed=.6),
                Case('unequal_delays',gain=(.8,1.15,1),tau=(1.3,.8,1.1),delay=(0,3,7)),
                Case('lateral_delays',axis=1,delay=(7,0,3)),Case('stiction',stiction=True,seconds=25),
                Case('constant_load',load=(-35,20,15)),Case('hold_under_load',event='hold_only',load=(-35,20,15)),
                Case('load_release',load=(-35,20,15),event='load_release'),Case('sag',event='sag'),
                Case('reverse',event='reverse'),Case('jam',event='jam',axis=0),
                Case('lease_loss',event='lease_loss'),Case('encoder_invalid',event='encoder_invalid'),
                Case('encoder_stuck',event='encoder_stuck')]
    return [Case('floor',plant='mujoco'),Case('floor_lateral',plant='mujoco',axis=1),
            Case('floor_delays',plant='mujoco',delay=(0,3,7)),Case('floor_weak_m1',plant='mujoco',weak=.65),
            Case('external_push',plant='mujoco',event='push'),Case('supply_sag',plant='mujoco',event='sag'),
            Case('wall',plant='mujoco',incline=90,adhesion=12),
            Case('wall_hold',plant='mujoco',incline=90,adhesion=12,event='hold_only'),
            Case('wall_slip',plant='mujoco',incline=90,adhesion=12,event='slip'),
            Case('wall_magnet_loss',plant='mujoco',incline=90,adhesion=12,event='magnet')]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--plant',choices=['wheels','mujoco'],default='wheels')
    parser.add_argument('--report',type=Path,required=True);parser.add_argument('--jobs',type=int,default=4)
    parser.add_argument('--holdout',action='store_true')
    parser.add_argument('--seed-offset',type=int,default=0)
    parser.add_argument('--case');args=parser.parse_args()
    if args.report.exists():raise SystemExit('Refusing to overwrite existing evidence')
    exe,sources=build();selected=[c for c in (holdout_cases(args.seed_offset) if args.holdout else cases(args.plant)) if not args.case or c.name==args.case]
    if not selected:raise SystemExit('No matching case')
    work=[(c,v,exe) for c in selected for v in ['hold6','servo']]
    # Record cases, controller identity and acceptance before running any trial.
    manifest=args.report.with_suffix('.manifest.json')
    acceptance=dict(final_task_error_mm=10,cross_track_peak_mm=5,final_heading_error_deg=5,
                    tail_position_range_mm=3,required_no_fault=True,required_no_end=True,
                    wall='Exploratory; no qualification claim. Score with the same bounds and retain failures.')
    args.report.parent.mkdir(parents=True,exist_ok=True)
    with manifest.open('x') as f:json.dump(dict(executable=str(exe),cases=[asdict(c) for c in selected],acceptance=acceptance),f,indent=2)
    results=[]
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for result in pool.map(run,work):
            results.append(result);m=result['metrics']
            m['within_motion_bounds']=not m['faults'] and not m['ended'] and all(m[k]<=acceptance[k] for k in
                ('final_task_error_mm','cross_track_peak_mm','final_heading_error_deg','tail_position_range_mm'))
            print(result['case']['name'],result['variant'],f"task={m['final_task_error_mm']:.2f}mm cross={m['cross_track_peak_mm']:.2f}mm",m['faults'],flush=True)
    sources += [Path(__file__),Path(__file__).with_name('engine.py')]
    report=dict(scope='Simulation only; no hardware validation; encoder-only controller input',
                prospective_holdout=args.holdout,acceptance=acceptance,
                assumptions='Measured moving-range K/tau only. Other load/contact/torque/delay values are assumptions.',
                source_sha256={str(p.relative_to(ROOT.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},results=results)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    with args.report.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)


if __name__=='__main__':main()
