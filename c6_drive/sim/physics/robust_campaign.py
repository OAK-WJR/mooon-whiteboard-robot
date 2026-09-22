#!/usr/bin/env python3
"""Independent MuJoCo comparison of the portable C++ robust candidate.

The controller pipe receives sensors, commands and applied duty only. Fixture
parameters and truth state remain in the plant, sensor generator and scorer.
"""
import argparse
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
import numpy as np
import mujoco
from engine import Plant, Wheels, ROOT, DT, MPC, RAD_PER_COUNT, RW, kin, motor_constants, SCENARIOS


def build_native(sanitize=False):
    header=ROOT/'include/robust_control.h'
    driver=Path(__file__).with_name('robust_bridge.cpp')
    digest=hashlib.sha256(header.read_bytes()+driver.read_bytes()+str(sanitize).encode()).hexdigest()[:18]
    out=Path(tempfile.gettempdir())/f'wbb-robust-{digest}'
    if not out.exists():
        flags=['-O1','-g','-fsanitize=address,undefined'] if sanitize else ['-O2']
        subprocess.run(['c++','-std=c++17',*flags,'-Wall','-Wextra','-Werror','-I',str(ROOT/'include'),str(driver),'-o',str(out)],check=True)
    return out


class Native:
    def __init__(self, *, observer=True, tuning=(.20,.7)):
        self.proc=subprocess.Popen([str(build_native()),str(tuning[0]),str(tuning[1]),str(int(observer))],
                                   stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)

    def update(self,t,command,speed,applied,sensor,valid=True,stop=False,command_stamp=None):
        stamp,pose=(sensor[0],sensor[1]) if sensor is not None else (0.,np.zeros(3))
        values=[t,DT,t if command_stamp is None else command_stamp,*command,*speed,*applied,int(valid),int(sensor is not None),stamp,*pose,int(stop)]
        # Encoder filtering can decay below float32's representable range after
        # a stopped trial. This is physically zero, not malformed controller data.
        number=lambda x:'0' if abs(x)<1e-12 else format(float(x),'.11g')
        line=' '.join(number(x) for x in values[:12])+' '+str(int(valid))+' '+str(int(sensor is not None))+' '+\
             ' '.join(number(x) for x in [stamp,*pose])+' '+str(int(stop))+'\n'
        self.proc.stdin.write(line)
        self.proc.stdin.flush()
        row=self.proc.stdout.readline().split()
        if len(row)!=22:raise RuntimeError(f'Native controller stopped: exit={self.proc.poll()} row={row} input={line!r}')
        v=[float(x) for x in row[:18]]
        return {'pwm':[int(x) for x in v[:3]],'targets':v[3:6],'comp':v[6:9],'integral':v[9:12],
                'estimated_pose':v[12:15],'goal':v[15:18],'status':row[18],
                'fault':row[18] if int(row[19]) else None,'slip':bool(int(row[20])),'scale':float(row[21])}

    def close(self):
        self.proc.stdin.close();self.proc.wait(timeout=5);self.proc.stdout.close()


class Baseline:
    """The previous model/ESO controller and supervisor, without truth access."""
    def __init__(self,variant):
        self.wheels=Wheels(variant);self.goal=np.array([0.,0.,math.pi/2]);self.fault=None
        self.slip_time=self.saturation_time=0.

    def update(self,t,command,speed,applied,sensor,valid=True,stop=False,command_stamp=None):
        self.goal+=command*DT
        if not valid:self.fault='ENCODER_INVALID'
        if sensor is None and t>.35:self.fault='POSITION_FEEDBACK_LOST'
        if command_stamp is not None and t-command_stamp>.3:self.fault='COMMAND_STALE'
        if stop:self.fault='EMERGENCY_STOP'
        yaw=sensor[1][2] if sensor else math.pi/2;A=kin(yaw)
        residual=speed*MPC-A@sensor[2] if sensor else np.zeros(3)
        self.slip_time=self.slip_time+DT if sensor is not None and max(abs(residual))>.08 else max(0.,self.slip_time-DT*2)
        slip=self.slip_time>.2
        self.saturation_time=self.saturation_time+DT if max(abs(applied))>=250 else max(0.,self.saturation_time-DT)
        limited=self.saturation_time>.25
        desired=command.copy()
        if sensor:
            error=self.goal-sensor[1];error[2]=math.atan2(math.sin(error[2]),math.cos(error[2]));desired+=2*error
        desired[:2]=np.clip(desired[:2],-.3,.3);desired[2]=np.clip(desired[2],-1.2,1.2)
        if slip or limited:
            desired*=.35 if slip else .6
            if sensor:self.goal=sensor[1].copy()
        target=A@desired/MPC;target*=min(1.,10000/max(1.,max(abs(target))))
        out=self.wheels.call(dict(targets=target.tolist(),measured=speed.tolist(),hold=True,learn=not(slip or limited),rate=3000 if slip else 12000,stop=bool(self.fault)))
        out.update(goal=self.goal.tolist(),estimated_pose=sensor[1].tolist() if sensor else [0,0,math.pi/2],
                   integral=[0,0,0],fault=self.fault,slip=slip,scale=.35 if slip else .6 if limited else 1.,
                   status=self.fault or ('SLIP_DERATE' if slip else 'OUTPUT_LIMITED' if limited else 'TRACKING' if np.any(command) else 'HOLD'))
        return out

    def close(self):self.wheels.close()


class CampaignPlant(Plant):
    def step_case(self,pwm,case,t,fault=False):
        mu=case.mu*(.10 if case.event=='slip' and t>=4 else 1.)
        self.set_mu(mu)
        supply=case.supply*(.65 if case.event=='sag' and t>=3 else .3 if case.event=='saturation' and t>=3 else 1.)
        adhesion=case.adhesion*(.125 if case.event=='magnet' and t>=4 else 1.)
        push=np.array(case.push) if 3<=t<6 else np.zeros(2)
        for _ in range(round(DT/self.m.opt.timestep)):
            self.d.xfrc_applied[:]=0
            gap=max(0.,self.d.xpos[self.body,2]-RW-.001)
            self.d.xfrc_applied[self.body,:3]=[*push,-adhesion*math.exp(-gap/.005)]
            for i,u in enumerate(pwm):
                omega=self.d.qvel[self.axles[i]]
                _,_,ks,ct,damping=motor_constants(i,u or omega)
                effective=np.sign(u)*max(0.,abs(u)*supply-ks)
                drag=.15 if fault and abs(u)<.5 else 1.
                self.d.ctrl[i]=np.clip(ct*effective*case.torque[i]-drag*damping*omega*case.drag[i],-.2,.2)
            mujoco.mj_step(self.m,self.d)
        mujoco.mj_forward(self.m,self.d)


@dataclass
class Case:
    name:str='floor'
    seed:int=42
    mass:float=.30
    mu:float=.65
    incline:float=0.
    adhesion:float=0.
    supply:float=1.
    torque:tuple=(1.,1.,1.)
    drag:tuple=(1.,1.,1.)
    inertia:tuple=(1.,1.,1.)
    camera_delay:float=.08
    camera_noise:float=.0002
    encoder_noise:float=0.
    drop_probability:float=0.
    actuation_delay:int=0
    speed:float=.14
    motion:str='lateral'
    event:str='none'
    push:tuple=(0.,0.)
    expected_failure:bool=False


class Sensor:
    def __init__(self,case):
        self.c=case;self.rng=np.random.default_rng(case.seed);self.queue=[];self.last=None;self.next=0.;self.outlier=False

    def read(self,pose,t):
        c=self.c;available=not(c.event=='camera' and t>=4)
        if available and t+1e-8>=self.next:
            self.next=t+.05
            noise=self.rng.uniform(-1,1,3)*[c.camera_noise,c.camera_noise,.001]
            p=pose+noise
            if c.event=='outlier' and 3.5<=t and not self.outlier:
                p[0]+=.30;self.outlier=True
            if self.rng.random()>=c.drop_probability:self.queue.append((t,p))
        while self.queue and self.queue[0][0]+c.camera_delay<=t+1e-8:
            stamp,p=self.queue.pop(0)
            vel=(p-self.last[1])/(stamp-self.last[0]) if self.last else np.zeros(3)
            if self.last:vel[2]=math.atan2(math.sin(p[2]-self.last[1][2]),math.cos(p[2]-self.last[1][2]))/(stamp-self.last[0])
            self.last=(stamp,p,vel)
        return self.last if self.last and t-self.last[0]<.30 else None


def command_at(case,t):
    if case.motion=='vertical':return np.array([0.,case.speed if 1<=t<3 else 0.,0.])
    if case.motion=='diagonal':return np.array([case.speed*.707,case.speed*.707,.25])*(1 if 1<=t<3 else -1 if 4<=t<6 else 0)
    return np.array([case.speed if 1<=t<3 else -case.speed if 4<=t<6 else 0.,0.,0.])


def run_case(case,variant,seconds=8,tuning=(.20,.7),save_trace=False):
    p=CampaignPlant(mass=case.mass,mu=case.mu,incline=case.incline)
    p.m.dof_armature[p.axles]*=case.inertia
    c=Native(observer=variant!='robust_no_eso',tuning=tuning) if variant.startswith('robust') else Baseline(variant)
    sensor=Sensor(case);encoder_rng=np.random.default_rng(case.seed+900000)
    filt=np.zeros(3);previous=np.round(p.d.qpos[p.angles]/RAD_PER_COUNT)
    applied=np.zeros(3);queue=[];task_goal=np.array([0.,0.,math.pi/2])
    trace=[];ended=False;termination=None;statuses=set()
    try:
        for n in range(round(seconds/DT)):
            t=n*DT;command=command_at(case,t);task_goal+=command*DT
            body=sensor.read(p.pose(),t)
            counts=np.round(p.d.qpos[p.angles]/RAD_PER_COUNT)
            instantaneous=(counts-previous)/DT;previous=counts
            if case.event=='stuck' and t>=4:instantaneous[0]=0
            filt+=.4*(instantaneous-filt)
            measured=filt+encoder_rng.uniform(-case.encoder_noise,case.encoder_noise,3)
            valid=not(case.event=='encoder' and t>=4)
            command_stamp=3.5 if case.event=='heartbeat' and t>=3.5 else t
            input_applied=applied.copy()
            out=c.update(t,command,measured,applied,body,valid=valid,stop=ended,command_stamp=command_stamp)
            requested=np.array(out['pwm'])
            queue.append(requested)
            if out['fault'] or ended:applied=np.zeros(3);queue=[]
            elif len(queue)>case.actuation_delay:applied=queue.pop(0)
            if not ended:p.step_case(applied,case,t,fault=bool(out['fault']))
            diag=p.diagnostics();pose=p.pose()
            if not np.isfinite(p.d.qpos).all() or not np.isfinite(p.d.qvel).all():raise RuntimeError('Nonfinite plant state')
            statuses.add(out['status'])
            position_error=float(np.linalg.norm(pose[:2]-task_goal[:2]))
            heading_error=abs(math.atan2(math.sin(pose[2]-task_goal[2]),math.cos(pose[2]-task_goal[2])))
            frame=dict(t=round(t+DT,4),pose=pose.tolist(),task_goal=task_goal.tolist(),
                       error_m=position_error,heading_error_deg=math.degrees(heading_error),
                       speed=measured.tolist(),requested_pwm=requested.tolist(),input_applied=input_applied.tolist(),
                       qpos=p.d.qpos.tolist() if save_trace else [],**out,**diag)
            # out['pwm'] is requested duty; keep the actual actuator separately.
            frame['pwm']=applied.tolist();trace.append(frame)
            if not ended and (pose[1]<-.6 or diag['tilt_deg']>65 or diag['height_m']>.12):
                ended=True;termination='LEFT_TEST_ENVELOPE'
    finally:c.close()
    window=[f for f in trace if 1<=f['t']<8]
    post=[f for f in trace if 3<=f['t']<6]
    result={'case':asdict(case),'variant':variant,'metrics':{
        'mean_task_error_m':float(np.mean([f['error_m'] for f in window])),
        'post_event_mean_error_m':float(np.mean([f['error_m'] for f in post])),
        'peak_error_m':max(f['error_m'] for f in trace),'final_error_m':trace[-1]['error_m'],
        'mean_heading_error_deg':float(np.mean([f['heading_error_deg'] for f in window])),
        'peak_heading_error_deg':max(f['heading_error_deg'] for f in trace),
        'peak_pwm':max(abs(u) for f in trace for u in f['pwm']),
        'pwm_variation':float(sum(sum(abs(np.array(b['pwm'])-a['pwm'])) for a,b in zip(trace,trace[1:]))),
        'max_integral':max(abs(u) for f in trace for u in f['integral']),
        'max_compensation':max(abs(u) for f in trace for u in f['comp']),
        'max_command_slew_per_s':max(max(abs(np.array(f['requested_pwm'])-f['input_applied']))/DT for f in trace if not f['fault']),
        'post_fault_nonzero_requests':sum(any(abs(u)>.5 for u in f['requested_pwm']) for f in trace if f['fault']),
        'minimum_demand_scale':min(f['scale'] for f in trace),
        'ended_early':ended,'termination':termination,'faults':sorted({f['fault'] for f in trace if f['fault']}),
        'statuses':sorted(statuses),'solver_warnings':int(sum(w.number for w in p.d.warning))}}
    if save_trace:result['trace']=trace[::5]
    return result


def nominal_cases():
    cases=[]
    for name,cfg in SCENARIOS.items():
        c=Case(name=name,mass=cfg['mass'],incline=cfg['incline'],adhesion=cfg['adhesion'],
               speed=.12 if cfg['incline'] else .14,motion='vertical' if cfg['incline'] else 'lateral')
        if name in ('sag','saturation','slip','magnet','camera','encoder'):c.event=name
        if name=='load':c.push=(-.8,0.)
        if name=='weak':c.torque=(.65,1.,1.)
        c.expected_failure=name in ('slip','magnet','camera','encoder','saturation')
        cases.append(c)
    cases.extend([Case(name='outlier',event='outlier'),Case(name='heartbeat',event='heartbeat',expected_failure=True),
                  Case(name='stuck',event='stuck',expected_failure=True),Case(name='diagonal',motion='diagonal')])
    return cases


def uncertain_cases(seed_start,count):
    cases=[]
    for seed in range(seed_start,seed_start+count):
        rng=np.random.default_rng(seed);incline=float(rng.choice([0.,45.,90.]))
        cases.append(Case(name=f'uncertain-{seed}',seed=seed,mass=float(rng.uniform(.22,.48)),mu=float(rng.uniform(.5,.85)),
                         incline=incline,adhesion=float(rng.uniform(14,18)) if incline else 0.,
                         supply=float(rng.uniform(.8,1.05)),torque=tuple(rng.uniform(.8,1.2,3)),
                         drag=tuple(rng.uniform(.85,1.15,3)),inertia=tuple(rng.uniform(.75,1.35,3)),
                         camera_delay=float(rng.uniform(.04,.15)),camera_noise=float(rng.uniform(.0002,.0008)),
                         encoder_noise=float(rng.uniform(10,50)),drop_probability=float(rng.uniform(0,.08)),
                         actuation_delay=int(rng.integers(0,2)),speed=float(rng.uniform(.09,.17)),
                         motion='vertical' if incline else str(rng.choice(['lateral','diagonal'])),
                         push=(float(rng.uniform(-.3,.3)),float(rng.uniform(-.3,.3)))))
    return cases


def qualification_cases(seed_start,count):
    # Prospective operating envelope. The wider stress distribution above is
    # retained separately; it includes combinations that lack drive authority.
    cases=uncertain_cases(seed_start,count)
    for c in cases:
        rng=np.random.default_rng(c.seed+100000)
        c.mass=float(rng.uniform(.22,.30));c.mu=float(rng.uniform(.60,.85))
        c.adhesion=float(rng.uniform(12,15)) if c.incline else 0.
        c.supply=float(rng.uniform(.90,1.05));c.torque=tuple(rng.uniform(.90,1.15,3))
        c.drag=tuple(rng.uniform(.90,1.10,3));c.inertia=tuple(rng.uniform(.80,1.30,3))
        c.camera_delay=float(rng.uniform(.04,.12));c.speed=float(rng.uniform(.08,.12))
        c.push=(float(rng.uniform(-.2,.2)),float(rng.uniform(-.2,.2)))
    return cases


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--suite',choices=['nominal','development','holdout','stress'],default='nominal')
    ap.add_argument('--variants',nargs='+',default=['model','eso','robust'])
    ap.add_argument('--case');ap.add_argument('--count',type=int,default=60)
    ap.add_argument('--lambda',dest='lam',type=float,default=.20)
    ap.add_argument('--acceleration-ff',type=float,default=.7)
    ap.add_argument('--output',default=None)
    args=ap.parse_args()
    cases=nominal_cases() if args.suite=='nominal' else uncertain_cases(1100,12) if args.suite=='development' else qualification_cases(8100,args.count) if args.suite=='holdout' else uncertain_cases(9100,args.count)
    if args.case:cases=[c for c in cases if c.name==args.case]
    if not cases:raise ValueError('No matching cases')
    started=time.monotonic();results=[]
    for case in cases:
        for variant in args.variants:
            r=run_case(case,variant,tuning=(args.lam,args.acceleration_ff),save_trace=args.suite=='nominal')
            results.append(r);m=r['metrics']
            print(f"{case.name:18} {variant:14} error={m['mean_task_error_m']*1000:6.1f}mm yaw={m['mean_heading_error_deg']:5.1f}deg faults={m['faults']} ended={m['ended_early']}",flush=True)
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'engine_version':mujoco.__version__,
            'suite':args.suite,'tuning':{'lambda':args.lam,'acceleration_ff':args.acceleration_ff},
            'controller_sha256':hashlib.sha256((ROOT/'include/robust_control.h').read_bytes()).hexdigest(),
            'harness_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'elapsed_seconds':time.monotonic()-started,'simulation_only':True,
            'results':[{k:v for k,v in r.items() if k!='trace'} for r in results]}
    dest=Path(args.output) if args.output else ROOT/f'reports/robust-{args.suite}.json'
    dest.write_text(json.dumps(report,indent=2)+'\n')
    if args.suite=='nominal':dest.with_name(dest.stem+'-traces.json').write_text(json.dumps(results))
    print('Saved',dest,flush=True)


if __name__=='__main__':main()
