#!/usr/bin/env python3
"""Prospective heavy-load sweep using the unchanged native encoder servo.

No hardware or network. The C++ controller sees encoder increments and PWM only;
contact, mass, torque and chassis truth are restricted to the plant and scorer.
"""
import argparse
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import mujoco
from encoder_campaign import Native, build
from engine import DT, RAD_PER_COUNT, ROOT
from load_plant import LoadPlant, Resistance

PROFILES = {
    'free': Resistance(),
    'bearings': Resistance(axle_Nm=(.008,.006,.006), roller_Nm=.00001),
    'pad': Resistance(axle_Nm=(.008,.006,.006), roller_Nm=.00001, pad_preload_N=2),
    'high_drag': Resistance(axle_Nm=(.030,.020,.020), roller_Nm=.00004, pad_preload_N=5),
}
ACCEPTANCE = dict(final_task_error_mm=10, cross_track_peak_mm=5,
                  final_heading_error_deg=5, tail_position_range_mm=3,
                  required_no_fault=True, required_no_end=True, required_no_solver_warning=True)


@dataclass(frozen=True)
class Case:
    name: str
    mass: float
    profile: str
    axis: int = 0
    speed: float = .08
    motor_force_N: float = 4.
    mu: float = .65
    delay: tuple = (0,3,7)
    seconds: float = 8.


def cases():
    out = [Case(f'{mass:g}kg_{p}_{"xy"[axis]}_{speed:g}', mass, p, axis, speed)
           for mass in (.5,1.,2.) for p in PROFILES for axis in (0,1) for speed in (.08,.04)]
    out += [Case(f'torque2N_1kg_{p}_{"xy"[axis]}', 1., p, axis, motor_force_N=2)
            for p in ('free','pad','high_drag') for axis in (0,1)]
    return out


def run(job):
    case, exe = job
    settings = asdict(PROFILES[case.profile]); settings['motor_force_N'] = case.motor_force_N
    resistance = Resistance(**settings)
    plant = LoadPlant(mass=case.mass, mu=case.mu, resistance=resistance)
    controller = Native('servo', exe)
    counts = np.round(plant.d.qpos[plant.angles]/RAD_PER_COUNT)
    filtered = np.zeros(3); applied = np.zeros(3); task = np.zeros(3)
    queues = [deque([0.]*n) for n in case.delay]
    trace = []; ended = False; faults = set(); first_fault = None
    try:
        for n in range(round(case.seconds/DT)):
            t = n*DT
            command = np.zeros(3)
            if 1 <= t < 4: command[case.axis] = case.speed
            task += command * DT
            new = np.round(plant.d.qpos[plant.angles]/RAD_PER_COUNT)
            delta = new-counts; counts=new; filtered += .4*(delta/DT-filtered)
            out = controller.step(n,command,delta,filtered,applied,stop=ended)
            applied = np.array(out['pwm'])
            motor = np.zeros(3)
            for i,q in enumerate(queues): q.append(applied[i]); motor[i]=q.popleft()
            if not out['armed']:
                motor[:] = 0; faults.add(out['reason'])
                if first_fault is None: first_fault = dict(t=t,reason=out['reason'])
            if not ended:
                plant.step(motor,coast=not out['armed'])
            world=plant.pose()
            truth=[world[1],-world[0],math.atan2(math.sin(world[2]-math.pi/2),math.cos(world[2]-math.pi/2))]
            diagnostics = plant.diagnostics()
            if max(abs(truth[0]),abs(truth[1]))>.6 or diagnostics['tilt_deg']>65 or diagnostics['height_m']>.12:
                ended=True
            if not np.isfinite(plant.d.qpos).all(): raise RuntimeError('Nonfinite physics')
            trace.append(dict(t=round(t+DT,4),truth=truth,task=task.tolist(),speed=filtered.tolist(),
                              physics=diagnostics,**out))
    finally: controller.close()
    travel=[f for f in trace if 1<f['t']<=4];tail=trace[-100:]
    wheel = []
    for i in range(3):
        moving=[f for f in travel if abs(f['reference'][i])>100]
        onset=next((f['t']-1 for f in moving if abs(f['speed'][i])>100),None)
        wheel.append(dict(peak_pwm=max(abs(f['pwm'][i]) for f in trace),
                          full_duty_s=DT*sum(abs(f['pwm'][i])>=250 for f in travel),
                          reference_error_cps=float(np.mean([abs(f['reference'][i]-f['speed'][i]) for f in moving])) if moving else None,
                          motion_onset_s=onset))
    m=dict(final_task_error_mm=float(np.linalg.norm(np.array(truth[:2])-task[:2])*1000),
           cross_track_peak_mm=max(abs(f['truth'][1-case.axis]) for f in travel)*1000,
           final_heading_error_deg=abs(math.degrees(truth[2])),
           tail_position_range_mm=max(np.linalg.norm(np.array(f['truth'][:2])-truth[:2])*1000 for f in tail),
           final_encoder_error_mm=float(np.linalg.norm(np.array(out['estimate'][:2])-out['goal'][:2])*1000),
           max_odometry_gap_mm=max(np.linalg.norm(np.array(f['truth'][:2])-f['estimate'][:2])*1000 for f in trace),
           travel_completion_at_release=float(trace[399]['truth'][case.axis]/(.0+case.speed*3)),
           minimum_governor=min(f['scale'] for f in travel),
           pad_normal_mean_N=float(np.mean([f['physics']['pad_normal_N'] for f in travel])),
           pad_drag_mean_N=float(np.mean([f['physics']['pad_tangent_N'] for f in travel])),
           wheel_normal_mean_N=np.mean([f['physics']['wheel_normal_N'] for f in travel],axis=0).tolist(),
           wheels=wheel, faults=sorted(faults), first_fault=first_fault, ended=ended,
           post_fault_nonzero=sum(not f['armed'] and any(f['pwm']) for f in trace),
           solver_warnings=int(sum(w.number for w in plant.d.warning)))
    m['within_motion_bounds']=not faults and not ended and not m['solver_warnings'] and all(
        m[k]<=ACCEPTANCE[k] for k in ('final_task_error_mm','cross_track_peak_mm','final_heading_error_deg','tail_position_range_mm'))
    return dict(case=asdict(case),resistance=asdict(resistance),metrics=m,trace=trace[::5])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--jobs',type=int,default=6);parser.add_argument('--case')
    args=parser.parse_args();exe,sources=build()
    selected=[c for c in cases() if not args.case or c.name==args.case]
    if not selected: raise SystemExit('No matching cases')
    sources += [Path(__file__),Path(__file__).with_name('load_plant.py'),
                Path(__file__).with_name('encoder_campaign.py'),Path(__file__).with_name('engine.py')]
    hashes={str(p.relative_to(ROOT.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    manifest=dict(scope='Exploratory load sensitivity, simulation only; not a hardware qualification',
                  assumptions='All mass/torque/contact/pad parameters assumed. Measured unloaded moving K/tau only.',
                  cases=[asdict(c) for c in selected],profiles={k:asdict(v) for k,v in PROFILES.items()},
                  acceptance=ACCEPTANCE,source_sha256=hashes,mujoco_version=mujoco.__version__)
    if args.report.exists(): raise SystemExit('Refusing to overwrite evidence')
    args.report.parent.mkdir(parents=True,exist_ok=True)
    with args.report.with_suffix('.manifest.json').open('x') as f: json.dump(manifest,f,indent=2)
    results=[]
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for result in pool.map(run,[(c,exe) for c in selected]):
            results.append(result);m=result['metrics']
            print(result['case']['name'],f"pass={m['within_motion_bounds']} task={m['final_task_error_mm']:.1f} cross={m['cross_track_peak_mm']:.1f} gap={m['max_odometry_gap_mm']:.1f}",m['faults'],flush=True)
    for name,digest in hashes.items():
        if hashlib.sha256((ROOT.parent/name).read_bytes()).hexdigest()!=digest: raise RuntimeError('Source changed during study')
    with args.report.open('x') as f: json.dump(dict(**manifest,results=results),f,indent=2,allow_nan=False)


if __name__=='__main__': main()
