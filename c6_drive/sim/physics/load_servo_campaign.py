#!/usr/bin/env python3
"""Build the portable load servo with the existing Control wrapper; no hardware.

The local bridge aliases only the wrapper's servo type, after both class
definitions are included. Archived production/default headers stay unchanged.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np

from encoder_campaign import build, cases as wheel_cases, holdout_cases
from load_campaign import cases as load_cases, Case as LoadCase, ACCEPTANCE
from startup_experiment import evaluate
from engine import ROOT


def build_load():
    baseline,sources=build()
    sources += [ROOT/'include/encoder_load_servo.h']
    digest=hashlib.sha256(b''.join(p.read_bytes() for p in sources)).hexdigest()[:18]
    folder=Path(tempfile.gettempdir())/f'wbb-load-servo-{digest}';folder.mkdir(exist_ok=True)
    prefix='#include "encoder_servo.h"\n#include "encoder_load_servo.h"\n#define EncoderServo EncoderLoadServo\n'
    bridge=folder/'bridge.cpp';bridge.write_text(prefix+(ROOT/'sim/physics/encoder_bridge.cpp').read_text())
    exe=folder/'controller'
    subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',str(ROOT/'include'),str(bridge),'-o',str(exe)],check=True)
    return baseline,exe,folder,sources,prefix


def independent_cases(offset=0):
    cases=[]
    # New seeds/masses/directions, declared before execution after development.
    for seed in range(17100+offset,17136+offset):
        rng=np.random.default_rng(seed)
        cases.append(('load',LoadCase(f'load_unseen_{seed}',mass=float(rng.uniform(.5,2.3)),
            profile=str(rng.choice(['free','bearings','pad','high_drag'])),axis=int(rng.integers(0,2)),
            speed=float(rng.choice([-.08,-.04,.04,.08])),motor_force_N=float(rng.uniform(2.5,5.)),
            mu=float(rng.uniform(.55,.85)),delay=tuple(int(x) for x in rng.integers(0,8,3)))))
    cases += [('wheels',c) for c in holdout_cases(600+offset)]
    return cases


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--jobs',type=int,default=6);parser.add_argument('--holdout',action='store_true')
    parser.add_argument('--regression',action='store_true',help='Re-run a previously seen randomized set; not a new holdout')
    parser.add_argument('--seed-offset',type=int,default=0)
    parser.add_argument('--case');args=parser.parse_args()
    if args.report.exists():raise SystemExit('Refusing overwrite')
    baseline,candidate,folder,sources,prefix=build_load()
    test=folder/'test.cpp';test.write_text(prefix+(ROOT/'tests/test_encoder_servo.cpp').read_text())
    subprocess.run(['c++','-std=c++17','-fsanitize=address,undefined','-I',str(ROOT/'include'),str(test),'-o',str(folder/'test')],check=True)
    subprocess.run([str(folder/'test')],check=True)
    if args.holdout and args.regression:raise SystemExit('Choose holdout or regression, not both')
    selected=independent_cases(args.seed_offset) if args.holdout or args.regression else [('load',c) for c in load_cases()]+[('wheels',c) for c in wheel_cases('wheels')]
    selected=[(f,c) for f,c in selected if not args.case or c.name==args.case]
    if not selected:raise SystemExit('No cases')
    sources += [Path(__file__),ROOT/'tests/test_encoder_servo.cpp',Path(__file__).with_name('startup_experiment.py'),
                Path(__file__).with_name('encoder_campaign.py'),Path(__file__).with_name('load_campaign.py'),
                Path(__file__).with_name('load_plant.py'),Path(__file__).with_name('engine.py')]
    hashes={str(p.relative_to(ROOT.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    manifest=dict(scope='Encoder-only native C++ load servo, software/simulation only',holdout=args.holdout,
                  regression=args.regression,seed_offset=args.seed_offset,
                  acceptance=ACCEPTANCE,source_sha256=hashes,
                  candidate_executable_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
                  bridge_source=(folder/'bridge.cpp').read_text(),cases=[dict(family=f,case=asdict(c)) for f,c in selected])
    with args.report.with_suffix('.manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
    results=[];work=[(f,c,v,e) for f,c in selected for v,e in [('fixed',baseline),('load_servo',candidate)]]
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for result in pool.map(evaluate,work):
            m=result['metrics']
            m['within_motion_bounds']=not m['faults'] and not m['ended'] and not m['solver_warnings'] and all(
                m[k]<=ACCEPTANCE[k] for k in ('final_task_error_mm','cross_track_peak_mm','final_heading_error_deg','tail_position_range_mm'))
            results.append(result)
            print(result['family'],result['case']['name'],result['variant'],
                  f"pass={m['within_motion_bounds']} task={m['final_task_error_mm']:.1f} cross={m['cross_track_peak_mm']:.1f}",m['faults'],flush=True)
    for name,digest in hashes.items():
        assert hashlib.sha256((ROOT.parent/name).read_bytes()).hexdigest()==digest,'Source changed during study'
    with args.report.open('x') as f:json.dump(dict(**manifest,results=results),f,indent=2,allow_nan=False)


if __name__=='__main__':main()
