#!/usr/bin/env python3
"""Native-only experiment: ramp a failed breakaway attempt within its deadline.

The published firmware headers and previous evidence stay unchanged. An exact,
checked source transformation is compiled in a temporary include directory. It
changes only the pulse request during demanded travel; no longer timeout, new
power ceiling, hardware transport or implicit deployment is introduced.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

from encoder_campaign import build, cases as wheel_cases, run as run_wheels
from load_campaign import cases as load_cases, run as run_load, ACCEPTANCE
from engine import ROOT

OLD='''                request=sign>0?std::fmax(raw,breakaway):std::fmin(raw,-breakaway);'''
NEW='''                // Experimental travel-only ramp within the original 250 ms
                // attempt. Holding behavior and the fault deadline are unchanged.
                float age=clip((.25f-recoveryTime[i])/.25f,0,1);
                float kick=holding?breakaway:breakaway+(ceiling-breakaway)*age;
                request=sign>0?std::fmax(raw,kick):std::fmin(raw,-kick);'''


def build_ramp():
    baseline,sources=build()
    source=(ROOT/'include/encoder_servo.h').read_text()
    if source.count(OLD)!=1: raise RuntimeError('Archived startup implementation changed; review experiment')
    transformed=source.replace(OLD,NEW)
    digest=hashlib.sha256(b''.join(p.read_bytes() for p in sources)+transformed.encode()).hexdigest()
    folder=Path(tempfile.gettempdir())/f'wbb-startup-{digest[:18]}'
    folder.mkdir(exist_ok=True)
    for p in sources:
        (folder/p.name).write_bytes(p.read_bytes())
    (folder/'encoder_servo.h').write_text(transformed)
    executable=folder/'controller'
    if not executable.exists():
        subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I',str(folder),
                        str(folder/'encoder_bridge.cpp'),'-o',str(executable)],check=True)
    return baseline,executable,folder,sources


def evaluate(job):
    family,case,variant,executable=job
    result=(run_load if family=='load' else run_wheels)((case,executable) if family=='load' else (case,'servo',executable))
    result['family']=family;result['variant']=variant
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--report',type=Path,required=True);p.add_argument('--jobs',type=int,default=6)
    args=p.parse_args()
    if args.report.exists(): raise SystemExit('Refusing to overwrite evidence')
    baseline,ramp,folder,sources=build_ramp()
    # Safety regression against the actual command/lease/fault wrapper, with the
    # transformed header. Includes frozen encoder, STOP, invalid data and hold.
    safety=folder/'test_encoder_servo.cpp';shutil.copyfile(ROOT/'tests/test_encoder_servo.cpp',safety)
    test=folder/'test-servo'
    subprocess.run(['c++','-std=c++17','-fsanitize=address,undefined','-I',str(folder),str(safety),'-o',str(test)],check=True)
    subprocess.run([str(test)],check=True)
    work=[(family,c,variant,exe) for family,cs in [('load',load_cases()),('wheels',wheel_cases('wheels'))]
          for c in cs for variant,exe in [('fixed',baseline),('ramp',ramp)]]
    sources += [Path(__file__),Path(__file__).with_name('load_campaign.py'),Path(__file__).with_name('load_plant.py'),
                Path(__file__).with_name('encoder_campaign.py'),Path(__file__).with_name('engine.py')]
    hashes={str(s.relative_to(ROOT.parent)):hashlib.sha256(s.read_bytes()).hexdigest() for s in sources}
    manifest=dict(scope='Development experiment only. No firmware modification or hardware access.',
                  acceptance=ACCEPTANCE,source_sha256=hashes,
                  generated_header_sha256=hashlib.sha256((folder/'encoder_servo.h').read_bytes()).hexdigest(),
                  baseline_executable_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest(),
                  ramp_executable_sha256=hashlib.sha256(ramp.read_bytes()).hexdigest(),
                  transformation=dict(before=OLD,after=NEW),
                  cases=[dict(family=f,case=c.__dict__,variant=v) for f,c,v,_ in work])
    with args.report.with_suffix('.manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
    results=[]
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for result in pool.map(evaluate,work):
            m=result['metrics'];results.append(result)
            print(result['family'],result['case']['name'],result['variant'],
                  f"task={m['final_task_error_mm']:.1f} cross={m['cross_track_peak_mm']:.1f}",m['faults'],flush=True)
    for name,digest in hashes.items():
        assert hashlib.sha256((ROOT.parent/name).read_bytes()).hexdigest()==digest,'Source changed during test'
    with args.report.open('x') as f:json.dump(dict(**manifest,results=results),f,indent=2,allow_nan=False)


if __name__=='__main__': main()
