#!/usr/bin/env python3
"""Recompute declared two-second floor criteria from retained board-clock data.

Offline only. This never arms a robot or writes its calibration flags.
"""
import argparse
import json
from pathlib import Path
import statistics
from c6_floor_response_report import summarize


def score(path,criteria):
    trial=summarize(path)
    if not trial.get('motion_started',True):
        return dict(source=path.name,source_sha256=trial['source_sha256'],raw_sha256=trial['raw_sha256'],
            key=trial['key'],controller_revision=trial['controller_revision'],motion_started=False,
            clean_performance_evidence=trial['clean_performance_evidence'],diagnostic_passed=False,
            last_second=[],passed=False,error=trial['capture_error'])
    end=trial.get('motion_end_ms',trial['stop_ms'])
    rows=[r for r in trial['samples'] if end is not None and end-1000<=r[0]<end]
    enough=len(rows)>=90 and rows[-1][0]-rows[0][0]>=950
    wheels=[]
    for wheel in trial['wheels']:
        values=[r[wheel['motor']+1]/wheel['target_cps'] for r in rows]
        mean=statistics.mean(values) if values else None
        std=statistics.pstdev(values) if values else None
        peak=max(r[wheel['motor']+1]/wheel['target_cps'] for r in trial['samples']
                 if trial['stop_ms'] is None or r[0]<trial['stop_ms'])
        passed=bool(enough and abs(mean-1)<=criteria['last1000_mean_error'] and
                    std<=criteria['last1000_std_fraction'] and
                    wheel['tail_relative_error']<=criteria['tail200_relative_error'])
        wheels.append(dict(motor=wheel['motor'],mean_percent=None if mean is None else mean*100,
            std_percent=None if std is None else std*100,
            tail_percent=wheel['tail_mean_target_percent'],peak_percent=peak*100,passed=passed))
    return dict(source=path.name,source_sha256=trial['source_sha256'],raw_sha256=trial['raw_sha256'],
                key=trial['key'],controller_revision=trial['controller_revision'],
                clean_performance_evidence=trial['clean_performance_evidence'],
                diagnostic_passed=trial['motion_quality_passed'],last_second=wheels,
                passed=bool(trial['motion_quality_passed'] and enough and
                            trial['maximum_gap_ms']<=criteria['maximum_gap_ms'] and
                            all(w['passed'] for w in wheels)))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory',type=Path)
    args=ap.parse_args()
    manifest=json.loads((args.directory/'manifest.json').read_text())
    trials=[score(p,manifest['criteria']) for p in sorted(args.directory.glob('[0-9]*.json'))
            if not p.name.endswith('.annotation.json')]
    complete=len(trials)==len(manifest['directions'])
    result=dict(results=trials,complete=complete,passed=complete and all(t['passed'] for t in trials),
                calibration_flags_written=False,body_position_measured=False)
    (args.directory/'scored-results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
