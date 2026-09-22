#!/usr/bin/env python3
"""Run the independent MuJoCo scenario matrix. No robot interfaces."""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import mujoco
from engine import ROOT, SCENARIOS, make_xml, run


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', choices=list(SCENARIOS))
    args = ap.parse_args()
    chosen = [args.scenario] if args.scenario else list(SCENARIOS)
    results = []
    for scenario in chosen:
        for variant in ('model', 'eso'):
            result = run(scenario, variant)
            results.append(result)
            print(scenario, variant, json.dumps(result['metrics']), flush=True)
    report = {
        'created_utc': datetime.now(timezone.utc).isoformat(), 'engine': 'MuJoCo',
        'engine_version': mujoco.__version__, 'simulation_only': True,
        'model_sha256': hashlib.sha256((ROOT/'reports/sysid-model-03.json').read_bytes()).hexdigest(),
        'physics_source_sha256': hashlib.sha256(Path(__file__).with_name('engine.py').read_bytes()).hexdigest(),
        'controller_source_sha256': hashlib.sha256(Path(__file__).parents[1].joinpath('core.mjs').read_bytes()).hexdigest(),
        'dt_physics_s': .001, 'dt_controller_s': .01,
        'assumptions': ['Approximate geometry; 38 mm wheels, 33.738 mm center radius and 2800 counts/rev are inherited, unverified.',
            'Mass, roller geometry, friction, force/PWM coefficient, reflected inertia and magnetic pull are not measured.',
            'Measured speed dynamics include filtering. Mechanical inertia is not independently identifiable from these logs.',
            '3 force-driven hubs, 36 passive roller hinges, free six-DOF chassis; no prescribed chassis velocity.',
            'Only tire-to-plane collision is enabled. Chassis collisions, electrical current, heat and detachment material physics are omitted.',
            'Simulated camera: 20 Hz, 80 ms delay, bounded 0.2 mm / 0.001 rad noise; unavailable on current hardware.',
            'Trial terminates below -0.6 m, at >65 deg tilt or >0.12 m separation. This is not a physical catch.',
            'Metrics use the original task reference, including when the supervisor reduces achievable motion.'],
        'scenarios': [{k:v for k,v in r.items() if k != 'trace'} for r in results]}
    suffix = '-'+args.scenario if args.scenario else ''
    (ROOT/f'reports/mujoco-results{suffix}.json').write_text(json.dumps(report, indent=2)+'\n')
    # 20 Hz playback. Computation and metrics above always use the full 100 Hz trace.
    reduced = [{**r, 'trace':r['trace'][::5]} for r in results]
    (ROOT/f'reports/mujoco-traces{suffix}.json').write_text(json.dumps(reduced))
    Path(__file__).with_name('robot.xml').write_text(make_xml())


if __name__ == '__main__':
    main()
