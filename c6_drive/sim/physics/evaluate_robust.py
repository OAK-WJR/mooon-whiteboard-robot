#!/usr/bin/env python3
"""Audit saved prospective trials without refitting or discarding failed trials."""
import hashlib
import json
from pathlib import Path
from statistics import mean

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def aggregate(rows):
    return {
        'runs': len(rows),
        'mean_task_error_mm': mean(r['metrics']['mean_task_error_m'] * 1000 for r in rows),
        'worst_run_mean_task_error_mm': max(r['metrics']['mean_task_error_m'] * 1000 for r in rows),
        'mean_heading_error_deg': mean(r['metrics']['mean_heading_error_deg'] for r in rows),
        'peak_heading_error_deg': max(r['metrics']['peak_heading_error_deg'] for r in rows),
        'falls_or_envelope_exits': sum(r['metrics']['ended_early'] for r in rows),
        'faulted_runs': sum(bool(r['metrics']['faults']) for r in rows),
        'fault_cases': {r['case']['name']: r['metrics']['faults'] for r in rows if r['metrics']['faults']},
    }


def audit():
    acceptance = json.loads((ROOT/'sim/physics/robust-acceptance.json').read_text())
    reports = {s: json.loads((ROOT/f'reports/robust-{s}.json').read_text())
               for s in ['nominal', 'holdout', 'stress']}
    sources = [ROOT/'include/robust_control.h', ROOT/'sim/physics/robust_campaign.py',
               ROOT/'sim/physics/robust_bridge.cpp', ROOT/'sim/physics/engine.py',
               ROOT/'sim/core.mjs', ROOT/'reports/sysid-model-03.json',
               ROOT/'sim/physics/robust-acceptance.json']
    check_count = 0

    def check(condition, message):
        nonlocal check_count
        check_count += 1
        if not condition:
            raise AssertionError(message)

    limits = acceptance['per_run_limits']
    summaries = {}
    for suite, report in reports.items():
        check(report['simulation_only'], f'{suite}: missing simulation label')
        check(report['controller_sha256'] == digest(sources[0]), f'{suite}: stale controller')
        check(report['harness_sha256'] == digest(sources[1]), f'{suite}: stale harness')
        check(report['tuning'] == {'lambda': .2, 'acceleration_ff': .7}, f'{suite}: changed tuning')
        variants = ['model', 'eso', 'robust'] if suite == 'nominal' else ['eso', 'robust']
        by_variant = {v: [r for r in report['results'] if r['variant'] == v] for v in variants}
        expected = 16 if suite == 'nominal' else 60 if suite == 'holdout' else 30
        check(len(report['results']) == expected * len(variants), f'{suite}: missing/extra trials')
        for v, rows in by_variant.items():
            check(len(rows) == expected, f'{suite}/{v}: incomplete')
            check(len({r['case']['name'] for r in rows}) == expected, f'{suite}/{v}: duplicate cases')
            if suite != 'nominal':
                lo, hi = acceptance[f'{suite}_seeds']
                check({r['case']['seed'] for r in rows} == set(range(lo, hi+1)), f'{suite}: seeds')
        previous = {r['case']['name']: r for r in by_variant['eso']}
        regressions = []
        for r in by_variant['robust']:
            name, m = r['case']['name'], r['metrics']
            check(r['case'] == previous[name]['case'], f'{suite}/{name}: unpaired plant/sensors')
            for key in ['peak_pwm', 'max_integral', 'max_compensation', 'max_command_slew_per_s',
                        'post_fault_nonzero_requests', 'solver_warnings']:
                check(m[key] <= limits[key], f'{suite}/{name}: {key}={m[key]}')
            if suite == 'holdout':
                for key in ['mean_task_error_m', 'mean_heading_error_deg', 'peak_heading_error_deg']:
                    check(m[key] <= limits[key], f'{name}: qualification failed {key}')
                check(not m['faults'] and not m['ended_early'], f'{name}: unexpected fault/fall')
            if m['mean_task_error_m'] > previous[name]['metrics']['mean_task_error_m']:
                regressions.append(name)
        summary = {v: aggregate(rows) for v, rows in by_variant.items()}
        summary['position_error_reduction_fraction'] = 1 - summary['robust']['mean_task_error_mm']/summary['eso']['mean_task_error_mm']
        summary['position_regressions'] = regressions
        summary['position_improved_pairs'] = expected - len(regressions)
        summaries[suite] = summary
    check(summaries['holdout']['position_error_reduction_fraction'] >= acceptance['aggregate_target']['mean_position_error_reduction_vs_previous_eso_at_least'], 'Aggregate reduction failed')
    # Retain adverse evidence explicitly. A new controller may legitimately improve
    # it later, but its report and documented conclusions must then be reviewed.
    check(summaries['stress']['robust']['fault_cases'].get('uncertain-9129') == ['STALL'], 'Known stress failure missing')
    check(summaries['stress']['robust']['falls_or_envelope_exits'] == 1, 'Stress boundary outcome changed')
    nominal = {r['case']['name']: r['metrics'] for r in reports['nominal']['results'] if r['variant'] == 'robust'}
    for case, fault in [('camera', 'BODY_LOST'), ('encoder', 'ENCODER_INVALID'),
                        ('heartbeat', 'COMMAND_STALE'), ('stuck', 'FEEDBACK_INCONSISTENT')]:
        check(nominal[case]['faults'] == [fault], f'{case}: root fault not retained')
    check(not nominal['outlier']['faults'] and nominal['outlier']['mean_task_error_m'] < .015, 'Outlier rejection failed')
    check(nominal['saturation']['minimum_demand_scale'] == 0 and 'OUTPUT_LIMITED' in nominal['saturation']['statuses'], 'Insufficient-output response missing')
    for case in ['camera', 'slip', 'magnet']:
        check(nominal[case]['ended_early'], f'{case}: physical failure omitted')
    return {
        'simulation_only': True, 'hardware_deployed': False,
        'qualification': 'PASS within the declared simulated envelope; wider stress suite fails motion qualification',
        'acceptance_checks_passed': check_count,
        'metric': 'Mean Euclidean error from the original, unmodified task reference over 1 <= t < 8 seconds; paired trials have identical fixtures and sensor noise seeds.',
        'operating_envelope': acceptance['operating_envelope'],
        'suites': summaries,
        'source_sha256': {str(p.relative_to(ROOT)): digest(p) for p in sources},
        'report_sha256': {s: digest(ROOT/f'reports/robust-{s}.json') for s in reports},
        'limitations': [
            'The holdout envelope was narrowed during development and fixed before seeds 8100-8159 were run. It is not the wider stress distribution.',
            'Mass, friction, torque scale, inertia, magnetic force and body sensors are not measured hardware parameters.',
            'The wider stress set has worse average position error for the candidate and one stall followed by a simulated fall.',
            'Rapid friction/adhesion loss can cause a fall before the slip detector triggers.',
            'Zero output on a latched fault does not arrest a robot on a vertical board.',
            'Full body control requires independent timestamped pose feedback, absent on the current robot.',
        ],
    }


if __name__ == '__main__':
    result = audit()
    path = ROOT/'reports/robust-validation.json'
    path.write_text(json.dumps(result, indent=2)+'\n')
    h = result['suites']['holdout']
    print(f"PASS: {result['acceptance_checks_passed']} audit checks; 60 paired holdouts; "
          f"{h['eso']['mean_task_error_mm']:.3f} -> {h['robust']['mean_task_error_mm']:.3f} mm "
          f"({h['position_error_reduction_fraction']:.1%} reduction).")
    print('Wider stress: motion qualification FAILED; one stall/fall and position regressions retained.')
    print('Saved', path)
