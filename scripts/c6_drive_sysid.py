#!/usr/bin/env python3
"""C6 PWM-to-encoder identification. Offline commands never open a serial port.

Fits an effective first-order model in the *moving* operating range. This is
not voltage/torque identification, a breakaway measurement, or a slip sensor.
No command in this tool writes gains or saves MCU preferences.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np

TRAIN = [(140, 1.5), (180, .8), (120, .9), (200, .9), (140, .8),
         (180, 1.1), (160, .8), (200, .8), (120, 1.1), (160, .9), (140, .9)]
VALIDATE = [(140, 1.5), (160, .9), (120, 1.0), (180, .8), (140, .9),
            (200, .9), (160, 1.2), (120, .8), (180, .9), (140, 1.0)]
LIMITS = {"max_pwm": 200, "max_run_seconds": 12, "telemetry_timeout_s": .25,
          "stop_timeout_s": 3, "other_wheel_count_limit": 10}
SOURCES = [
    "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html",
    "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html",
    "https://www.mathworks.com/help/ident/gs/about-system-identification.html",
    "https://www.mathworks.com/help/ident/select-data-for-estimation.html",
]


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix(path.suffix + '.tmp')
    pending.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    pending.replace(path)


def delta32(a, b):
    return ((int(a) - int(b) + 0x80000000) & 0xffffffff) - 0x80000000


def plan():
    return {"stage": "suspended_wheels_only", "limits": LIMITS,
            "training_pwm_seconds": TRAIN, "validation_pwm_seconds": VALIDATE,
            "directions": [1, -1], "motor_order": [1, 2, 3],
            "changes_firmware_or_gains": False,
            "notes": ["One wheel at a time; verify stopped before each run.",
                      "Do not issue driving commands during collection; a conflicting lease/stop aborts it.",
                      "Stop on feedback fault, stale telemetry or unintended motion.",
                      "PWM is not measured motor voltage; no current/voltage sensor.",
                      "50ms telemetry and firmware speed filter limit dynamic resolution.",
                      "Loaded motion, wall slip and gravity hold require separate tests."]}


def check_frame(frame, motor, sign, counts):
    if not frame.get('ready'):
        raise RuntimeError('Output fault; test cancelled')
    if frame.get('control_age_ms', 0) > 40:
        raise RuntimeError('Stale control task')
    for i, m in enumerate(frame['motors']):
        if abs(m['pwm']) > LIMITS['max_pwm']:
            raise RuntimeError('PWM exceeded experiment limit')
        if i != motor and (m['pwm'] != 0 or abs(delta32(m['count'], counts[i])) > 10):
            raise RuntimeError(f'Uncommanded motion on motor {i + 1}')
    if sign * frame['motors'][motor]['speed'] < -100:
        raise RuntimeError('Encoder direction disagrees with commanded direction')


def stopped(frame):
    return (not frame['armed'] and all(
        m['pwm'] == 0 and m['fi_duty'] == 0 and m['bi_duty'] == 0 and
        m['fi_level'] == 0 and m['bi_level'] == 0 and abs(m['speed']) < 30
        for m in frame['motors']))


def stop_and_wait(client):
    client.request('stop')
    end = time.monotonic() + LIMITS['stop_timeout_s']
    # Require several NEW stopped frames, never accept a pre-command snapshot.
    quiet = 0
    while time.monotonic() < end:
        start = len(client.frames)
        client.pump(.055)
        if time.monotonic() - client.last_status > LIMITS['telemetry_timeout_s']:
            raise RuntimeError('No fresh stop confirmation')
        for f in client.frames[start:]:
            quiet = quiet + 1 if stopped(f) else 0
        if quiet >= 4:
            return
    raise RuntimeError('Wheels did not stop within 3 seconds')


def collect_run(client, motor, sign, role, profile, run):
    if sum(s for _, s in profile) > LIMITS['max_run_seconds']:
        raise ValueError('Experiment too long')
    if any(not 0 < p <= LIMITS['max_pwm'] or not 0 < s <= 2 for p, s in profile):
        raise ValueError('Unsafe experiment profile')
    stop_and_wait(client)
    counts = [m['count'] for m in client.latest['motors']]
    check_frame(client.latest, motor, sign, counts)
    lease = client.request('arm')['lease']
    run.update(motor=motor + 1, sign=sign, role=role, lease=lease,
               session=client.session, profile=profile, frames=[], commands=[])
    sequence = 0
    try:
        for pwm, duration in profile:
            end = time.monotonic() + duration
            while time.monotonic() < end:
                start = len(client.frames)
                sequence += 1
                values = [0, 0, 0]
                values[motor] = sign * pwm
                client.request('pwm', lease=lease, seq=sequence, values=values)
                run['commands'].append({'host': time.monotonic(), 'seq': sequence, 'values': values})
                # Old READY frames can precede the first command acknowledgement.
                watermark = client.latest['ms']
                client.pump(.045)
                if time.monotonic() - client.last_status > LIMITS['telemetry_timeout_s']:
                    raise RuntimeError('Stale telemetry')
                for f in client.frames[start:]:
                    if f['armed'] and f['lease'] == lease and f['reason'] == 'BENCH':
                        check_frame(f, motor, sign, counts)
                        run['frames'].append(f)
                    elif delta32(f['ms'], watermark) > 0:
                        raise RuntimeError('Experiment stopped: ' + f['reason'])
        run['complete'] = True
    finally:
        # A failure here propagates: no subsequent motor is tested.
        stop_and_wait(client)


def collect(args):
    if not args.suspended:
        raise ValueError('Collection requires --suspended after physically securing the robot off the wall')
    # Import here so offline analysis cannot accidentally construct a serial client.
    from c6_drive_tune import Client
    path = Path(args.report)
    if path.exists() or path.with_suffix('.jsonl').exists():
        raise ValueError('Choose a new report path; experiment evidence will not be overwritten')
    report = {"schema": "wbb-sysid-1", "started_utc": datetime.now(timezone.utc).isoformat(),
              "condition": "suspended", "hardware_id":getattr(args,'hardware_id',None),
              "control_trace_hz":100,"plan": plan(), "runs": [], "status": "COLLECTING"}
    write_json(path, report)
    client = None
    log = path.with_suffix('.jsonl').open('x')
    try:
        client = Client(args.port, log)
        report['initial'] = client.latest
        stop_and_wait(client)
        client.request('trace',enabled=True)
        for motor in args.motors:
            for sign in [1, -1]:
                for role, profile in [('train', TRAIN), ('validation', VALIDATE)]:
                    run = {}
                    report['runs'].append(run)
                    print(f'M{motor} {sign:+d} {role}: up to 200/255 PWM', flush=True)
                    collect_run(client, motor - 1, sign, role, profile, run)
                    write_json(path, report)
                    print(f'  {len(run["frames"])} samples; stopped', flush=True)
        report['status'] = 'COLLECTED_NOT_TUNED'
    except BaseException as exc:
        report['status'] = 'ABORTED'
        report['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        if client is not None:
            try:
                stop_and_wait(client)
                report['stopped'] = True
                client.request('trace',enabled=False)
            except Exception as exc:
                report['stopped'] = False
                report['stop_error'] = str(exc)
                report['status'] = 'ABORTED'
            finally:
                report['final'] = client.latest
                client.s.close()
        log.close()
        write_json(path, report)
        if report.get('stop_error') and not report.get('error'):
            raise RuntimeError('Final stop was not confirmed: ' + report['stop_error'])


def series(run):
    if not run.get('complete'):
        raise ValueError('Incomplete experiment')
    frames = run['frames']
    if len(frames) < 60:
        raise ValueError('Too few samples')
    motor, sign = run['motor'] - 1, run['sign']
    ms = np.array([f['ms'] for f in frames], dtype=np.int64)
    dt = (np.diff(ms) & 0xffffffff) / 1000.
    if np.any(dt <= 0) or np.max(dt) > .15:
        raise ValueError('Duplicate, out-of-order or missing telemetry')
    if len({f['session'] for f in frames}) != 1:
        raise ValueError('Mixed board sessions')
    if any(not f['armed'] or f['reason'] != 'BENCH' or f['lease'] != run['lease'] for f in frames):
        raise ValueError('Closed-loop, stopped or foreign-lease data')
    all_t = np.r_[0, np.cumsum(dt)]
    # Discard initial startup: this model describes already-moving wheels.
    keep = all_t >= 1.0
    chosen = [f for f, k in zip(frames, keep) if k]
    t = all_t[keep]; t -= t[0]
    u = np.array([sign * f['motors'][motor]['pwm'] for f in chosen])
    y = np.array([sign * f['motors'][motor]['speed'] for f in chosen])
    if not np.isfinite(u).all() or not np.isfinite(y).all():
        raise ValueError('Nonfinite data')
    if np.min(u) <= 0 or np.max(u) > LIMITS['max_pwm'] or np.ptp(u) < 50:
        raise ValueError('Insufficient or invalid input range')
    if np.count_nonzero(np.diff(u)) < 5 or np.min(y) < 100:
        raise ValueError('Not an excited, continuously moving experiment')
    for f in chosen:
        m = f['motors'][motor]
        if abs((m['fi_duty'] - m['bi_duty']) - m['pwm']) > 1:
            raise ValueError('PWM register and command mismatch')
        for i, other in enumerate(f['motors']):
            if i != motor and (other['pwm'] != 0 or abs(delta32(other['count'], chosen[0]['motors'][i]['count'])) > 10):
                raise ValueError('Other wheel moved during experiment')
    first, last = chosen[0]['motors'][motor], chosen[-1]['motors'][motor]
    edges = (last['edges'] - first['edges']) & 0xffffffff
    efficiency = abs(delta32(last['count'], first['count'])) / max(1, edges)
    if not .8 <= efficiency <= 1.05:
        raise ValueError('Poor encoder quadrature quality')
    return t, u, y, {'median_sample_s': float(np.median(dt)), 'max_sample_s': float(np.max(dt)),
                     'quadrature_efficiency': efficiency}


def response_basis(t, u, tau, delay):
    """Exact first-order integration of delayed, zero-order-held sampled input.

    Before t=0 the input is assumed equal to u[0] (initial settled plateau).
    Real command edges are only known to within one telemetry interval.
    """
    changes = np.flatnonzero(np.diff(u)) + 1
    events = t[changes] + delay
    xu = np.zeros(len(t))
    held = float(u[0]); state = 0.; event = 0
    for k in range(1, len(t)):
        cursor = t[k - 1]
        while event < len(events) and events[event] <= t[k]:
            edge = events[event]
            a = np.exp(-(edge - cursor) / tau)
            state = a * state + (1 - a) * held
            held = float(u[changes[event]])
            cursor = edge; event += 1
        a = np.exp(-(t[k] - cursor) / tau)
        state = a * state + (1 - a) * held
        xu[k] = state
    initial = np.exp(-t / tau)
    return np.column_stack((xu, 1 - initial)), initial


def series_high_rate(run):
    """Validate status-level isolation, then use deduplicated control-clock rows."""
    _, _, _, quality = series(run)
    unique = {}
    for frame in run['frames']:
        for row in frame.get('control_samples', []):
            if len(row)!=11:
                raise ValueError('Invalid control trace row')
            if row[0] in unique and unique[row[0]]!=row:
                raise ValueError('Inconsistent duplicate control sample')
            unique[row[0]]=row
    if len(unique)<500:
        raise ValueError('Insufficient high-rate trace')
    raw=np.asarray(list(unique.values()),dtype=float)
    if not np.isfinite(raw).all():raise ValueError('Nonfinite control trace')
    motor,sign=run['motor']-1,run['sign']
    active=np.flatnonzero(sign*raw[:,8+motor]>0)
    if not len(active):raise ValueError('No driven control samples')
    raw=raw[active[0]:]
    dt=(np.diff(raw[:,0].astype(np.int64))&0xffffffff)/1000
    if np.any(dt<=0) or np.max(dt)>.03:raise ValueError('High-rate trace gap exceeds 30ms')
    t=np.r_[0,np.cumsum(dt)];keep=t>=1
    u=sign*raw[keep,8+motor];y=sign*raw[keep,2+motor];t=t[keep];t-=t[0]
    if np.min(u)<=0 or np.max(u)>LIMITS['max_pwm'] or np.ptp(u)<50:
        raise ValueError('Invalid high-rate excitation')
    if np.count_nonzero(np.diff(u))<5 or np.min(y)<100:
        raise ValueError('High-rate data not continuously moving and excited')
    for other in range(3):
        if other!=motor and np.any(raw[:,8+other]!=0):
            raise ValueError('Other actuator active in control trace')
    quality.update(status_median_sample_s=quality['median_sample_s'],
                   median_sample_s=float(np.median(dt)),max_sample_s=float(np.max(dt)),
                   series_source='100Hz control trace; filtered speed and applied duty',
                   fitted_samples=len(t))
    return t,u,y,quality


def simulate(t, u, initial_speed, model):
    basis, initial = response_basis(t, u, model['tau_s'], model['delay_s'])
    return initial * initial_speed + basis @ [model['gain_cps_per_pwm'], model['bias_cps']]


def metrics(y, predicted):
    residual = y - predicted
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    variance = float(np.sum((y - np.mean(y)) ** 2))
    return {'rmse_cps': rmse, 'r_squared': 1 - float(residual @ residual) / max(variance, 1e-12),
            'rmse_fraction_of_speed_span': rmse / max(float(np.ptp(y)), 1),
            'residual_mean_cps': float(np.mean(residual))}


def fit_model(t, u, y):
    best = None
    sample = float(np.median(np.diff(t)))
    for tau in np.geomspace(.015, 1.2, 55):
        for delay in np.arange(0., .201, sample / 2):
            basis, initial = response_basis(t, u, tau, delay)
            coeff, _, rank, _ = np.linalg.lstsq(basis, y - initial * y[0], rcond=None)
            if rank < 2 or coeff[0] <= 0:
                continue
            prediction = initial * y[0] + basis @ coeff
            loss = float(np.mean((y - prediction) ** 2))
            if best is None or loss < best[0]:
                best = (loss, {'gain_cps_per_pwm': float(coeff[0]), 'bias_cps': float(coeff[1]),
                               'tau_s': float(tau), 'delay_s': float(delay)})
    if best is None:
        raise ValueError('No positive-gain first-order model')
    model = best[1]
    model['train'] = metrics(y, simulate(t, u, y[0], model))
    return model


def analyze(data, high_rate=False):
    if data.get('schema') != 'wbb-sysid-1' or data.get('status') != 'COLLECTED_NOT_TUNED' or not data.get('stopped'):
        raise ValueError('Need a complete, stopped identification experiment')
    result = {'schema': 'wbb-sysid-model-1', 'models': [], 'sources': SOURCES,
              'condition': data['condition'], 'gains_written': False,
              'limitations': ['Effective PWM-to-filtered-encoder model, not voltage or torque.',
                              'Moving range only; friction intercept is NOT static breakaway PWM.',
                              'Encoder alone cannot identify chassis slip or gravity compensation.',
                              'Filtered wheel speed includes sensor filtering dynamics; PWM is not motor voltage.',
                              'No automatic PID gains; independent loaded validation still required.']}
    traces = []
    for motor in sorted({r['motor'] for r in data['runs']}):
        for sign in [1, -1]:
            rows = [r for r in data['runs'] if r['motor'] == motor and r['sign'] == sign]
            train = [r for r in rows if r['role'] == 'train']
            validation = [r for r in rows if r['role'] == 'validation']
            if len(train) != 1 or len(validation) != 1 or train[0]['lease'] == validation[0]['lease']:
                raise ValueError('Need distinct training and validation experiments')
            model = {'motor': motor, 'sign': sign}
            try:
                extract=series_high_rate if high_rate else series
                t, u, y, quality = extract(train[0])
                vt, vu, vy, vquality = extract(validation[0])
                model.update(fit_model(t, u, y))
                prediction = simulate(vt, vu, vy[0], model)
                model.update(validation=metrics(vy, prediction), quality=quality,
                             validation_quality=vquality, pwm_range=[float(min(u)), float(max(u))])
                model['moving_feedforward'] = {'kv_pwm_per_cps': 1 / model['gain_cps_per_pwm'],
                                              'ks_pwm': -model['bias_cps'] / model['gain_cps_per_pwm']}
                model['dynamics_resolved'] = (2 * max(quality['median_sample_s'], vquality['median_sample_s'])
                                             <= model['tau_s'] < 1.19 and model['delay_s'] < .19)
                model['validation_passed'] = (model['validation']['r_squared'] >= .9 and
                                             model['validation']['rmse_fraction_of_speed_span'] <= .1)
                model['status'] = ('VALIDATED_MOVING_MODEL' if model['validation_passed'] and model['dynamics_resolved']
                                   else 'INSUFFICIENT_MODEL_OR_SAMPLE_RATE')
                traces.append((motor, sign, vt, vy, prediction))
            except ValueError as exc:
                model.update(status='REJECTED_DATA', error=str(exc))
            result['models'].append(model)
    result['all_models_qualified'] = len(result['models']) == 6 and all(
        m['status'] == 'VALIDATED_MOVING_MODEL' for m in result['models'])
    return result, traces


def plot(traces, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), constrained_layout=True)
    for ax in axes.flat:
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Speed magnitude (encoder counts/s)')
        ax.grid(alpha=.2)
    for motor, sign, t, actual, predicted in traces:
        ax = axes[motor - 1][0 if sign == 1 else 1]
        ax.plot(t, actual, label='Measured', lw=1.3)
        ax.plot(t, predicted, label='Independent prediction', lw=1.3)
        ax.set_title(f'Motor {motor}, direction {sign:+d}'); ax.legend(fontsize=8)
    fig.suptitle('Held-out suspended-wheel validation — not wall/slip validation')
    fig.savefig(path, dpi=160)
    plt.close(fig)


def audit(paths):
    """Summarize existing open-loop data without treating closed-loop runs as excitation."""
    report = {'schema': 'wbb-sysid-audit-1', 'logs': [], 'gains_written': False}
    for path in paths:
        groups = {}
        for line in Path(path).read_text().splitlines():
            row = json.loads(line); f = row.get('frame', {})
            if f.get('type') != 'status' or not f.get('armed') or f.get('reason') != 'BENCH':
                continue
            moving = [i for i, m in enumerate(f['motors']) if m['pwm'] != 0]
            if len(moving) != 1:
                continue
            i = moving[0]; pwm = f['motors'][i]['pwm']
            groups.setdefault((f['session'], f['lease'], i, pwm), []).append(f)
        rows = []
        for (_, _, i, pwm), frames in groups.items():
            tail = frames[len(frames) // 2:]
            if len(tail) < 4:
                continue
            speed = np.array([f['motors'][i]['speed'] for f in tail])
            first, last = tail[0]['motors'][i], tail[-1]['motors'][i]
            edges = (last['edges'] - first['edges']) & 0xffffffff
            rows.append({'motor': i + 1, 'pwm': pwm, 'mean_cps': float(np.mean(speed)),
                         'std_cps': float(np.std(speed)), 'samples': len(frames),
                         'duration_s': ((frames[-1]['ms'] - frames[0]['ms']) & 0xffffffff) / 1000.,
                         'quadrature_efficiency': abs(delta32(last['count'], first['count'])) / max(1, edges)})
        report['logs'].append({'path': str(path), 'runs': rows})
    report['conclusion'] = 'Historical short single-level runs only; not an independently validated dynamic plant model.'
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='mode', required=True)
    sub.add_parser('plan')
    a = sub.add_parser('audit'); a.add_argument('logs', nargs='+'); a.add_argument('--report', required=True)
    c = sub.add_parser('collect'); c.add_argument('--port', default='/dev/cu.usbmodem1101')
    c.add_argument('--report', required=True); c.add_argument('--suspended', action='store_true')
    c.add_argument('--motors', type=int, choices=[1, 2, 3], nargs='+', default=[1, 2, 3])
    c.add_argument('--hardware-id',help='Confirmed device identity for keeping boards separate')
    a = sub.add_parser('analyze'); a.add_argument('input'); a.add_argument('--report', required=True)
    a.add_argument('--plot', action='store_true')
    a.add_argument('--high-rate', action='store_true',help='Fit the optional 100Hz control trace after status-level validation')
    args = ap.parse_args()
    if args.mode == 'plan':
        print(json.dumps(plan(), indent=2))
    elif args.mode == 'collect':
        if len(set(args.motors)) != len(args.motors):
            ap.error('Duplicate motor selection')
        collect(args)
    elif args.mode == 'audit':
        write_json(args.report, audit(args.logs))
    else:
        result, traces = analyze(json.loads(Path(args.input).read_text()),args.high_rate)
        write_json(args.report, result)
        if args.plot:
            plot(traces, Path(args.report).with_suffix('.png'))
        print(json.dumps({k: v for k, v in result.items() if k != 'sources'}, indent=2))


if __name__ == '__main__':
    main()
