#!/usr/bin/env python3
"""Short USB polarity pulses after explicit authorization for floor movement.

Passive encoder motion is expected on the floor. Only the selected motor may
receive drive. These encoder bounds do not measure actual chassis slip.
Planning is the default; collection never writes gains or validation flags.
"""
import argparse
import json
import math
from pathlib import Path
import time

SIGNS = [1, 1, -1]  # Same raw quadrature convention as c6_drive/src/main.cpp.
MPC = .11938052 / 2800
PLAN = dict(pwm=140, pulse_seconds=.22, maximum_wheel_travel_m=.04,
            encoder_excursion_m=.035, encoder_heading_rad=.22,
            total_encoder_excursion_m=.09, total_encoder_heading_rad=.35,
            passive_wheel_motion_allowed=True, firmware_or_gain_changes=False)


def pose(frame):
    od = frame['odometry']
    if not od['valid'] or od['age_ms'] > 25:
        raise RuntimeError('Stale odometry')
    p = [od['x_m'], od['y_m'], od['yaw_rad']]
    if not all(math.isfinite(x) for x in p):
        raise RuntimeError('Invalid odometry')
    return p


def bounds(frame, origin, distance, angle):
    p = pose(frame)
    if math.hypot(p[0]-origin[0], p[1]-origin[1]) > distance:
        raise RuntimeError('Encoder displacement bound reached')
    yaw = math.atan2(math.sin(p[2]-origin[2]), math.cos(p[2]-origin[2]))
    if abs(yaw) > angle:
        raise RuntimeError('Encoder heading bound reached')


def pulse(client, motor, sign, run, overall):
    from c6_drive_sysid import stop_and_wait, delta32
    stop_and_wait(client)
    before = client.latest
    if not before['ready'] or before['controller'] != 'legacy':
        raise RuntimeError('Requires ready, stopped legacy controller')
    origin = pose(before)
    bounds(before, overall, PLAN['total_encoder_excursion_m'], PLAN['total_encoder_heading_rad'])
    run.update(motor=motor+1, sign=sign, before=before, frames=[], commands=[])
    lease = client.request('arm')['lease']
    accepted = None
    seq = 0
    end = time.monotonic() + PLAN['pulse_seconds']
    try:
        while time.monotonic() < end:
            mark = len(client.frames)
            seq += 1
            values = [0, 0, 0]
            values[motor] = sign * PLAN['pwm']
            ack = client.request('pwm', lease=lease, seq=seq, values=values)
            if accepted is None:
                accepted = ack['command_ms']
            run['commands'].append(dict(host=time.monotonic(), ack=ack, values=values))
            client.pump(min(.025, max(0, end-time.monotonic())))
            if time.monotonic()-client.last_status > .20:
                raise RuntimeError('Stale telemetry')
            for f in client.frames[mark:]:
                if delta32(f['ms'], accepted) < 0:
                    continue
                run['frames'].append(f)
                if not f['armed'] or f['lease'] != lease or f['reason'] != 'BENCH':
                    raise RuntimeError('Experiment stopped: '+f['reason'])
                if not f['ready'] or f['control_age_ms'] > 40:
                    raise RuntimeError('Output/control task fault')
                bounds(f, origin, PLAN['encoder_excursion_m'], PLAN['encoder_heading_rad'])
                bounds(f, overall, PLAN['total_encoder_excursion_m'], PLAN['total_encoder_heading_rad'])
                for i, m in enumerate(f['motors']):
                    duty = m['fi_duty']-m['bi_duty']
                    if abs(m['pwm']) > PLAN['pwm'] or abs(duty) > PLAN['pwm']:
                        raise RuntimeError('Experiment duty exceeded')
                    if i != motor and any(m[k] != 0 for k in ('pwm', 'fi_duty', 'bi_duty')):
                        raise RuntimeError('Unselected motor actively driven')
                m = f['motors'][motor]
                delta = delta32(m['count'], before['motors'][motor]['count'])*SIGNS[motor]
                if abs(delta)*MPC > PLAN['maximum_wheel_travel_m']:
                    raise RuntimeError('Selected wheel travel bound reached')
                if sign*delta < -12 or sign*m['speed'] < -150:
                    raise RuntimeError('Selected encoder sign disagrees with applied drive')
    finally:
        stop_mark = len(client.frames)
        stop_and_wait(client)
        run['stop_frames'] = client.frames[stop_mark:]
        run['after'] = client.latest
    frames = run['frames']
    samples = sorted({r[0]:r for f in frames+run['stop_frames'] for r in f.get('control_samples',[])
                      if delta32(r[0], accepted)>=0 and sign*r[8+motor]>0}.values())
    if len(samples)<3:
        raise RuntimeError('Insufficient 100Hz driven samples')
    m = client.latest['motors'][motor]
    b = before['motors'][motor]
    delta = delta32(m['count'], b['count'])*SIGNS[motor]
    peak = max(sign*r[2+motor] for r in samples)
    run['result'] = dict(signed_encoder_counts=delta, aligned_peak_cps=peak,
                         a_edges=delta32(m['a_edges'], b['a_edges']),
                         b_edges=delta32(m['b_edges'], b['b_edges']),
                         bad_transitions=delta32(m['bad'], b['bad']),
                         driven_samples=len(samples),
                         observed_drive_ms=delta32(samples[-1][0],samples[0][0])+10,
                         outputs_verified_stopped=True,
                         passed=sign*delta >= 12 and peak > 100)
    bounds(client.latest, origin, PLAN['encoder_excursion_m'], PLAN['encoder_heading_rad'])
    bounds(client.latest, overall, PLAN['total_encoder_excursion_m'], PLAN['total_encoder_heading_rad'])
    if not run['result']['passed']:
        raise RuntimeError('No usable selected-wheel motion; polarity unproven')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--floor-confirmed', action='store_true')
    ap.add_argument('--port')
    ap.add_argument('--report', type=Path)
    ap.add_argument('--motors', type=int, nargs='+', choices=[1, 2, 3], default=[1, 2, 3])
    ap.add_argument('--signs', type=int, nargs='+', choices=[1, -1], default=[1, -1])
    ap.add_argument('--pwm', type=int, choices=[140, 160, 180], default=140)
    ap.add_argument('--pulse-seconds', type=float, choices=[.06, .12, .16, .22], default=.22)
    args = ap.parse_args()
    PLAN['pwm'] = args.pwm
    PLAN['pulse_seconds'] = args.pulse_seconds
    if not args.execute:
        print(json.dumps(PLAN, indent=2));return
    if not args.floor_confirmed or not args.port or not args.report:
        ap.error('Requires current floor authorization, port and new report path')
    if args.report.exists() or args.report.with_suffix('.jsonl').exists():
        ap.error('Choose a new report path')
    from c6_drive_tune import Client
    from c6_drive_sysid import stop_and_wait
    record = dict(plan=PLAN, condition='user-authorized floor', runs=[], passed=False)
    client = None
    with args.report.with_suffix('.jsonl').open('x') as log:
        try:
            client = Client(args.port, log)
            stop_and_wait(client)
            record['initial'] = client.latest
            overall = pose(client.latest)
            client.request('trace', enabled=True)
            for physical in args.motors:
                motor = physical-1
                for sign in args.signs:
                    run = {};record['runs'].append(run)
                    pulse(client, motor, sign, run, overall)
                    print(json.dumps(dict(motor=motor+1, sign=sign, **run['result'])), flush=True)
            record['passed'] = True
        except Exception as error:
            record['error'] = str(error)
        finally:
            if client:
                try:
                    stop_and_wait(client)
                    client.request('trace', enabled=False)
                    record['final'] = client.latest
                    client.close()
                except Exception as error:
                    record['stop_error'] = str(error);record['passed'] = False
            args.report.write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in record.items() if k not in ('initial','final','runs')}))
    if not record['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
