#!/usr/bin/env python3
"""One bounded floor transition, with current user movement authorization.

Uses existing legacy gains, never changes calibration, and never retries a fault.
The excursion checks use encoders, not an independent chassis-position sensor.
"""
import argparse
import json
import math
from pathlib import Path
import time
from c6_floor_polarity import bounds, pose, MPC

VECTORS = {'W': (-math.sqrt(3)/2, 0, math.sqrt(3)/2),
           'S': (math.sqrt(3)/2, 0, -math.sqrt(3)/2),
           'A': (.5, -1, .5), 'D': (-.5, 1, -.5)}


def capture(client, record, pair, speed, duration=None, batch_origin=None, expected_session=None):
    from c6_drive_sysid import stop_and_wait, delta32
    if duration is not None and (pair[0]!=pair[1] or not .3<=duration<=1.5):
        raise ValueError('Custom duration requires a straight step, 0.3–1.5 seconds')
    profile = [(pair[0], duration if duration is not None else .85)] if pair[0] == pair[1] else [(pair[0], .35), (pair[1], .50)]
    stop_and_wait(client)
    before = client.latest
    record['initial'] = before
    if expected_session is not None and before['session']!=expected_session:
        raise RuntimeError('Device session changed; no motion command sent')
    if batch_origin is not None:
        bounds(before,batch_origin,.15,.40)
    if not before['ready'] or before['controller'] != 'legacy' or before.get('direction_guard_revision') != 2:
        raise RuntimeError('Requires verified stopped legacy controller and direction guard revision 2')
    if not before['calibrated']:
        raise RuntimeError('Existing calibration missing')
    origin = pose(before)
    client.request('trace', enabled=True)
    lease = client.request('arm')['lease']
    seq = 0
    accepted = None
    try:
        for key, duration in profile:
            target = [v*speed/MPC for v in VECTORS[key]]
            stage = dict(key=key, target_cps=target, duration_s=duration)
            record['stages'].append(stage)
            end = time.monotonic()+duration
            while time.monotonic() < end:
                mark = len(client.frames);seq += 1
                ack = client.request('wheels', lease=lease, seq=seq, values=target)
                stage.setdefault('accepted_ms', ack['command_ms'])
                if accepted is None:
                    accepted = ack['command_ms']
                client.pump(min(.025, max(0, end-time.monotonic())))
                if time.monotonic()-client.last_status > .20:
                    raise RuntimeError('Stale telemetry')
                for f in client.frames[mark:]:
                    if delta32(f['ms'], accepted) < 0:
                        continue
                    record['frames'].append(f)
                    if not f['armed'] or f['lease'] != lease or f['reason'] != 'DRIVE':
                        raise RuntimeError('Experiment stopped: '+f['reason'])
                    if not f['ready'] or f['control_age_ms'] > 40 or f['controller'] != 'legacy':
                        raise RuntimeError('Control/output fault')
                    bounds(f, origin, .075, .30)
                    if batch_origin is not None:
                        bounds(f,batch_origin,.15,.40)
                    for i, m in enumerate(f['motors']):
                        if abs(m['pwm']) > 180 or abs(m['fi_duty']-m['bi_duty']) > 180:
                            raise RuntimeError('Floor experiment duty bound exceeded')
                        if abs(delta32(m['count'], before['motors'][i]['count']))*MPC > .11:
                            raise RuntimeError('Wheel travel bound exceeded')
        record['commands_completed'] = True
    finally:
        stop_mark = len(client.frames)
        record['stop_requested_host'] = time.monotonic()
        stop_and_wait(client)
        record['stop_frames'] = client.frames[stop_mark:]
        client.request('trace', enabled=False)
        record['final'] = client.latest
    bounds(client.latest, origin, .075, .30)
    rows = sorted({row[0]: row for f in record['frames'] for row in f.get('control_samples', [])
                   if delta32(row[0], accepted) >= 0}.values())
    minimum_samples = max(20, int(sum(seconds for _,seconds in profile)*80))
    if len(rows) < minimum_samples or any(b[0]-a[0] > 40 for a,b in zip(rows,rows[1:])):
        raise RuntimeError('Incomplete 100Hz control trace')
    # This is a counterfactual predicate on the measured trace, not an old-build replay.
    legacy_hits = [dict(ms=r[0], motor=i+1, speed=r[2+i], reference=r[5+i], applied_pwm=r[8+i])
                   for r in rows for i in range(3)
                   if abs(r[5+i]) > 500 and abs(r[2+i]) > 400 and r[5+i]*r[2+i] < 0]
    record['analysis'] = dict(sample_count=len(rows), old_instantaneous_predicate=legacy_hits,
                              final_pose=pose(client.latest), origin=origin,
                              body_position_measured=False, outputs_verified_stopped=True)
    record['passed'] = True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--floor-confirmed', action='store_true')
    ap.add_argument('--port')
    ap.add_argument('--report', type=Path)
    ap.add_argument('--pair', choices=['WA', 'SD', 'AD', 'DA', 'WS', 'SW', 'AS', 'DW', 'WW', 'SS', 'AA', 'DD'], default='WA')
    ap.add_argument('--speed', type=float, choices=[.04, .06, .08, .10, .12], default=.08)
    ap.add_argument('--duration', type=float, help='Straight steps only, 0.3–1.5 seconds')
    ap.add_argument('--expected-session', type=int)
    ap.add_argument('--batch-origin', type=float, nargs=3)
    args = ap.parse_args()
    if args.duration is not None and (args.pair[0]!=args.pair[1] or not .3<=args.duration<=1.5):
        ap.error('Custom duration requires a straight step, 0.3–1.5 seconds')
    plan = dict(pair=args.pair, requested_speed_mps=args.speed, duration_s=args.duration if args.duration is not None else .85,
                encoder_excursion_m=.075, encoder_heading_rad=.30, max_duty=180)
    if not args.execute:
        print(json.dumps(plan));return
    if not args.floor_confirmed or not args.port or not args.report:
        ap.error('Requires current floor authorization, port and report')
    if args.report.exists() or args.report.with_suffix('.jsonl').exists():
        ap.error('Use a new evidence path')
    from c6_drive_tune import Client
    from c6_drive_sysid import stop_and_wait
    client = None
    record = dict(plan=plan, stages=[], frames=[], passed=False, commands_completed=False)
    with args.report.with_suffix('.jsonl').open('x') as log:
        try:
            client = Client(args.port, log)
            capture(client, record, args.pair, args.speed, args.duration,args.batch_origin,args.expected_session)
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
    print(json.dumps({k:v for k,v in record.items() if k not in ('initial','final','frames','stop_frames')},indent=2))
    if not record['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
