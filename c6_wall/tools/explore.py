#!/usr/bin/env python3
"""Slip characterisation with camera ground truth: repeated board-frame moves in four directions."""
import json, math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import Robot
import vision
from roundtrip import wait_still, odom_board

MARGIN = 10.0
r = Robot()
if r.status()['mode'] != 'active':
    r.cmd('enable'); time.sleep(1)
r.cmd('beacon', on=True)
cam = vision.Camera()
front = cam.cfg['front_from_beacon_deg']
W, Hh = cam.cfg['surface_cm']
dist = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
reps = int(sys.argv[3]) if len(sys.argv) > 3 else 2
rows, H = [], None
try:
    for k in range(reps):
        for name, ux, uy in (('右', 1, 0), ('左', -1, 0), ('上', 0, 1), ('下', 0, -1)):
            H, a = vision.measure(cam, n=11, H=H)
            err = (90 - (a['heading'] + front) + 180) % 360 - 180
            if abs(err) > 5:
                r.cmd('turn', deg=err, rate=0.4); wait_still(r)
                H, a = vision.measure(cam, n=11, H=H)
            tx, ty = a['x'] + ux * dist, a['y'] + uy * dist
            def edge_gap(x, y):
                return min(x, W - x, y, Hh - y)
            if edge_gap(tx, ty) < MARGIN and edge_gap(tx, ty) < edge_gap(a['x'], a['y']):
                print(f'  {name}: 会靠近边框，跳过 (目标 {tx:.1f},{ty:.1f})'); continue
            oa = odom_board(r.status())
            r.cmd('move', x=ux * dist / 100, y=uy * dist / 100, speed=speed, accel=0.15)
            wait_still(r)
            H, b = vision.measure(cam, n=11, H=H)
            ob = odom_board(r.status())
            mx, my = b['x'] - a['x'], b['y'] - a['y']
            turn = (b['heading'] - a['heading'] + 180) % 360 - 180
            along = mx * ux + my * uy
            row = dict(dir=name, cmd=dist, along=along, across=(my * ux - mx * uy), turn=turn,
                       odo=[(ob[0] - oa[0]) / 10, (ob[1] - oa[1]) / 10, ob[2] - oa[2]], at=[b['x'], b['y']])
            rows.append(row)
            print(f"  {name}{dist:g}: 实际沿方向 {along:+5.2f} cm ({along / dist:4.0%})  侧偏 {row['across']:+5.2f} cm  转 {turn:+6.2f}°   位置 ({b['x']:.1f},{b['y']:.1f})")
            if abs(turn) > 15:
                print('  转角过大，停下'); raise SystemExit
finally:
    r.cmd('stop'); cam.close()
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs', time.strftime('explore-%m%d-%H%M%S.json'))
    json.dump(rows, open(p, 'w'), indent=1, ensure_ascii=False)
    print('log:', p)
