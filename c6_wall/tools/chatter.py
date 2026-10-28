#!/usr/bin/env python3
"""Count wheel output reversals (chatter) on a sideways move and back.  chatter.py [dist_m]"""
import sys, time
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from bench import Robot
import vision


def flips(vv, floor=0.03):
    s = [1 if x > floor else -1 if x < -floor else 0 for x in vv]
    s = [x for x in s if x]
    return sum(1 for a, b in zip(s, s[1:]) if a != b)


def run(r, dist=0.15, speed=0.03):
    d0 = -dist  # robot sits right of the centre (x ~ 42 cm): left first, then back
    res = []
    for d in (d0, -d0):
        r.cmd('move', x=d, y=0, speed=speed, accel=0.08)
        rows, t0 = [], time.time()
        time.sleep(0.3)
        while time.time() - t0 < 15:
            try:
                s = r.status(); rows.append(s)
                if s['src'] == 'none' and not s['moving'] and time.time() - t0 > 1: break
            except Exception:
                pass
            time.sleep(0.02)
        T = rows[-1]['_t'] - rows[0]['_t']
        f = [flips([x['w'][i][1] for x in rows]) / T for i in range(3)]
        res.append(f)
        print('  %s: flips/s  w1 %.1f  w2 %.1f  w3 %.1f' % ('right' if d > 0 else 'left ', *f), flush=True)
    return res


if __name__ == '__main__':
    run(Robot(), float(sys.argv[1]) if len(sys.argv) > 1 else 0.15)
