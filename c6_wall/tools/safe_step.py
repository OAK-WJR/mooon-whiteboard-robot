#!/usr/bin/env python3
"""Guarded small moves with camera checks (tapes visible).

    python3 tools/safe_step.py up 3            # one 3 cm step up
    python3 tools/safe_step.py to 27 40        # walk in <=step cm steps to board X,Y (cm), checking every step

Every step: camera pose before/after. The run stops (robot holds) as soon as a step travels
less than 60 % / more than 140 % of the command, turns more than 3 deg, or would end within
the edge margin of the writable surface.
"""
import math
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from bench import Robot  # noqa: E402
import vision  # noqa: E402
from roundtrip import wait_still  # noqa: E402

MARGIN = 11.0   # cm from the writable edge to the robot centre


def guarded_step(r, cam, H, dx, dy, speed=0.04):
    H, a = vision.measure(cam, n=11, H=H)
    w, h = cam.cfg["surface_cm"]
    tx, ty = a["x"] + dx, a["y"] + dy
    if not (MARGIN <= tx <= w - MARGIN and MARGIN <= ty <= h - MARGIN) and \
            (math.hypot(w / 2 - tx, h / 2 - ty) > math.hypot(w / 2 - a["x"], h / 2 - a["y"])):
        return H, a, None, "Target too close to the frame; not moving"
    ack = r.cmd("move", x=dx / 100, y=dy / 100, speed=speed, accel=0.1)
    if not ack.get("ok"):
        return H, a, None, f"move rejected {ack.get('error')}"
    wait_still(r)
    H, b = vision.measure(cam, n=11, H=H)
    mx, my = b["x"] - a["x"], b["y"] - a["y"]
    along = (mx * dx + my * dy) / max(1e-6, math.hypot(dx, dy))
    turn = (b["heading"] - a["heading"] + 180) % 360 - 180
    ratio = along / math.hypot(dx, dy)
    msg = f"command ({dx:+.1f},{dy:+.1f}) cm  measured ({mx:+.2f},{my:+.2f}) cm  ratio {ratio:.2f}  turn {turn:+.2f}°"
    bad = not (0.6 <= ratio <= 1.4) or abs(turn) > 3
    return H, b, msg, ("Out of control; stopping" if bad else None)


def main():
    r = Robot()
    s = r.status()
    if s["mode"] != "active":
        r.cmd("enable")
        time.sleep(1.0)
    r.cmd("beacon", on=True)
    time.sleep(0.5)
    cam = vision.Camera()
    H = None
    try:
        if sys.argv[1] == "to":
            gx, gy = float(sys.argv[2]), float(sys.argv[3])
            step = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
            for _ in range(40):
                H, a = vision.measure(cam, n=11, H=H)
                ex, ey = gx - a["x"], gy - a["y"]
                d = math.hypot(ex, ey)
                if d < 0.5:
                    print(f"Arrived: X {a['x']:.2f} Y {a['y']:.2f}")
                    break
                k = min(1.0, step / d)
                H, b, msg, stop = guarded_step(r, cam, H, ex * k, ey * k)
                print(" ", msg or "", stop or "")
                if stop:
                    break
        else:
            dirs = {"up": (0, 1), "down": (0, -1), "left": (-1, 0), "right": (1, 0)}
            ux, uy = dirs[sys.argv[1]]
            dist = float(sys.argv[2])
            H, b, msg, stop = guarded_step(r, cam, H, ux * dist, uy * dist)
            print(msg or "", stop or "")
            print(f"Now: X {b['x']:.2f} Y {b['y']:.2f} cm  heading {b['heading'] + cam.cfg['front_from_beacon_deg']:.2f}°")
    finally:
        r.cmd("stop")
        cam.close()


if __name__ == "__main__":
    main()
