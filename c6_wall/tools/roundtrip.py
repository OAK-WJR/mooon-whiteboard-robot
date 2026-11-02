#!/usr/bin/env python3
"""Round-trip accuracy test without the board tapes: the LCD beacon is the ruler.

    python3 tools/roundtrip.py --dist 20 --reps 3          # right D, back left D; repeat
    python3 tools/roundtrip.py --dist 20 --axis y          # up D, back down D
    python3 tools/roundtrip.py --square                    # only turn the front straight up (image vertical)

The beacon (cyan left half, magenta right half of the 172x320 LCD, 0.1011 mm pixels) has its
two colour centroids 16.18 mm apart, in the plane the robot moves in, so pixels convert to mm
with no camera distance, zoom or parallax term. The robot centre sits 11.64 mm from the
beacon midpoint along cyan->magenta (CAD). Only start/end differences are reported, so lens
distortion and camera roll cancel. The robot is not corrected by the camera in any way.
Logs: logs/roundtrip-*.json.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import Robot  # noqa: E402
import vision  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BEACON_SEP_MM = 160 * 0.1011


def rel_measure(cam, n=15, tries=3):
    """Median robot pose in camera-plane mm (X right, Y up) and heading (deg, beacon axis)."""
    cfg = cam.cfg
    off = [10 * v for v in cfg.get("beacon_to_centre_cm", (1.164, 0.039))]
    for attempt in range(tries):
        xs, ys, hs, ss = [], [], [], []
        for _ in range(n):
            b = vision.find_beacon(cam.frame())
            if b is None:
                continue
            c = np.array([b[0][0], -b[0][1]])
            m = np.array([b[1][0], -b[1][1]])
            d = m - c
            sep = float(np.hypot(*d))
            s = BEACON_SEP_MM / sep                     # mm per pixel in the beacon plane
            a = math.atan2(d[1], d[0])
            mid = (c + m) / 2
            cx = mid[0] * s + math.cos(a) * off[0] - math.sin(a) * off[1]
            cy = mid[1] * s + math.sin(a) * off[0] + math.cos(a) * off[1]
            xs.append(cx); ys.append(cy); hs.append(math.degrees(a)); ss.append(s)
        if len(xs) >= max(3, n // 2):
            ref = hs[0]
            hs = [ref + ((h - ref + 180) % 360 - 180) for h in hs]
            return {"x": float(np.median(xs)), "y": float(np.median(ys)), "heading": float(np.median(hs)),
                    "mm_per_px": float(np.median(ss)), "spread_mm": float(max(np.ptp(xs), np.ptp(ys))),
                    "spread_deg": float(np.ptp(hs)), "frames": len(xs)}
        print(f"  Beacon seen in only {len(xs)}/{n} frames; retrying in 2 s")
        time.sleep(2)
    raise SystemExit("The camera cannot see the robot beacon")


def wait_still(r, timeout=40):
    end = time.time() + timeout
    time.sleep(0.3)
    while time.time() < end:
        s = r.status()
        if s["mode"] != "active":
            raise SystemExit(f"Robot is not holding: {s['mode']} {s['fault']}")
        if s["src"] == "none" and not s["moving"]:
            time.sleep(0.8)
            return r.status()
        time.sleep(0.15)
    raise SystemExit("Move timed out")


def odom_board(s):
    a = math.radians(s["mount"] - s.get("front", 0))
    x, y = s["pose"][0], s["pose"][1]
    return (1000 * (math.cos(a) * x - math.sin(a) * y), 1000 * (math.sin(a) * x + math.cos(a) * y),
            math.degrees(s["pose"][2]))


def dturn(a, b):
    return (b - a + 180) % 360 - 180


def square(r, cam, front, tol=0.3):
    """Turn until the robot front points straight up in the image (camera roll included)."""
    for _ in range(4):
        v = rel_measure(cam, n=9)
        turn = dturn(v["heading"] + front, 90)
        print(f"  Heading {v['heading'] + front:.2f}° (image vertical = 90°), turning {turn:+.2f}°")
        if abs(turn) < tol:
            return v
        r.cmd("turn", deg=turn, rate=0.3)
        wait_still(r)
    return rel_measure(cam, n=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=20, help="one-way distance, cm")
    ap.add_argument("--axis", default="x", choices=("x", "y"))
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--accel", type=float, default=0.1)
    ap.add_argument("--square", action="store_true", help="only square the robot up")
    ap.add_argument("--no-square", action="store_true")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    r = Robot()
    s = r.status()
    if s["mode"] != "active":
        r.cmd("enable")
        time.sleep(1.0)
    r.cmd("beacon", on=True)
    time.sleep(0.6)
    cam = vision.Camera()
    front = cam.cfg.get("front_from_beacon_deg", 66.0)
    log = {"start": time.strftime("%F %T"), "args": vars(args), "status0": {k: s[k] for k in ("fw", "slip", "gains", "radius_mm", "mount", "front")},
           "trips": []}
    try:
        if args.square or not args.no_square:
            print("Squaring up the heading")
            square(r, cam, front)
            r.cmd("origin")
            if args.square:
                return
        D = args.dist / 100
        out = (D, 0) if args.axis == "x" else (0, D)
        for k in range(args.reps):
            a = rel_measure(cam)
            oa = odom_board(r.status())
            ack = r.cmd("move", x=out[0], y=out[1], speed=args.speed, accel=args.accel)
            if not ack.get("ok"):
                raise SystemExit(f"move rejected: {ack.get('error')}")
            wait_still(r)
            b = rel_measure(cam)
            ob = odom_board(r.status())
            ack = r.cmd("move", x=-out[0], y=-out[1], speed=args.speed, accel=args.accel)
            if not ack.get("ok"):
                raise SystemExit(f"move rejected: {ack.get('error')}")
            wait_still(r)
            c = rel_measure(cam)
            oc = odom_board(r.status())
            trip = {"a": a, "b": b, "c": c, "odo": [oa, ob, oc],
                    "out_mm": [b["x"] - a["x"], b["y"] - a["y"]], "out_turn": dturn(a["heading"], b["heading"]),
                    "err_mm": [c["x"] - a["x"], c["y"] - a["y"]], "err_turn": dturn(a["heading"], c["heading"])}
            trip["err_dist"] = math.hypot(*trip["err_mm"])
            log["trips"].append(trip)
            ok = trip["err_dist"] <= 2 and abs(trip["err_turn"]) <= 1
            print(f"Trip {k + 1}: outbound camera ({trip['out_mm'][0]:+7.1f},{trip['out_mm'][1]:+6.1f}) mm turn {trip['out_turn']:+5.2f}°"
                  f" | return-to-start error {trip['err_dist']:5.2f} mm ({trip['err_mm'][0]:+5.2f},{trip['err_mm'][1]:+5.2f})"
                  f"  heading {trip['err_turn']:+5.2f}°  {'pass' if ok else 'not met'}"
                  f"  [measurement jitter {c['spread_mm']:.2f} mm / {c['spread_deg']:.2f}°]")
    finally:
        r.cmd("stop")
        cam.close()
        os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
        path = os.path.join(ROOT, "logs", time.strftime("roundtrip-%m%d-%H%M%S") + (f"-{args.tag}" if args.tag else "") + ".json")
        json.dump(log, open(path, "w"), indent=1, ensure_ascii=False)
        print("log:", path)


if __name__ == "__main__":
    main()
