#!/usr/bin/env python3
"""Spin-erase test: the wheels sweep the board while the robot moves and turns (no cloth).

    python3 tools/erase_test.py ink                         # print ink area per 5 cm band (find a line)
    python3 tools/erase_test.py run --dx 20 --turns 3       # spin-move 20 cm right turning 3 turns, measure ink before/after
    python3 tools/erase_test.py run --dx 20 --turns 3 --back   # ... and come back turning -3 turns (round-trip error too)

The board image is rectified with the tape homography to 1 px = 1 mm. Ink = saturated
orange/red pixels inside the swept band (robot centre path +- 60 mm), excluding the tapes.
Before/after crops go to logs/erase-*.png.
"""
import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import Robot  # noqa: E402
import vision  # noqa: E402
from roundtrip import wait_still  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rectified(cam):
    """Top-down board image, 1 px = 1 mm, origin bottom-left (image row 0 = top of the board)."""
    img = cam.frame()
    H = vision.surface_homography(img, cam.cfg)
    if H is None:
        raise RuntimeError("看不到胶带")
    w, h = cam.cfg["surface_cm"]
    W, Hh = int(w * 10), int(h * 10)
    # board cm (y up) -> image mm (y down)
    S = np.array([[10, 0, 0], [0, -10, Hh], [0, 0, 1]], dtype=np.float64)
    return cv2.warpPerspective(img, S @ H, (W, Hh)), Hh


def ink_mask(top):
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 70, 60), (22, 255, 255)) | cv2.inRange(hsv, (170, 70, 60), (180, 255, 255))
    m[:60, :60] = m[:60, -60:] = m[-60:, :60] = m[-60:, -60:] = 0  # tapes
    return m


def band_ink(cam, x0, y0, x1, y1, half=60, frames=5, avoid=()):
    """Ink pixels (mm^2) within +-half mm of the segment (x0,y0)-(x1,y1) in board cm, outside the
    robot footprints given in `avoid` (board cm centres; the robot carries red wires)."""
    counts, top, mask = [], None, None
    for _ in range(frames):
        top, Hh = rectified(cam)
        m = ink_mask(top)
        band = np.zeros_like(m)
        p0 = (int(x0 * 10), int(Hh - y0 * 10))
        p1 = (int(x1 * 10), int(Hh - y1 * 10))
        cv2.line(band, p0, p1, 255, thickness=2 * half)
        for ax, ay in avoid:
            cv2.circle(band, (int(ax * 10), int(Hh - ay * 10)), 75, 0, -1)
        mask = m & band
        counts.append(int(np.count_nonzero(mask)))
    return int(np.median(counts)), top, mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("ink", "run"))
    ap.add_argument("--dx", type=float, default=20.0, help="cm, board frame")
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--turns", type=float, default=3.0)
    ap.add_argument("--speed", type=float, default=0.02, help="m/s along the path")
    ap.add_argument("--rate", type=float, default=2.5, help="rad/s turn rate limit")
    ap.add_argument("--back", action="store_true")
    args = ap.parse_args()
    cam = vision.Camera()
    try:
        if args.cmd == "ink":
            top, Hh = rectified(cam)
            m = ink_mask(top)
            for y in range(0, Hh, 50):
                print(f"Y {(Hh - y - 50) / 10:5.1f}-{(Hh - y) / 10:5.1f} cm: ink {np.count_nonzero(m[y:y + 50]) / 100:7.1f} cm^2")
            return
        r = Robot()
        if r.status()["mode"] != "active":
            r.cmd("enable")
            time.sleep(1)
        r.cmd("beacon", on=True)
        time.sleep(0.5)
        _, a = vision.measure(cam, n=11)
        x0, y0 = a["x"], a["y"]
        x1, y1 = x0 + args.dx, y0 + args.dy
        avoid = [(x0, y0)] if args.back else [(x0, y0), (x1, y1)]
        before, top0, m0 = band_ink(cam, x0, y0, x1, y1, avoid=avoid)
        ack = r.cmd("spin", x=args.dx / 100, y=args.dy / 100, deg=360 * args.turns, speed=args.speed, accel=0.05, rate=args.rate)
        if not ack.get("ok"):
            raise SystemExit(f"spin 被拒绝: {ack.get('error')}")
        wait_still(r, timeout=300)
        _, b = vision.measure(cam, n=11)
        if args.back:
            ack = r.cmd("spin", x=-args.dx / 100, y=-args.dy / 100, deg=-360 * args.turns, speed=args.speed, accel=0.05, rate=args.rate)
            wait_still(r, timeout=300)
        _, c = vision.measure(cam, n=11)
        after, top1, m1 = band_ink(cam, x0, y0, x1, y1, avoid=avoid)
        print(f"去程实际: ({b['x'] - x0:+.2f}, {b['y'] - y0:+.2f}) cm  转角 {(b['heading'] - a['heading'] + 180) % 360 - 180:+.2f}° (命令 {360 * args.turns:+.0f}°)")
        print(f"扫过区域墨迹: 前 {before / 100:.1f} cm^2  后 {after / 100:.1f} cm^2  擦掉 {100 * (1 - after / max(1, before)):.0f}%")
        if args.back:
            err = math.hypot(c["x"] - x0, c["y"] - y0) * 10
            print(f"往返误差: {err:.2f} mm ({(c['x'] - x0) * 10:+.2f}, {(c['y'] - y0) * 10:+.2f})  航向 {(c['heading'] - a['heading'] + 180) % 360 - 180:+.2f}°")
        ys = int(top0.shape[0] - max(y0, y1) * 10 - 90)
        ye = int(top0.shape[0] - min(y0, y1) * 10 + 90)
        xs, xe = int(min(x0, x1) * 10 - 90), int(max(x0, x1) * 10 + 90)
        crop = lambda im: im[max(0, ys):ye, max(0, xs):xe]  # noqa: E731
        out = np.vstack([crop(top0), crop(top1)])
        path = os.path.join(ROOT, "logs", time.strftime("erase-%m%d-%H%M%S.png"))
        cv2.imwrite(path, out)
        print("前/后对比图:", path)
    finally:
        cam.close()


if __name__ == "__main__":
    main()
