#!/usr/bin/env python3
"""Which speed / acceleration slips least and most repeatably? (camera as referee)

    python3 tools/slip_doe.py                      # default grid
    python3 tools/slip_doe.py --speeds 0.04 0.08 --accels 0.1 0.25 --repeats 3

Compensation is switched off. For each setting the robot is re-centred with the
camera, then drives right/left 15 cm and up/down 10 cm `repeats` times. Reports
mean and spread of every slip term; the spread is what no open-loop model can
remove, so the best setting is the one with the smallest spread.
Safety: stops if the robot centre comes within 10 cm of the writable edge.
"""
import argparse
import json
import math
import os
import statistics as st
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vision  # noqa: E402
from bench import Robot  # noqa: E402
from sysid_wall import odom_board, wait_still  # noqa: E402
from run_board import square_up  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EDGE_CM = 10.0


def centre(r, cam, w, h):
    for _ in range(4):
        _, v = vision.measure(cam, n=7)
        dx, dy = w / 2 - v["x"], h / 2 - v["y"]
        if abs(dx) < 1 and abs(dy) < 1:
            return v
        r.cmd("move", x=dx / 100, y=dy / 100, speed=0.05, accel=0.15)
        wait_still(r, 30)
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", type=float, nargs="+", default=[0.04, 0.06, 0.08, 0.12])
    ap.add_argument("--accels", type=float, nargs="+", default=[0.1, 0.25])
    ap.add_argument("--repeats", type=int, default=2)
    args = ap.parse_args()
    r = Robot()
    if r.status()["mode"] != "active":
        r.cmd("enable")
    r.cmd("abort")
    r.cmd("beacon", on=True)
    time.sleep(0.5)
    old = r.status()["slip"]
    r.cmd("slip", reset=True)
    cam = vision.Camera()
    w, h = cam.cfg["surface_cm"]
    results = []
    try:
        centre(r, cam, w, h)
        square_up(r, cam)
        r.cmd("origin")
        for speed in args.speeds:
            for accel in args.accels:
                v0 = centre(r, cam, w, h)
                H = None
                rows = []
                for k in range(args.repeats):
                    for name, dx, dy in [("R", 15, 0), ("L", -15, 0), ("U", 0, 10), ("D", 0, -10)]:
                        H, va = vision.measure(cam, n=7, H=H)
                        oa = odom_board(r.status())
                        r.cmd("move", x=dx / 100, y=dy / 100, speed=speed, accel=accel)
                        wait_still(r)
                        H, vb = vision.measure(cam, n=7, H=H)
                        ob = odom_board(r.status())
                        if min(vb["x"], w - vb["x"], vb["y"], h - vb["y"]) < EDGE_CM:
                            raise SystemExit(f"Robot center ({vb['x']:.1f}, {vb['y']:.1f}) is less than {EDGE_CM:g} cm from the edge; stopping")
                        cam_d = (vb["x"] - va["x"], vb["y"] - va["y"], (vb["heading"] - va["heading"] + 180) % 360 - 180)
                        odo_d = (ob[0] - oa[0], ob[1] - oa[1], ob[2] - oa[2])
                        rows.append({"dir": name, "cam": cam_d, "odo": odo_d})
                side = [q["cam"][0] / q["odo"][0] for q in rows if q["dir"] in "LR"]
                sag = [-100 * (q["cam"][1] - q["odo"][1]) / abs(q["odo"][0]) for q in rows if q["dir"] in "LR"]
                turn = [(q["cam"][2] - q["odo"][2]) / (q["odo"][0] / 100) for q in rows if q["dir"] in "LR"]
                up = [q["cam"][1] / q["odo"][1] for q in rows if q["dir"] == "U"]
                down = [q["cam"][1] / q["odo"][1] for q in rows if q["dir"] == "D"]
                drift = [100 * (q["cam"][0] - q["odo"][0]) / abs(q["odo"][1]) for q in rows if q["dir"] in "UD"]
                sd = lambda v: st.pstdev(v) if len(v) > 1 else float("nan")
                res = {"speed": speed, "accel": accel, "rows": rows,
                       "side": (st.mean(side), sd(side)), "sag_cm_m": (st.mean(sag), sd(sag)),
                       "turn_deg_m": (st.mean(turn), sd(turn)), "up": (st.mean(up), sd(up)),
                       "down": (st.mean(down), sd(down)), "vdrift_cm_m": (st.mean(drift), sd(drift))}
                # Unpredictable error per 10 cm of travel, cm: what a fixed model cannot remove.
                res["noise_cm_per_10cm"] = 10 * math.sqrt(sd(side) ** 2 + (sd(sag) / 100) ** 2 + sd(up) ** 2 + sd(down) ** 2)
                results.append(res)
                print(f"speed {speed * 100:4.0f} cm/s accel {accel:.2f}: side {res['side'][0]:.3f}±{res['side'][1]:.3f} "
                      f"sag {res['sag_cm_m'][0]:5.1f}±{res['sag_cm_m'][1]:.1f} cm/m turn {res['turn_deg_m'][0]:+5.1f}±{res['turn_deg_m'][1]:.1f}°/m "
                      f"up {res['up'][0]:.3f}±{res['up'][1]:.3f} down {res['down'][0]:.3f}±{res['down'][1]:.3f} "
                      f"→ unpredictable error {res['noise_cm_per_10cm']:.2f} cm/10cm")
    finally:
        r.cmd("slip", **{k: old[k] for k in ("up", "down", "side", "sag", "yaw", "up_side", "down_side") if k in old})
        r.cmd("stop")
        cam.close()
        path = os.path.join(ROOT, "logs", time.strftime("doe-%m%d-%H%M%S.json"))
        json.dump(results, open(path, "w"), indent=1, ensure_ascii=False)
        print("log:", path)
    if results:
        best = min(results, key=lambda q: q["noise_cm_per_10cm"])
        print(f"Most stable: {best['speed'] * 100:.0f} cm/s, accel {best['accel']:.2f} m/s² (unpredictable error {best['noise_cm_per_10cm']:.2f} cm/10cm)")


if __name__ == "__main__":
    main()
