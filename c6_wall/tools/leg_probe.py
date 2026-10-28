#!/usr/bin/env python3
"""When does the heading jump during a leg?  Camera heading at ~10 Hz + wheel telemetry at ~25 Hz.

    python3 tools/leg_probe.py --axis y --dist 20 --reps 3

Prints, per leg, the camera heading change in the acceleration / cruise / braking / settle phases
and the wheel behaviour (peak error, peak duty, stuck episodes) in the phase with the largest turn.
Logs: logs/legprobe-*.json
"""
import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import Robot  # noqa: E402
import vision  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def poll(r, out, stop):
    rr = Robot()
    while not stop.is_set():
        try:
            out.append(rr.status())
        except Exception:
            pass
        time.sleep(0.03)


def leg(r, cam, H, dx, dy, speed, accel):
    tel, stop = [], threading.Event()
    th = threading.Thread(target=poll, args=(r, tel, stop), daemon=True)
    th.start()
    heads = []
    t0 = time.time()
    r.cmd("move", x=dx / 100, y=dy / 100, speed=speed, accel=accel)
    moving_seen, t_end = False, None
    while time.time() - t0 < 30:
        img = cam.frame()
        _, p = vision.locate(img, cam.cfg, H)
        if p:
            heads.append((time.time() - t0, p["heading"], p["x"], p["y"]))
        if tel:
            s = tel[-1]
            if s["moving"]:
                moving_seen = True
            if (moving_seen or time.time() - t0 > 1.0) and s["src"] == "none" and not s["moving"]:
                t_end = t_end or time.time() - t0
                if time.time() - t0 > t_end + 1.5:
                    break
    stop.set()
    th.join(1)
    return heads, [dict(t=s["_t"] - t0, w=s["w"], lag=s["lag"], moving=s["moving"]) for s in tel], t_end


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", default="y")
    ap.add_argument("--dist", type=float, default=20)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--speed", type=float, default=0.03)
    ap.add_argument("--accel", type=float, default=0.08)
    ap.add_argument("--yaw", type=float, default=None, help="square front up, set origin, then turn by this (deg)")
    ap.add_argument("--raw", action="store_true", help="slip compensation off (restored at the end)")
    a = ap.parse_args()
    r = Robot()
    if r.status()["mode"] != "active":
        r.cmd("enable")
        time.sleep(1)
    r.cmd("beacon", on=True)
    cam = vision.Camera()
    H, v = vision.measure(cam, n=5)
    slip0 = r.status()["slip"]
    if a.raw:
        r.cmd("slip", reset=True)
    if a.yaw is not None:
        from campaign import wait_still
        for _ in range(4):
            H, v = vision.measure(cam, n=9)
            err = (90 - (v["heading"] + cam.cfg["front_from_beacon_deg"]) + 180) % 360 - 180
            if abs(err) < 0.4:
                break
            r.cmd("turn", deg=err, rate=0.4)
            wait_still(r)
        r.cmd("origin")
        if a.yaw:
            r.cmd("turn", deg=a.yaw, rate=0.4)
            wait_still(r)
    D = a.dist
    T = D / 100 / a.speed + a.speed / a.accel   # nominal motion time
    ta = a.speed / a.accel                      # acceleration time
    out = []
    try:
        for k in range(a.reps):
            for sign in (1, -1):
                dx, dy = (sign * D, 0) if a.axis == "x" else (0, sign * D)
                heads, tel, t_end = leg(r, cam, H, dx, dy, a.speed, a.accel)
                if len(heads) < 5:
                    print("  相机帧太少，跳过")
                    continue
                h0 = heads[0][1]
                rel = [(t, (h - h0 + 180) % 360 - 180) for t, h, _, _ in heads]

                def at(tq):
                    best = min(rel, key=lambda x: abs(x[0] - tq))
                    return best[1]
                phases = {"起步": (0, ta), "匀速": (ta, T - ta), "刹车": (T - ta, T), "停稳": (T, (t_end or T) + 1.4)}
                dturn = {n: at(b) - at(a_) for n, (a_, b) in phases.items()}
                total = rel[-1][1]
                worst = max(dturn, key=lambda n: abs(dturn[n]))
                w0, w1 = phases[worst]
                seg = [x for x in tel if w0 <= x["t"] <= w1] or tel
                wheel = []
                for i in range(3):
                    errs = [abs(x["w"][i][0]) for x in seg]
                    us = [abs(x["w"][i][1]) for x in seg]
                    stuck = sum(1 for x in seg if abs(x["w"][i][2]) < 25 and abs(x["w"][i][0]) > 4 and x["moving"])
                    wheel.append(f"轮{i + 1} 误差峰 {max(errs) if errs else 0:3.0f} 输出峰 {max(us) if us else 0:.2f} 卡住 {stuck}")
                name = ("上" if dy > 0 else "下") if a.axis == "y" else ("右" if dx > 0 else "左")
                print(f"  {name}{D:g}: 总转 {total:+5.2f}°  " + "  ".join(f"{n} {v:+5.2f}°" for n, v in dturn.items())
                      + f"   | 最大在[{worst}]: " + "; ".join(wheel), flush=True)
                out.append({"dir": name, "heads": rel, "tel": tel, "phases": dturn, "total": total})
    finally:
        r.cmd("stop")
        if a.raw:
            r.cmd("slip", **{k: slip0[k] for k in slip0 if k != "on"})
        cam.close()
        if out:
            for nm in sorted(set(o["dir"] for o in out)):
                t = [o["total"] for o in out if o["dir"] == nm]
                m = sum(t) / len(t)
                sdv = (sum((x - m) ** 2 for x in t) / max(1, len(t) - 1)) ** 0.5
                print(f"  {nm}: 转角 均值 {m:+.2f}°  标准差 {sdv:.2f}°  (n={len(t)})")
        path = os.path.join(ROOT, "logs", time.strftime("legprobe-%m%d-%H%M%S.json"))
        json.dump(out, open(path, "w"))
        print("log:", path)


if __name__ == "__main__":
    main()
