#!/usr/bin/env python3
"""Camera-measured slip identification on the board (system python3, needs OpenCV).

    python3 tools/sysid_wall.py              # identify with compensation off, apply, verify
    python3 tools/sysid_wall.py --verify     # only measure residual error with current settings
    python3 tools/sysid_wall.py --dist 12    # test distance in cm (default 15)

Each test is a straight 'move' in the board frame. Before and after, the robot
pose is taken from the camera (median of several frames) and from the wheel
odometry; the ratio between them is the slip. Results: logs/sysid-*.json.
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import Robot  # noqa: E402
import vision  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def odom_board(s):
    """Wheel-odometry pose in the board frame: x, y (cm) and heading (deg)."""
    a = math.radians(s["mount"] - s.get("front", 0))
    x, y = s["pose"][0], s["pose"][1]
    return (100 * (math.cos(a) * x - math.sin(a) * y), 100 * (math.sin(a) * x + math.cos(a) * y),
            math.degrees(s["pose"][2]))


def wait_still(r, timeout=20):
    end = time.time() + timeout
    time.sleep(0.3)
    while time.time() < end:
        s = r.status()
        if s["mode"] != "active":
            raise SystemExit(f"Robot is not holding: {s['mode']} {s['fault']}")
        if s["src"] == "none" and not s["moving"]:
            time.sleep(0.6)  # let the chassis settle
            return r.status()
        time.sleep(0.15)
    raise SystemExit("Move timed out")


def snapshot(r, cam, H):
    H, v = vision.measure(cam, n=9, H=H)
    s = r.status()
    return H, v, odom_board(s)


def run_tests(r, cam, dist, rounds, label):
    H = None
    H, v0, o0 = snapshot(r, cam, H)
    print(f"Start: camera X {v0['x']:.1f} Y {v0['y']:.1f} cm  {v0['heading']:.1f}°  (frame-to-frame jitter {v0['spread_cm']:.2f} cm)")
    tests = []
    for k in range(rounds):
        for name, dx, dy in [("R", dist, 0), ("L", -dist, 0), ("U", 0, dist), ("D", 0, -dist)]:
            H, va, oa = snapshot(r, cam, H)
            ack = r.cmd("move", x=dx / 100, y=dy / 100)
            if not ack.get("ok"):
                raise SystemExit(f"move rejected: {ack.get('error')}")
            wait_still(r)
            H, vb, ob = snapshot(r, cam, H)
            t = {"name": name, "cmd": [dx, dy],
                 "cam": [vb["x"] - va["x"], vb["y"] - va["y"], (vb["heading"] - va["heading"] + 180) % 360 - 180],
                 "odo": [ob[0] - oa[0], ob[1] - oa[1], ob[2] - oa[2]]}
            tests.append(t)
            print(f"  {label} {name}{dist:g}: encoder ({t['odo'][0]:+6.2f},{t['odo'][1]:+6.2f}) cm  "
                  f"camera ({t['cam'][0]:+6.2f},{t['cam'][1]:+6.2f}) cm  turn camera {t['cam'][2]:+5.1f}° encoder {t['odo'][2]:+5.1f}°")
    return tests


def run_turns(r, cam, deg, rounds):
    """In-place turns: camera vs encoder rotation gives the true wheel contact radius."""
    H = None
    out = []
    for k in range(rounds):
        for sign in (1, -1):
            H, va, oa = snapshot(r, cam, H)
            ack = r.cmd("turn", deg=sign * deg)
            if not ack.get("ok"):
                raise SystemExit(f"turn rejected: {ack.get('error')}")
            wait_still(r)
            H, vb, ob = snapshot(r, cam, H)
            cam_d = (vb["heading"] - va["heading"] + 180) % 360 - 180
            enc_d = ob[2] - oa[2]
            out.append({"cam": cam_d, "enc": enc_d, "shift_cm": math.hypot(vb["x"] - va["x"], vb["y"] - va["y"])})
            print(f"  turn {sign * deg:+g}°: encoder {enc_d:+6.2f}°  camera {cam_d:+6.2f}°  center drift {out[-1]['shift_cm']:.2f} cm")
    return out


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def fit(tests, old):
    """New slip parameters from raw (compensation-off) or residual (compensation-on) tests."""
    up = [t["cam"][1] / t["odo"][1] for t in tests if t["name"] == "U"]
    down = [t["cam"][1] / t["odo"][1] for t in tests if t["name"] == "D"]
    side = [t["cam"][0] / t["odo"][0] for t in tests if t["name"] in "LR"]
    # Sink (m per m, either direction) and turn (rad per m travelled RIGHT, signed) during
    # sideways travel, beyond what the odometry already believes. cm / cm -> m / m.
    sag = [-(t["cam"][1] - t["odo"][1]) / abs(t["odo"][0]) for t in tests if t["name"] in "LR"]
    yaw = [math.radians(t["cam"][2] - t["odo"][2]) / (t["odo"][0] / 100) for t in tests if t["name"] in "LR"]
    return {"up": old["up"] * mean(up), "down": old["down"] * mean(down), "side": old["side"] * mean(side),
            "sag": old["sag"] + mean(sag), "yaw": old["yaw"] + mean(yaw)}, {
        "up": up, "down": down, "side": side, "sag_cm_per_m": [100 * v for v in sag],
        "yaw_deg_per_m": [math.degrees(v) for v in yaw]}


def residual(tests):
    err = [math.hypot(t["cam"][0] - t["cmd"][0], t["cam"][1] - t["cmd"][1]) for t in tests]
    turn = [abs(t["cam"][2]) for t in tests]
    return max(err), mean(err), max(turn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=15)
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--turns", type=float, default=0, help="only run in-place turns of this many degrees")
    args = ap.parse_args()
    r = Robot()
    s = r.status()
    if s["mode"] != "active":
        r.cmd("enable")
    r.cmd("beacon", on=True)
    time.sleep(0.5)
    cam = vision.Camera()
    log = {"start": time.strftime("%F %T"), "slip_before": s["slip"]}
    # Start from the middle of the board so no test reaches the frame (a wheel touching
    # the raised frame carries the robot and corrupts the slip numbers).
    w, h = cam.cfg.get("surface_cm", (54.6, 85.1))
    for _ in range(3):
        _, v = vision.measure(cam, n=7)
        dx, dy = w / 2 - v["x"], h / 2 - v["y"]
        print(f"Robot at X {v['x']:.1f} Y {v['y']:.1f} cm, offset from board center ({dx:+.1f}, {dy:+.1f}) cm")
        if abs(dx) < 1.5 and abs(dy) < 1.5:
            break
        r.cmd("move", x=dx / 100, y=dy / 100, speed=0.06)
        wait_still(r, 30)
    # Square the robot up: its front must point straight up when the origin is set,
    # otherwise every board-frame test is driven at an angle.
    front = cam.cfg.get("front_from_beacon_deg", 66.0)
    for _ in range(3):
        _, v = vision.measure(cam, n=7)
        turn = (90 - (v["heading"] + front) + 180) % 360 - 180
        print(f"Heading {v['heading'] + front:.1f}° (straight up = 90°), need to turn {turn:+.1f}°")
        if abs(turn) < 0.5:
            break
        r.cmd("turn", deg=turn)
        wait_still(r, 20)
    r.cmd("origin")
    old = dict(s["slip"])
    try:
        if args.turns:
            turns = run_turns(r, cam, args.turns, args.rounds)
            ratio = mean([t["enc"] / t["cam"] for t in turns if abs(t["cam"]) > 1])
            radius = s["radius_mm"] * ratio
            log.update(turns=turns, radius_mm_before=s["radius_mm"], radius_mm_fit=radius)
            print(f"encoder/camera turn ratio {ratio:.3f} -> body radius {s['radius_mm']:.1f} mm should be {radius:.1f} mm")
            return
        if not args.verify:
            print("Step 1: compensation off, measure raw slip")
            r.cmd("slip", reset=True)
            raw = run_tests(r, cam, args.dist, args.rounds, "raw")
            neutral = {"up": 1, "down": 1, "side": 1, "sag": 0, "yaw": 0}
            new, detail = fit(raw, neutral)
            log.update(raw=raw, fit=new, detail=detail)
            print("Fit:", {k: round(v, 4) for k, v in new.items()},
                  f"(sag {new['sag'] * 100:.1f} cm/m, turn {math.degrees(new['yaw']):.1f}°/m)")
            ack = r.cmd("slip", **new)
            if not ack.get("ok"):
                print("Fit is out of the allowed range; restoring the previous compensation:", ack.get("error"))
                r.cmd("slip", **{k: old[k] for k in ("up", "down", "side", "sag", "yaw")})
                return
        print("Verify: run again with compensation on")
        check = run_tests(r, cam, args.dist, 1, "verify")
        mx, avg, turn = residual(check)
        log.update(verify=check, residual={"max_cm": mx, "mean_cm": avg, "max_turn_deg": turn})
        print(f"Verify result: position error mean {avg:.2f} cm, max {mx:.2f} cm; max turn {turn:.1f}°")
    finally:
        r.cmd("stop")
        cam.close()
        os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
        path = os.path.join(ROOT, "logs", time.strftime("sysid-%m%d-%H%M%S.json"))
        json.dump(log, open(path, "w"), indent=1, ensure_ascii=False)
        print("log:", path)


if __name__ == "__main__":
    main()
