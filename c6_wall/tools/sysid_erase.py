#!/usr/bin/env python3
"""System identification for erasing (system python3 with OpenCV): how the robot really moves at
erase speed, and how much ink one pass of the 3 cm felt under the magnet takes off.

    python3 tools/sysid_erase.py motion                          # open loop, then with live camera fixes
    python3 tools/sysid_erase.py motion --modes fix --dists 4 8 --centre 27,67.5
    python3 tools/sysid_erase.py erase --line 12,45,42,45        # robot centre path, cm; 2 passes there and back
    python3 tools/sysid_erase.py erase --line 12,45,42,45 --passes 3 --speed 0.03

motion: from a clean patch (--centre), out-and-back moves along 0/45/90/135 deg (so 8 directions)
of every --dists length. The camera measures each move: travel ratio along the command, direction
error, sideways wander during the move and body rotation. "open" sends nothing but moves (what
clean_dirty did); "fix" streams the camera pose to the firmware (op fix) the whole time, as
run_board.py does, so the robot's own servo corrects slip while it drives.

erase: drives the robot centre along --line, back and forth --passes times with live fixes, and
photographs the board after every pass. Ink is the Lab colour distance from the board around each
pixel (1.25 mm pixels, no blob filtering), binned by the distance from the line, using only the
part of the band the robot body does not hide in any photo. That gives the real cleaned width and
how much each extra pass still removes.

Safety: robot centre stays 11 cm from the frame (pauses when the camera sees it outside), moves are
refused if the camera result disagrees badly. Results: logs/sysid-erase-*.json / .jpg / .mp4.
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import Robot  # noqa: E402
import vision  # noqa: E402
from clean_dirty import ClipRecorder, Tracker, dturn, rectify  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MARGIN = 11.0
PPC = 8.0  # ink analysis resolution, px per cm


def wait_still(r, trk, timeout=40):
    end = time.time() + timeout
    time.sleep(0.3)
    while time.time() < end:
        if trk.out:
            raise SystemExit(f"Robot center ({trk.out[0]:.1f}, {trk.out[1]:.1f}) left the safe area; paused")
        s = r.status()
        if s["mode"] != "active":
            raise SystemExit(f"Robot is not holding: {s['mode']} {s['fault']}")
        if s["src"] == "none" and not s["moving"]:
            time.sleep(0.5)  # let the chassis settle
            return r.status()
        time.sleep(0.1)
    r.cmd("pause")
    raise SystemExit("Move timed out; stopped")


def odom_board(s):
    a = math.radians(s["mount"] - s.get("front", 0))
    x, y = s["pose"][0], s["pose"][1]
    return (100 * (math.cos(a) * x - math.sin(a) * y), 100 * (math.sin(a) * x + math.cos(a) * y),
            math.degrees(s["pose"][2]))


def measure(rec, H, n=7):
    return vision.measure(rec, n=n, H=H)[1]


def move(r, trk, dx, dy, speed, accel):
    ack = r.cmd("move", x=dx / 100, y=dy / 100, speed=speed, accel=accel)
    if not ack.get("ok"):
        raise SystemExit(f"move rejected: {ack.get('error')}")
    return wait_still(r, trk, timeout=15 + math.hypot(dx, dy) / (100 * speed) * 3)


def goto(r, rec, trk, H, tgt, speed, accel, tol=0.8):
    for _ in range(5):
        v = measure(rec, H)
        dx, dy = tgt[0] - v["x"], tgt[1] - v["y"]
        if math.hypot(dx, dy) < tol:
            return v
        move(r, trk, dx, dy, speed, accel)
    return measure(rec, H)


def square_up(r, rec, trk, H, tol=1.0):
    front = rec.cfg.get("front_from_beacon_deg", 66.0)
    for _ in range(4):
        v = measure(rec, H)
        turn = dturn(v["heading"] + front, 90)
        if abs(turn) < tol:
            return v
        r.cmd("turn", deg=turn)
        wait_still(r, trk)
    return measure(rec, H)


def wander(points, a, cmd):
    """Largest sideways distance (cm) of the camera track from the commanded straight line."""
    d = math.hypot(*cmd)
    if d < 1e-6 or not points:
        return 0.0
    ux, uy = cmd[0] / d, cmd[1] / d
    return max(abs((x - a[0]) * -uy + (y - a[1]) * ux) for _, x, y, _ in points)


# ----------------------------------------------------------------------------------------- motion
def run_motion(args, r, rec, trk, H):
    c = args.centre
    tests = []
    for mode in args.modes:
        print(f"\n== {'Open loop (move only)' if mode == 'open' else 'Live camera correction (fix)'}", flush=True)
        # Re-zero so the first fix does not yank the robot by the drift collected so far.
        trk.fix = False
        v = square_up(r, rec, trk, H)
        r.cmd("origin")
        trk.set_origin(r, v)
        trk.fix = mode == "fix"
        time.sleep(1.5 if trk.fix else 0.2)
        for d in args.dists:
            for ang in (0, 45, 90, 135):
                for rep in range(args.reps):
                    u = (math.cos(math.radians(ang)), math.sin(math.radians(ang)))
                    rec.text = f"SysID motion [{mode}]: positioning for {d:g} cm at {ang} deg"
                    goto(r, rec, trk, H, (c[0] - d / 2 * u[0], c[1] - d / 2 * u[1]), args.speed, args.accel)
                    v = measure(rec, H)
                    if abs(dturn(v["heading"] + rec.cfg.get("front_from_beacon_deg", 66.0), 90)) > 1.5:
                        square_up(r, rec, trk, H)
                    for sgn in (1, -1):
                        cmd = (sgn * d * u[0], sgn * d * u[1])
                        heading = (ang if sgn > 0 else ang + 180) % 360
                        rec.text = f"SysID motion [{mode}]: {d:g} cm toward {heading} deg"
                        a = measure(rec, H)
                        oa = odom_board(r.status())
                        t0 = time.time()
                        rec.target = (a["x"] + cmd[0], a["y"] + cmd[1])
                        s = move(r, trk, cmd[0], cmd[1], args.speed, args.accel)
                        dt = time.time() - t0
                        b = measure(rec, H)
                        ob = odom_board(s)
                        cam = (b["x"] - a["x"], b["y"] - a["y"])
                        along = (cam[0] * cmd[0] + cam[1] * cmd[1]) / d
                        cross = (cam[1] * cmd[0] - cam[0] * cmd[1]) / d  # + = left of the travel direction
                        t = {"mode": mode, "dist": d, "dir": heading, "cmd": cmd, "cam": cam,
                             "odo": (ob[0] - oa[0], ob[1] - oa[1]), "along": along / d,
                             "dir_err_deg": math.degrees(math.atan2(cross, along)),
                             "end_err": math.hypot(cam[0] - cmd[0], cam[1] - cmd[1]),
                             "wander": wander(trk.since(t0 + 0.2), (a["x"], a["y"]), cmd),
                             "turn": dturn(a["heading"], b["heading"]), "time": dt}
                        tests.append(t)
                        print(f"  {mode:4s} {d:4.1f} cm → {heading:3d}°: moved {100 * t['along']:5.1f}%  "
                              f"dir err {t['dir_err_deg']:+5.1f}°  end err {t['end_err']:.2f} cm  "
                              f"wander {t['wander']:.2f} cm  turn {t['turn']:+.1f}°", flush=True)
                        if (d >= 4 and t["along"] < 0.3) or t["along"] > 1.8 or abs(t["turn"]) > 12:
                            r.cmd("pause")
                            raise SystemExit("This step is too far from the command; stopped to check")
    trk.fix = False
    return tests


def summarize_motion(tests):
    rows = []
    for mode in sorted({t["mode"] for t in tests}):
        for d in sorted({t["dist"] for t in tests}):
            g = [t for t in tests if t["mode"] == mode and t["dist"] == d]
            if not g:
                continue
            m = lambda k: float(np.mean([t[k] for t in g]))  # noqa: E731
            rows.append({"mode": mode, "dist": d, "n": len(g), "along_pct": 100 * m("along"),
                         "along_min_pct": 100 * min(t["along"] for t in g),
                         "dir_err_abs_deg": float(np.mean([abs(t["dir_err_deg"]) for t in g])),
                         "end_err_cm": m("end_err"), "end_err_max_cm": max(t["end_err"] for t in g),
                         "wander_cm": m("wander"), "turn_abs_deg": float(np.mean([abs(t["turn"]) for t in g]))})
    print("\nmode  dist    n   moved% (min)   dir|°|  end err mean/max cm     wander cm   turn|°|")
    for w in rows:
        print(f"{w['mode']:5s} {w['dist']:4.1f}  {w['n']:3d}   {w['along_pct']:5.1f} ({w['along_min_pct']:5.1f})"
              f"   {w['dir_err_abs_deg']:5.1f}      {w['end_err_cm']:5.2f} / {w['end_err_max_cm']:5.2f}"
              f"          {w['wander_cm']:5.2f}       {w['turn_abs_deg']:4.1f}")
    by_dir = {}
    for t in tests:
        if t["dist"] == max(x["dist"] for x in tests):
            by_dir.setdefault((t["mode"], t["dir"]), []).append(t)
    print("\nLongest distance by direction (0°=right, 90°=up):")
    for (mode, ang), g in sorted(by_dir.items()):
        print(f"  {mode:5s} {ang:3d}°: moved {100 * np.mean([t['along'] for t in g]):5.1f}%  "
              f"dir err {np.mean([t['dir_err_deg'] for t in g]):+5.1f}°  end err {np.mean([t['end_err'] for t in g]):.2f} cm")
    return rows


# ------------------------------------------------------------------------------------------ erase
def ink_map(img, H, surface):
    """Lab colour distance from the board around each pixel (rectified, PPC px per cm)."""
    top, _ = rectify(img, H, surface, ppc=PPC)
    lab = cv2.cvtColor(top, cv2.COLOR_BGR2LAB)
    k = int(8 * PPC) | 1
    bg = np.dstack([cv2.medianBlur(lab[:, :, i], k) for i in range(3)]).astype(np.float32)
    d = lab.astype(np.float32) - bg
    return np.sqrt(d[:, :, 0] ** 2 + (1.6 * d[:, :, 1]) ** 2 + (1.6 * d[:, :, 2]) ** 2), top


def still_image(rec, n=5):
    """Pixel median of n frames: hides the beacon flicker and camera noise."""
    return np.median(np.stack([rec.frame() for _ in range(n)]), axis=0).astype(np.uint8)


def run_erase(args, r, rec, trk, H):
    surface = rec.cfg["surface_cm"]
    x0, y0, x1, y1 = args.line
    L = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / L, (y1 - y0) / L
    trk.fix = not args.open
    time.sleep(1.5)
    rec.text = "SysID erase: going to the start of the test line"
    rec.route = [(x0, y0), (x1, y1)]
    goto(r, rec, trk, H, (x0, y0), args.speed, args.accel, tol=0.5)
    square_up(r, rec, trk, H)
    shots = [(still_image(rec), measure(rec, H))]
    print(f"Start ({shots[0][1]['x']:.1f}, {shots[0][1]['y']:.1f}), line length {L:.1f} cm", flush=True)
    passes = []
    for k in range(args.passes):
        fwd = k % 2 == 0
        a = measure(rec, H)
        tgt = (x1, y1) if fwd else (x0, y0)
        rec.text = f"SysID erase: pass {k + 1}/{args.passes} at {100 * args.speed:.0f} cm/s"
        rec.target = tgt
        t0 = time.time()
        move(r, trk, tgt[0] - a["x"], tgt[1] - a["y"], args.speed, args.accel)
        pts = trk.since(t0)
        off = [(x - x0) * -uy + (y - y0) * ux for _, x, y, _ in pts
               if 0.1 * L <= (x - x0) * ux + (y - y0) * uy <= 0.9 * L]
        b = measure(rec, H)
        shots.append((still_image(rec), b))
        passes.append({"pass": k + 1, "time": time.time() - t0,
                       "track_off_mean": float(np.mean(off)) if off else None,
                       "track_off_max": float(np.max(np.abs(off))) if off else None,
                       "end_err": math.hypot(b["x"] - tgt[0], b["y"] - tgt[1])})
        print(f"  Pass {k + 1}: robot center off the line mean {passes[-1]['track_off_mean'] or 0:+.2f} cm, "
              f"max {passes[-1]['track_off_max'] or 0:.2f} cm; end err {passes[-1]['end_err']:.2f} cm", flush=True)
    trk.fix = False

    # Profile across the line, only where no photo has the robot body over it.
    maps = [ink_map(img, H, surface) for img, _ in shots]
    h, w = maps[0][0].shape
    xs = (np.arange(w) + 0.5) / PPC
    ys = surface[1] - (np.arange(h) + 0.5) / PPC
    X, Y = np.meshgrid(xs, ys)
    s_al = (X - x0) * ux + (Y - y0) * uy
    s_off = (X - x0) * -uy + (Y - y0) * ux
    vis = (s_al >= 0) & (s_al <= L) & (np.abs(s_off) <= args.band)
    for _, p in shots:
        vis &= np.hypot(X - p["x"], Y - p["y"]) > args.robot_radius
    vis_len = float(np.ptp(s_al[vis])) if vis.any() else 0.0
    if vis_len < 2:
        raise SystemExit("Line too short: the robot body covers all of it; use a longer line (at least 26 cm)")
    bins = np.arange(-args.band, args.band + 1e-6, 0.25)
    idx = np.digitize(s_off[vis], bins) - 1
    prof = []
    for j in range(len(bins) - 1):
        sel = idx == j
        row = {"off": float(bins[j] + 0.125)}
        for k, (dE, _) in enumerate(maps):
            v = dE[vis][sel]
            row[f"ink{k}"] = float(np.sum(np.clip(v - 10, 0, None)))  # ink "mass"
            row[f"px{k}"] = int(np.count_nonzero(v > 22))
        prof.append(row)
    tot0 = sum(p["ink0"] for p in prof) or 1.0
    print(f"\nVisible segment length {vis_len:.1f} cm. Distance from the line vs. ink left (pass 0 = before erasing = 100%):")
    print("  offset cm   " + "  ".join(f"pass{k}" for k in range(len(maps))))
    for p in prof:
        if abs(p["off"]) <= args.band and p["ink0"] > 0.003 * tot0:
            print(f"  {p['off']:+6.2f}   " + "  ".join(f"{100 * p[f'ink{k}'] / p['ink0']:5.0f}%" for k in range(len(maps))))
    summary = {}
    for half in (0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0):
        sel = [p for p in prof if abs(p["off"]) <= half]
        i0 = sum(p["ink0"] for p in sel) or 1.0
        summary[half] = [sum(p[f"ink{k}"] for p in sel) / i0 for k in range(len(maps))]
    print("\nInk left within ±x cm on both sides of the line:")
    for half, v in summary.items():
        print(f"  ±{half:4.2f} cm: " + "  ".join(f"{100 * q:5.1f}%" for q in v))

    # Picture: before / after every pass, cropped to the band.
    crops = []
    for dE, top in maps:
        img = top.copy()
        img[~vis] = (img[~vis] * 0.45).astype(np.uint8)
        p0 = (int(x0 * PPC), int((surface[1] - y0) * PPC))
        p1 = (int(x1 * PPC), int((surface[1] - y1) * PPC))
        cv2.line(img, p0, p1, (255, 120, 20), 1, cv2.LINE_AA)
        bx0 = max(0, min(p0[0], p1[0]) - int(args.band * PPC) - 8)
        bx1 = min(w, max(p0[0], p1[0]) + int(args.band * PPC) + 8)
        by0 = max(0, min(p0[1], p1[1]) - int(args.band * PPC) - 8)
        by1 = min(h, max(p0[1], p1[1]) + int(args.band * PPC) + 8)
        crops.append(cv2.resize(img[by0:by1, bx0:bx1], None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST))
    for k, c in enumerate(crops):
        cv2.putText(c, "before" if k == 0 else f"after pass {k}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2)
    pic = np.vstack(crops) if (x1 - x0) ** 2 > (y1 - y0) ** 2 else np.hstack(crops)
    return {"line": args.line, "speed": args.speed, "fix": not args.open, "passes": passes, "profile": prof,
            "visible_len_cm": vis_len, "remaining": {str(k): v for k, v in summary.items()}}, pic


# ------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["motion", "erase"])
    ap.add_argument("--speed", type=float, default=0.05, help="m/s")
    ap.add_argument("--accel", type=float, default=0.15, help="m/s^2")
    ap.add_argument("--centre", default="27,67.5", help="motion: clean patch to test in, cm")
    ap.add_argument("--dists", type=float, nargs="+", default=[2, 4, 8], help="motion: move lengths, cm")
    ap.add_argument("--modes", nargs="+", default=["open", "fix"], choices=["open", "fix"])
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--line", help="erase: x0,y0,x1,y1 robot centre path, cm (>= 26 cm long)")
    ap.add_argument("--passes", type=int, default=2)
    ap.add_argument("--band", type=float, default=6.0, help="erase: half width of the analysed band, cm")
    ap.add_argument("--robot-radius", type=float, default=11.0, help="erase: body radius hidden in photos, cm")
    ap.add_argument("--open", action="store_true", help="erase: no live camera fixes")
    args = ap.parse_args()
    args.centre = tuple(float(v) for v in args.centre.split(","))
    if args.what == "erase":
        if not args.line:
            raise SystemExit("erase needs --line x0,y0,x1,y1")
        args.line = tuple(float(v) for v in args.line.split(","))

    cfg = vision.config()
    W, Hs = cfg["surface_cm"]
    box = (MARGIN, W - MARGIN, MARGIN, Hs - MARGIN)
    pts = [args.centre] if args.what == "motion" else [args.line[:2], args.line[2:]]
    for p in pts:
        if not (box[0] <= p[0] <= box[1] and box[2] <= p[1] <= box[3]):
            raise SystemExit(f"({p[0]}, {p[1]}) is less than {MARGIN} cm from the frame")

    r = Robot()
    if r.status()["mode"] != "active":
        r.cmd("enable")
    r.cmd("abort")
    r.cmd("beacon", on=True)
    time.sleep(0.5)
    stamp = time.strftime("%m%d-%H%M%S")
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    base = os.path.join(ROOT, "logs", f"sysid-erase-{stamp}-{args.what}")
    rec = ClipRecorder(vision.Camera(cfg), base + ".mp4")
    trk = None
    log = {"start": time.strftime("%F %T"), "args": {k: v for k, v in vars(args).items()}, "slip": r.status().get("slip")}
    try:
        img = rec.frame()
        H = vision.surface_homography(img, cfg)
        if H is None:
            raise SystemExit("Cannot find the red tape on the four corners")
        rec.H, rec.box = H, box
        trk = Tracker(rec, H, box)
        start = square_up(r, rec, trk, H)
        r.cmd("origin")
        trk.set_origin(r, start)
        if args.what == "motion":
            tests = run_motion(args, r, rec, trk, H)
            log.update(tests=tests, summary=summarize_motion(tests))
        else:
            res, pic = run_erase(args, r, rec, trk, H)
            log.update(res)
            cv2.imwrite(base + ".jpg", pic)
            print("Image:", base + ".jpg")
        log["fixes"] = {"sent": trk.sent, "rejected": trk.rejected}
    finally:
        r.cmd("pause")
        if trk:
            trk.close()
        rec.close()
        json.dump(log, open(base + ".json", "w"), indent=1, ensure_ascii=False, default=float)
        print("Record:", base + ".json", "\nVideo:", base + ".mp4")


if __name__ == "__main__":
    main()
