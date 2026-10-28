#!/usr/bin/env python3
"""Full slip + round-trip campaign with camera ground truth (camera never corrects the robot).

    python3 tools/campaign.py [--dist 10] [--reps 3] [--trip 20] [--trips 5] [--skip-fit]

1. go to the board centre, square the front up, set the origin;
2. compensation OFF: right/left/up/down moves, camera vs odometry -> new slip model;
3. apply it;
4. horizontal and vertical round trips -> return error (mm, deg).
Everything is appended to logs/campaign-*.json as it happens.
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
MARGIN = 11.0


class Cam:
    """vision.Camera with automatic reopen on read failures."""

    def __init__(self):
        self.cam = vision.Camera()
        self.cfg = self.cam.cfg
        self.H = None

    def measure(self, n=11):
        for attempt in range(6):
            try:
                self.H, v = vision.measure(self.cam, n=n, H=None)
                return v
            except (RuntimeError, SystemExit) as err:
                print(f"    相机: {err}，重开再试", flush=True)
                try:
                    self.cam.close()
                except Exception:
                    pass
                time.sleep(3)
                self.cam = vision.Camera()
        raise SystemExit("相机连续失败")

    def close(self):
        self.cam.close()


def wait_still(r, timeout=40):
    end = time.time() + timeout
    time.sleep(0.3)
    while time.time() < end:
        try:
            s = r.status()
        except Exception:
            time.sleep(0.5)
            continue
        if s["mode"] != "active":
            print(f"    小车不在保持: {s['mode']} {s['fault']}，重新使能", flush=True)
            r.cmd("enable")
            time.sleep(1)
            continue
        if s["src"] == "none" and not s["moving"]:
            time.sleep(0.8)
            return r.status()
        time.sleep(0.15)
    r.cmd("stop")
    return r.status()


def odom_board(s):
    a = math.radians(s["mount"] - s.get("front", 0))
    x, y = s["pose"][0], s["pose"][1]
    return (100 * (math.cos(a) * x - math.sin(a) * y), 100 * (math.sin(a) * x + math.cos(a) * y),
            math.degrees(s["pose"][2]))


def dturn(a, b):
    return (b - a + 180) % 360 - 180


class Campaign:
    def __init__(self, args):
        self.args = args
        self.r = Robot()
        s = self.r.status()
        if s["mode"] != "active":
            self.r.cmd("enable")
            time.sleep(1)
        self.r.cmd("beacon", on=True)
        time.sleep(0.5)
        self.cam = Cam()
        self.front = self.cam.cfg.get("front_from_beacon_deg", 66.0)
        self.W, self.Hh = self.cam.cfg["surface_cm"]
        self.path = os.path.join(ROOT, "logs", time.strftime("campaign-%m%d-%H%M%S.json"))
        self.log = {"start": time.strftime("%F %T"), "args": vars(args), "slip_before": s["slip"], "steps": []}

    def save(self):
        json.dump(self.log, open(self.path, "w"), indent=1, ensure_ascii=False)

    def heading(self, v):
        return v["heading"] + self.front

    def square(self, tol=0.4):
        v = self.cam.measure()
        for _ in range(5):
            err = dturn(self.heading(v), 90)
            if abs(err) < tol:
                break
            self.r.cmd("turn", deg=err, rate=0.4)
            wait_still(self.r)
            v = self.cam.measure()
        return v

    def goto(self, gx, gy, step=5.0):
        for _ in range(30):
            v = self.square(tol=2.0)
            ex, ey = gx - v["x"], gy - v["y"]
            d = math.hypot(ex, ey)
            if d < 1.0:
                return v
            k = min(1.0, step / d)
            self.r.cmd("move", x=ex * k / 100, y=ey * k / 100, speed=0.05, accel=0.15)
            wait_still(self.r)
        return self.cam.measure()

    def move(self, dx, dy, speed, label):
        """Board-frame move (cm) with camera + odometry before/after; refuses to approach an edge."""
        a = self.cam.measure()
        gap = lambda x, y: min(x, self.W - x, y, self.Hh - y)  # noqa: E731
        if gap(a["x"] + dx, a["y"] + dy) < MARGIN and gap(a["x"] + dx, a["y"] + dy) < gap(a["x"], a["y"]):
            print(f"    {label}: 太靠边，先回中间", flush=True)
            self.goto(*self.args.centre)
            a = self.cam.measure()
        oa = odom_board(self.r.status())
        self.r.cmd("move", x=dx / 100, y=dy / 100, speed=speed, accel=self.args.accel)
        s = wait_still(self.r)
        b = self.cam.measure()
        ob = odom_board(s)
        st = {"label": label, "cmd": [dx, dy], "t": time.time(),
              "cam": [b["x"] - a["x"], b["y"] - a["y"], dturn(a["heading"], b["heading"])],
              "odo": [ob[0] - oa[0], ob[1] - oa[1], ob[2] - oa[2]], "at": [b["x"], b["y"], self.heading(b)]}
        self.log["steps"].append(st)
        self.save()
        return st, a, b

    # ---------------------------------------------------------------- phases
    def identify(self):
        print("== 2. 关闭补偿，测原始打滑", flush=True)
        self.r.cmd("slip", reset=True)
        D = self.args.dist
        tests = []
        for k in range(self.args.reps):
            dirs = [("右", 1, 0), ("左", -1, 0)] + ([("上", 0, 1), ("下", 0, -1)] if "y" in self.args.axes else [])
            for name, ux, uy in dirs:
                self.square(tol=1.0)
                st, a, b = self.move(ux * D, uy * D, 0.05, name)
                st["name"] = name
                if abs(st["cam"][2]) > 20:
                    print(f"   {name}{D:g}: 失去抓地 (转 {st['cam'][2]:+.0f}°)，这次不算", flush=True)
                    continue
                tests.append(st)
                c, o = st["cam"], st["odo"]
                print(f"   {name}{D:g}: 编码器 ({o[0]:+6.2f},{o[1]:+6.2f}) cm {o[2]:+5.1f}°   相机 ({c[0]:+6.2f},{c[1]:+6.2f}) cm {c[2]:+5.1f}°", flush=True)
        return tests

    @staticmethod
    def fit(tests):
        mean = lambda v: sum(v) / len(v) if v else float("nan")  # noqa: E731
        up = [t["cam"][1] / t["odo"][1] for t in tests if t["name"] == "上" and abs(t["odo"][1]) > 1]
        down = [t["cam"][1] / t["odo"][1] for t in tests if t["name"] == "下" and abs(t["odo"][1]) > 1]
        side = [t["cam"][0] / t["odo"][0] for t in tests if t["name"] in "左右" and abs(t["odo"][0]) > 1]
        sag = [-(t["cam"][1] - t["odo"][1]) / abs(t["odo"][0]) for t in tests if t["name"] in "左右" and abs(t["odo"][0]) > 1]
        yaw = [math.radians(t["cam"][2] - t["odo"][2]) / (t["odo"][0] / 100) for t in tests if t["name"] in "左右" and abs(t["odo"][0]) > 1]
        up_side = [t["cam"][0] / t["odo"][1] for t in tests if t["name"] == "上" and abs(t["odo"][1]) > 1]
        down_side = [t["cam"][0] / abs(t["odo"][1]) for t in tests if t["name"] == "下" and abs(t["odo"][1]) > 1]
        sd = lambda v: (sum((x - mean(v)) ** 2 for x in v) / max(1, len(v) - 1)) ** 0.5 if len(v) > 1 else 0  # noqa: E731
        fitd = {"up": mean(up), "down": mean(down), "side": mean(side), "sag": mean(sag), "yaw": mean(yaw),
                "up_side": mean(up_side), "down_side": mean(down_side)}
        spread = {"up": sd(up), "down": sd(down), "side": sd(side), "sag": sd(sag), "yaw_deg_per_m": math.degrees(sd(yaw))}
        return fitd, spread

    def trips(self, axis, n, D):
        print(f"== 4. {'横向' if axis == 'x' else '竖向'} {D:g} cm 往返 × {n}", flush=True)
        res = []
        for k in range(n):
            cx, cy = self.args.centre
            start = (cx - D / 2, cy) if axis == "x" else (cx, cy - D / 2)
            v = self.cam.measure()
            if math.hypot(v["x"] - start[0], v["y"] - start[1]) > 3:
                self.goto(*start)
            self.square(tol=0.4)
            self.r.cmd("origin")
            out = (D, 0) if axis == "x" else (0, D)
            st1, a, b = self.move(out[0], out[1], self.args.speed, f"trip{axis}{k}-out")
            st2, b2, c = self.move(-out[0], -out[1], self.args.speed, f"trip{axis}{k}-back")
            err = [10 * (c["x"] - a["x"]), 10 * (c["y"] - a["y"]), dturn(a["heading"], c["heading"])]
            dist = math.hypot(err[0], err[1])
            ok = dist <= 2 and abs(err[2]) <= 1
            res.append({"axis": axis, "err_mm": err[:2], "err_deg": err[2], "dist_mm": dist, "out_cam": st1["cam"], "ok": ok})
            self.log.setdefault("trips", []).append(res[-1])
            self.save()
            print(f"   第 {k + 1} 趟: 去程实际 ({st1['cam'][0]:+6.2f},{st1['cam'][1]:+6.2f}) cm  "
                  f"回到起点误差 {dist:5.2f} mm ({err[0]:+5.2f},{err[1]:+5.2f})  航向 {err[2]:+5.2f}°  {'达标' if ok else '未达标'}", flush=True)
        if res:
            md = sum(t["dist_mm"] for t in res) / len(res)
            mx = sum(t["err_mm"][0] for t in res) / len(res)
            my = sum(t["err_mm"][1] for t in res) / len(res)
            mt = sum(t["err_deg"] for t in res) / len(res)
            print(f"   汇总: 平均误差 {md:.2f} mm, 平均偏移 ({mx:+.2f},{my:+.2f}) mm, 平均航向 {mt:+.2f}°, 达标 {sum(t['ok'] for t in res)}/{len(res)}", flush=True)
        return res

    def run(self):
        try:
            print("== 1. 开到中间，车头转正", flush=True)
            cx, cy = self.args.centre
            v = self.goto(cx, cy)
            v = self.square()
            print(f"   在 X {v['x']:.1f} Y {v['y']:.1f} cm，车头 {self.heading(v):.2f}°", flush=True)
            self.r.cmd("origin")
            if not self.args.skip_fit:
                tests = self.identify()
                fitd, spread = self.fit(tests)
                self.log.update(fit=fitd, spread=spread)
                self.save()
                print("   拟合:", {k: round(x, 4) for k, x in fitd.items()}, flush=True)
                print("   重复性(标准差):", {k: round(x, 4) for k, x in spread.items()}, flush=True)
                print("== 3. 写入新补偿", flush=True)
                clean = {k: (x if math.isfinite(x) else (1.0 if k in ("up", "down", "side") else 0.0)) for k, x in fitd.items()}
                ack = self.r.cmd("slip", **clean)
                print("   ", "已写入" if ack.get("ok") else f"被拒绝 {ack.get('error')}", flush=True)
                if not ack.get("ok"):
                    self.r.cmd("slip", **{k: self.log["slip_before"][k] for k in ("up", "down", "side", "sag", "yaw", "up_side", "down_side")})
            for axis in self.args.axes:
                self.trips(axis, self.args.trips, self.args.trip)
        finally:
            self.r.cmd("stop")
            self.cam.close()
            self.save()
            print("log:", self.path, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=10)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--trip", type=float, default=20)
    ap.add_argument("--trips", type=int, default=5)
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--accel", type=float, default=0.15)
    ap.add_argument("--skip-fit", action="store_true")
    ap.add_argument("--centre", type=float, nargs=2, default=(27.3, 42.5), help="work centre, board cm")
    ap.add_argument("--axes", default="xy")
    Campaign(ap.parse_args()).run()


if __name__ == "__main__":
    main()
