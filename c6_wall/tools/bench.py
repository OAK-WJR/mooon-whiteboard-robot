#!/usr/bin/env python3
"""Short scripted motion tests over Wi-Fi with telemetry logging.

    python tools/bench.py hold 3              # enable, hold 3 s, release
    python tools/bench.py moves 0.08 1.2      # up/down/left/right at 0.08 m/s for 1.2 s each
    python tools/bench.py wall 20             # hold on the wall for 20 s, keep holding

Directions are board-frame (W = up). Logs go to logs/<time>-<test>.json.
Always ends with a hold command; floor tests release (brake) at the end.
"""
import http.client
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOST = open(os.path.join(ROOT, ".robot_host")).read().strip() if os.path.exists(os.path.join(ROOT, ".robot_host")) else "192.168.4.1"


class Robot:
    def __init__(self, host=HOST):
        self.host = host
        self.conn = http.client.HTTPConnection(host, 80, timeout=3)
        self.log = []

    def _req(self, method, path, body=None):
        for attempt in range(2):
            try:
                self.conn.request(method, path, body=body, headers={"Content-Type": "application/json"} if body else {})
                r = self.conn.getresponse()
                return json.loads(r.read())
            except Exception:
                self.conn.close()
                self.conn = http.client.HTTPConnection(self.host, 80, timeout=3)
                if attempt:
                    raise

    def cmd(self, op, **kw):
        r = self._req("POST", "/api/cmd", json.dumps(dict(op=op, **kw)))
        if not r.get("ok"):
            print(f"  {op}: {r.get('error')}")
        return r

    def status(self):
        s = self._req("GET", "/api/status")
        s["_t"] = time.time()
        self.log.append(s)
        return s

    def drive_for(self, x, y, seconds, w=0.0):
        end = time.time() + seconds
        while time.time() < end:
            self.cmd("drive", x=x, y=y, w=w)
            self.status()
            time.sleep(0.03)
        self.cmd("drive", x=0, y=0, w=0)

    def watch(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.status()
            time.sleep(0.05)

    def save(self, name):
        os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
        path = os.path.join(ROOT, "logs", time.strftime("%m%d-%H%M%S-") + name + ".json")
        json.dump(self.log, open(path, "w"))
        return path


def board(s):
    import math
    a = math.radians(s["mount"] - s.get("front", 0))
    x, y = s["pose"][0], s["pose"][1]
    return math.cos(a) * x - math.sin(a) * y, math.sin(a) * x + math.cos(a) * y


def summary(r, label, since):
    rows = [s for s in r.log if s["_t"] >= since]
    if not rows:
        return
    last = rows[-1]
    X, Y = board(last)
    max_err = [max(abs(s["w"][i][0]) for s in rows) for i in range(3)]
    max_u = [max(abs(s["w"][i][1]) for s in rows) for i in range(3)]
    lag = max(s["lag"] for s in rows)
    print(f"  {label}: X {X*100:6.1f} cm  Y {Y*100:6.1f} cm  yaw {last['pose'][2]*57.3:5.1f}°  "
          f"max lag {lag:4.1f}mm  max err {max_err}  max |u| {[round(u, 2) for u in max_u]}  "
          f"mode {last['mode']} {last['fault']}")


def main():
    test = sys.argv[1] if len(sys.argv) > 1 else "hold"
    r = Robot()
    s = r.status()
    print(f"fw {s['fw']} mode {s['mode']}")
    try:
        r.cmd("enable")
        t = time.time()
        if test == "hold":
            r.watch(float(sys.argv[2]) if len(sys.argv) > 2 else 3)
            summary(r, "hold", t)
        elif test == "moves":
            v = float(sys.argv[2]) if len(sys.argv) > 2 else 0.08
            dur = float(sys.argv[3]) if len(sys.argv) > 3 else 1.2
            r.watch(1.0)
            summary(r, "hold", t)
            for name, x, y in [("up", 0, v), ("down", 0, -v), ("left", -v, 0), ("right", v, 0)]:
                t = time.time()
                r.drive_for(x, y, dur)
                r.watch(1.0)
                summary(r, name, t)
        elif test == "wall":
            r.watch(float(sys.argv[2]) if len(sys.argv) > 2 else 20)
            summary(r, "wall hold", t)
    finally:
        r.cmd("stop")
        if test != "wall":
            r.cmd("release")
        print("log:", r.save(test))


if __name__ == "__main__":
    main()
