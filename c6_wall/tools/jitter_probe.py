#!/usr/bin/env python3
"""Exercise the robot (small turns and moves) and score wheel chatter while holding.

    python tools/jitter_probe.py [repeats]
Score per wheel: output standard deviation and peak speed in the 2 s hold after each step.
"""
import statistics as st
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from bench import Robot  # noqa: E402


def wait(r, t=8):
    end = time.time() + t
    time.sleep(0.3)
    while time.time() < end:
        s = r.status()
        if s["src"] == "none" and not s["moving"]:
            return
        time.sleep(0.05)


def flips(values, floor=0.05):
    """Sign reversals of a wheel output: the signature of chatter, not of settling."""
    sig = [1 if v > floor else -1 if v < -floor else 0 for v in values]
    sig = [v for v in sig if v]
    return sum(1 for a, b in zip(sig, sig[1:]) if a != b)


def probe(r, repeats=2):
    worst = [0.0, 0.0, 0.0]
    speed = [0, 0, 0]
    reversals = [0, 0, 0]
    for _ in range(repeats):
        for op, kw in [("turn", {"deg": 12}), ("turn", {"deg": -12}), ("move", {"x": 0, "y": 0.04}), ("move", {"x": 0, "y": -0.04}),
                       ("move", {"x": 0.04, "y": 0}), ("move", {"x": -0.04, "y": 0})]:
            r.cmd(op, **kw)
            wait(r)
            n0 = len(r.log)
            r.watch(2.5)
            seg = [x for x in r.log[n0:] if x["_t"] - r.log[n0]["_t"] > 0.5]
            for i in range(3):
                worst[i] = max(worst[i], st.pstdev([x["w"][i][1] for x in seg]))
                speed[i] = max(speed[i], max(abs(x["w"][i][2]) for x in seg))
                reversals[i] += flips([x["w"][i][1] for x in seg])
    return worst, speed, reversals


if __name__ == "__main__":
    r = Robot()
    w, v, f = probe(r, int(sys.argv[1]) if len(sys.argv) > 1 else 2)
    print("hold u sd", [round(x, 3) for x in w], "peak speed", v, "output reversals", f)
