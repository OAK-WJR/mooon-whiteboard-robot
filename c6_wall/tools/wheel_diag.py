#!/usr/bin/env python3
"""Per-wheel health check on the wall: hold, small turns, small moves (no camera needed).

    python3 tools/wheel_diag.py

Per wheel: tracking error during motion (RMS / peak counts), settling after the move,
output chatter while holding (sign reversals, output s.d.), peak duty, encoder errors.
The wheel whose numbers stand out is the one to look at (gains, gearbox, encoder, contact).
"""
import json
import math
import os
import statistics as st
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import Robot  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def flips(values, floor=0.04):
    sig = [1 if v > floor else -1 if v < -floor else 0 for v in values]
    sig = [v for v in sig if v]
    return sum(1 for a, b in zip(sig, sig[1:]) if a != b)


def record(r, seconds):
    out, end = [], time.time() + seconds
    while time.time() < end:
        try:
            out.append(r.status())
        except Exception:
            pass
        time.sleep(0.04)
    return out


def run_step(r, op, kw, hold_after=2.0):
    t0 = time.time()
    r.cmd(op, **kw)
    rows = []
    end = time.time() + 15
    time.sleep(0.1)
    while time.time() < end:
        try:
            s = r.status()
        except Exception:
            continue
        rows.append(s)
        if s["src"] == "none" and not s["moving"] and time.time() - t0 > 0.4:
            break
        time.sleep(0.04)
    t_end = time.time()
    rows_after = record(r, hold_after)
    return rows, rows_after, t_end - t0


def main():
    r = Robot()
    s = r.status()
    bad0 = [w[5] for w in s["w"]]
    if s["mode"] != "active":
        r.cmd("enable")
        time.sleep(1)
    report = {"fw": s["fw"], "gains": s["gains"], "steps": []}
    print("Holding still for 5 s ...", flush=True)
    hold = record(r, 5)
    for i in range(3):
        u = [x["w"][i][1] for x in hold]
        e = [x["w"][i][0] for x in hold]
        print(f"  wheel{i + 1}: output mean {st.mean(u):+.3f} std dev {st.pstdev(u):.3f}  reversals {flips(u):3d}  error std dev {st.pstdev(e):.1f} counts  integral {hold[-1]['w'][i][3]:+.3f}", flush=True)
    report["hold"] = [{"u_mean": st.mean([x['w'][i][1] for x in hold]), "u_sd": st.pstdev([x['w'][i][1] for x in hold]),
                       "flips": flips([x['w'][i][1] for x in hold])} for i in range(3)]
    steps = [("turn", {"deg": 10, "rate": 0.5}), ("turn", {"deg": -10, "rate": 0.5}),
             ("move", {"x": 0, "y": 0.03, "speed": 0.04}), ("move", {"x": 0, "y": -0.03, "speed": 0.04}),
             ("move", {"x": 0.03, "y": 0, "speed": 0.04}), ("move", {"x": -0.03, "y": 0, "speed": 0.04})]
    agg = {i: {"rms": [], "peak": [], "settle": [], "umax": [], "flips_after": []} for i in range(3)}
    for op, kw in steps:
        rows, after, dur = run_step(r, op, kw)
        line = f"  {op} {kw.get('deg', '')}{kw.get('x', '') if op == 'move' else ''} {kw.get('y', '') if op == 'move' else ''}:"
        for i in range(3):
            e = [x["w"][i][0] for x in rows] or [0]
            u = [x["w"][i][1] for x in rows] or [0]
            rms = math.sqrt(sum(v * v for v in e) / len(e))
            peak = max(abs(v) for v in e)
            ua = [x["w"][i][1] for x in after] or [0]
            ea = [x["w"][i][0] for x in after] or [0]
            settle = next((k * 0.04 for k, v in enumerate(ea) if all(abs(w) <= 2 for w in ea[k:])), 9.9)
            agg[i]["rms"].append(rms); agg[i]["peak"].append(peak); agg[i]["settle"].append(settle)
            agg[i]["umax"].append(max(abs(v) for v in u)); agg[i]["flips_after"].append(flips(ua))
            line += f"  wheel{i + 1} error RMS {rms:5.1f} peak {peak:4.0f} settle {settle:.2f}s |u|max {max(abs(v) for v in u):.2f} reversals after stop {flips(ua)}"
        print(line, flush=True)
        report["steps"].append({"op": op, "kw": kw, "dur": dur, "rows": rows, "after": after})
    s = r.status()
    bad1 = [w[5] for w in s["w"]]
    print("Summary (three wheels compared):")
    for i in range(3):
        a = agg[i]
        print(f"  wheel{i + 1}: tracking error RMS mean {st.mean(a['rms']):5.1f}  peak max {max(a['peak']):4.0f}  "
              f"settling time mean {st.mean(a['settle']):.2f}s  max output {max(a['umax']):.2f}  reversals after stop total {sum(a['flips_after'])}  "
              f"encoder bad codes +{bad1[i] - bad0[i]}")
    report["summary"] = {i: {k: (st.mean(v) if k in ('rms', 'settle') else (max(v) if k in ('peak', 'umax') else sum(v))) for k, v in agg[i].items()} for i in range(3)}
    path = os.path.join(ROOT, "logs", time.strftime("wheeldiag-%m%d-%H%M%S.json"))
    json.dump(report, open(path, "w"))
    print("log:", path)
    r.cmd("stop")


if __name__ == "__main__":
    main()
