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
    print("静止保持 5 s ...", flush=True)
    hold = record(r, 5)
    for i in range(3):
        u = [x["w"][i][1] for x in hold]
        e = [x["w"][i][0] for x in hold]
        print(f"  轮{i + 1}: 输出 均值 {st.mean(u):+.3f} 标准差 {st.pstdev(u):.3f}  反转 {flips(u):3d}  误差标准差 {st.pstdev(e):.1f} 计数  积分 {hold[-1]['w'][i][3]:+.3f}", flush=True)
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
            line += f"  轮{i + 1} 误差RMS {rms:5.1f} 峰值 {peak:4.0f} 稳定 {settle:.2f}s |u|max {max(abs(v) for v in u):.2f} 停后反转 {flips(ua)}"
        print(line, flush=True)
        report["steps"].append({"op": op, "kw": kw, "dur": dur, "rows": rows, "after": after})
    s = r.status()
    bad1 = [w[5] for w in s["w"]]
    print("汇总（三个轮子对比）:")
    for i in range(3):
        a = agg[i]
        print(f"  轮{i + 1}: 跟踪误差RMS 平均 {st.mean(a['rms']):5.1f}  峰值 最大 {max(a['peak']):4.0f}  "
              f"稳定时间 平均 {st.mean(a['settle']):.2f}s  最大输出 {max(a['umax']):.2f}  停后反转 合计 {sum(a['flips_after'])}  "
              f"编码器错码 +{bad1[i] - bad0[i]}")
    report["summary"] = {i: {k: (st.mean(v) if k in ('rms', 'settle') else (max(v) if k in ('peak', 'umax') else sum(v))) for k, v in agg[i].items()} for i in range(3)}
    path = os.path.join(ROOT, "logs", time.strftime("wheeldiag-%m%d-%H%M%S.json"))
    json.dump(report, open(path, "w"))
    print("log:", path)
    r.cmd("stop")


if __name__ == "__main__":
    main()
