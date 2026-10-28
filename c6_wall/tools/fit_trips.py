#!/usr/bin/env python3
"""Refine the slip model from compensated round-trip legs (campaign log).

    python3 tools/fit_trips.py logs/campaign-XXXX.json            # print the proposed parameters
    python3 tools/fit_trips.py logs/campaign-XXXX.json --apply    # and send them to the robot

Each leg ran with the compensation in `slip_before` (or the fit written at step 3). The camera
residual of a leg tells how wrong the model is in that direction:
  ratio:  true = model * (camera travel / commanded travel)
  sag:    true = model - camera dy / |dx|           (sideways legs)
  side drift on vertical legs: true = model + camera dx / |dy|
  yaw:    true = model + camera turn / signed travel  (per direction)
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

KEYS = ("up", "down", "side", "sag", "yaw", "up_side", "down_side", "side_left", "sag_left", "yaw_left", "yaw_up", "yaw_down")


def mean(v):
    return sum(v) / len(v) if v else None


def sd(v):
    if len(v) < 2:
        return 0.0
    m = mean(v)
    return (sum((x - m) ** 2 for x in v) / (len(v) - 1)) ** 0.5


def main():
    path = sys.argv[1]
    log = json.load(open(path))
    model = dict(log.get("slip_before", {}))
    if "fit" in log:
        model.update(log["fit"])  # what was written at step 3 (= what the trips ran with)
    for k in KEYS:
        model.setdefault(k, 1.0 if k in ("up", "down", "side") else 0.0)
    legs = [s for s in log["steps"] if s["label"].startswith("trip")]
    R = [s for s in legs if s["cmd"][0] > 0]
    L = [s for s in legs if s["cmd"][0] < 0]
    U = [s for s in legs if s["cmd"][1] > 0]
    D = [s for s in legs if s["cmd"][1] < 0]
    new = dict(model)
    rep = {}
    if R:
        ratio = [s["cam"][0] / s["cmd"][0] for s in R]
        new["side"] = model["side"] * mean(ratio)
        new["sag"] = model["sag"] - mean([s["cam"][1] / abs(s["cmd"][0]) for s in R])
        new["yaw"] = model["yaw"] + mean([math.radians(s["cam"][2]) / (s["cmd"][0] / 100) for s in R])
        rep["right"] = {"ratio": (mean(ratio), sd(ratio)), "dy_cm": (mean([s["cam"][1] for s in R]), sd([s["cam"][1] for s in R])),
                        "turn_deg": (mean([s["cam"][2] for s in R]), sd([s["cam"][2] for s in R]))}
    if L:
        ratio = [s["cam"][0] / s["cmd"][0] for s in L]
        left_ratio = (model["side"] + model["side_left"]) * mean(ratio)
        new["side_left"] = left_ratio - new["side"]
        left_sag = (model["sag"] + model["sag_left"]) - mean([s["cam"][1] / abs(s["cmd"][0]) for s in L])
        new["sag_left"] = left_sag - new["sag"]
        kL_model = model["yaw_left"] - model["yaw"]
        kL_true = kL_model + mean([math.radians(s["cam"][2]) / (abs(s["cmd"][0]) / 100) for s in L])
        new["yaw_left"] = kL_true + new["yaw"]
        rep["left"] = {"ratio": (mean(ratio), sd(ratio)), "dy_cm": (mean([s["cam"][1] for s in L]), sd([s["cam"][1] for s in L])),
                       "turn_deg": (mean([s["cam"][2] for s in L]), sd([s["cam"][2] for s in L]))}
    if U:
        ratio = [s["cam"][1] / s["cmd"][1] for s in U]
        new["up"] = model["up"] * mean(ratio)
        new["up_side"] = model["up_side"] + mean([s["cam"][0] / s["cmd"][1] for s in U])
        new["yaw_up"] = model["yaw_up"] + mean([math.radians(s["cam"][2]) / (s["cmd"][1] / 100) for s in U])
        rep["up"] = {"ratio": (mean(ratio), sd(ratio)), "dx_cm": (mean([s["cam"][0] for s in U]), sd([s["cam"][0] for s in U])),
                     "turn_deg": (mean([s["cam"][2] for s in U]), sd([s["cam"][2] for s in U]))}
    if D:
        ratio = [s["cam"][1] / s["cmd"][1] for s in D]
        new["down"] = model["down"] * mean(ratio)
        new["down_side"] = model["down_side"] + mean([s["cam"][0] / abs(s["cmd"][1]) for s in D])
        new["yaw_down"] = model["yaw_down"] + mean([math.radians(s["cam"][2]) / (abs(s["cmd"][1]) / 100) for s in D])
        rep["down"] = {"ratio": (mean(ratio), sd(ratio)), "dx_cm": (mean([s["cam"][0] for s in D]), sd([s["cam"][0] for s in D])),
                       "turn_deg": (mean([s["cam"][2] for s in D]), sd([s["cam"][2] for s in D]))}
    gain = float(next((a.split("=")[1] for a in sys.argv if a.startswith("--gain=")), 1.0))
    if gain != 1.0:  # damped update: move only part of the way (noisy residuals)
        new = {k: model[k] + gain * (new[k] - model[k]) for k in KEYS}
        print(f"阻尼: 只修正 {gain:.0%}")
    print("每个方向的残差（均值, 标准差）:")
    for k, v in rep.items():
        print(f"  {k:5s}", {kk: (round(a, 4), round(b, 4)) for kk, (a, b) in v.items()})
    print("旧参数:", {k: round(model[k], 4) for k in KEYS})
    print("新参数:", {k: round(new[k], 4) for k in KEYS})
    if "--apply" in sys.argv:
        from bench import Robot
        r = Robot()
        ack = r.cmd("slip", **{k: new[k] for k in KEYS})
        print("写入:", ack.get("ok"), ack.get("error"))
        print("小车现在:", r.status()["slip"])
    json.dump(new, open(path.replace(".json", "-refit.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
