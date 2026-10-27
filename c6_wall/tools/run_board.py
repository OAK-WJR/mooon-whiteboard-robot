#!/usr/bin/env python3
"""Cover the whole board once with the camera as referee (system python3).

    python3 tools/run_board.py               # lanes 6 cm apart, 8 cm/s
    python3 tools/run_board.py --pitch 5 --speed 0.06

1. Camera-guided drive to the bottom-left start, square up, set the origin.
2. Start the robot's own boustrophedon plan (it returns to the origin by itself).
3. Track the true pose; pause the robot if its centre leaves the safe box.
4. Report the return-to-origin error and save the true trajectory drawn on the board.
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
import vision  # noqa: E402
from bench import Robot  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Robot outline from the CAD, front up: wheels reach 6.1 cm left/right, 6.0 cm up, 4.6 cm down.
REACH = {"left": 6.1, "right": 6.1, "up": 6.0, "down": 4.6}


class Recorder:
    """Owns the camera: one thread reads every frame, writes the overlay video and
    hands the newest frame to the measuring code (same .frame()/.cfg as vision.Camera)."""

    def __init__(self, cam, path, scale=0.6667):
        import threading
        self.cam, self.cfg, self.scale = cam, cam.cfg, scale
        self.lock = threading.Lock()
        self.latest, self.seq, self.running, self.stamp = None, 0, True, 0.0
        self.track, self.box, self.H, self.text = [], None, None, ""
        self.writer, self.path = None, path
        self.frames, self.t_first = 0, None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _px(self, x, y):
        q = cv2.perspectiveTransform(np.array([[[x, y]]], dtype=np.float32), np.linalg.inv(self.H))[0, 0]
        return int(q[0] * self.scale), int(q[1] * self.scale)

    def _run(self):
        while self.running:
            ok, f = self.cam.cap.read()
            if not ok:
                continue
            with self.lock:
                self.latest, self.seq, self.stamp = f, self.seq + 1, time.time()
                track, box, H, text = list(self.track), self.box, self.H, self.text
            out = cv2.resize(f, None, fx=self.scale, fy=self.scale)
            if H is not None:
                if box:
                    cv2.rectangle(out, self._px(box["x0"], box["y1"]), self._px(box["x1"], box["y0"]), (255, 160, 0), 2)
                pts = [self._px(x, y) for _, x, y, _ in track]
                for a, b in zip(pts, pts[1:]):
                    cv2.line(out, a, b, (0, 0, 255), 2)
            cv2.putText(out, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (20, 20, 220), 2)
            if self.writer is None:
                h, w = out.shape[:2]
                for code in ("avc1", "mp4v"):
                    self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*code), 30, (w, h))
                    if self.writer.isOpened():
                        break
            self.writer.write(out)
            self.frames += 1
            self.t_first = self.t_first or time.time()

    def frame(self, with_time=False):
        start = self.seq
        while self.seq == start:
            time.sleep(0.005)
        with self.lock:
            return (self.latest.copy(), self.stamp) if with_time else self.latest.copy()

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        if self.writer:
            self.writer.release()
        self.cam.close()
        # The phone delivers ~21-24 fps, the file was written at 30: restore real-time speed.
        if self.frames > 30 and self.t_first:
            fps = self.frames / (time.time() - self.t_first)
            tmp = self.path.replace(".mp4", ".tmp.mp4")
            if os.system(f'ffmpeg -hide_banner -loglevel error -y -itsscale {30 / fps:.4f} -i "{self.path}" -c copy "{tmp}"') == 0:
                os.replace(tmp, self.path)


def wait_still(r, timeout=30):
    end = time.time() + timeout
    time.sleep(0.3)
    while time.time() < end:
        s = r.status()
        if s["src"] == "none" and not s["moving"]:
            time.sleep(0.5)
            return s
        time.sleep(0.1)
    raise SystemExit("移动超时")


def goto(r, cam, x, y, tol=0.6):
    for _ in range(4):
        _, v = vision.measure(cam, n=7)
        dx, dy = x - v["x"], y - v["y"]
        print(f"  在 ({v['x']:.1f}, {v['y']:.1f})，目标 ({x:.1f}, {y:.1f})")
        if math.hypot(dx, dy) < tol:
            return
        r.cmd("move", x=dx / 100, y=dy / 100, speed=0.06)
        wait_still(r)


def square_up(r, cam):
    front = cam.cfg.get("front_from_beacon_deg", 66.0)
    for _ in range(3):
        _, v = vision.measure(cam, n=7)
        turn = (90 - (v["heading"] + front) + 180) % 360 - 180
        if abs(turn) < 0.5:
            return
        r.cmd("turn", deg=turn)
        wait_still(r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--margin", type=float, default=2.0, help="extra clearance to the frame, cm")
    ap.add_argument("--pitch", type=float, default=6.0, help="lane spacing, cm")
    ap.add_argument("--speed", type=float, default=0.08, help="m/s")
    ap.add_argument("--learn", action="store_true", help="apply the slip update fitted from this run (needs --no-fix)")
    ap.add_argument("--no-fix", action="store_true", help="open loop: do not send camera corrections")
    args = ap.parse_args()

    r = Robot()
    if r.status()["mode"] != "active":
        r.cmd("enable")
    r.cmd("beacon", on=True)
    time.sleep(0.5)
    stamp = time.strftime("%m%d-%H%M%S")
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    video = os.path.join(ROOT, "logs", f"board-{stamp}.mp4")
    cam = Recorder(vision.Camera(), video)
    W, Hs = cam.cfg["surface_cm"]
    hard = {"x0": REACH["left"], "x1": W - REACH["right"], "y0": REACH["down"], "y1": Hs - REACH["up"]}
    soft = {"x0": hard["x0"] + args.margin, "x1": hard["x1"] - args.margin,
            "y0": hard["y0"] + args.margin, "y1": hard["y1"] - args.margin}
    width, height = soft["x1"] - soft["x0"], soft["y1"] - soft["y0"]
    print(f"白板 {W:.1f} x {Hs:.1f} cm；车中心可走 {width:.1f} x {height:.1f} cm，起点 ({soft['x0']:.1f}, {soft['y0']:.1f})")

    r.cmd("abort")  # a previous run may have left a paused path behind
    cam.text = "WhiteBoardBot: driving to the start corner"
    print("1. 开到左下角起点")
    goto(r, cam, soft["x0"], soft["y0"])
    square_up(r, cam)
    H, start = vision.measure(cam, n=9)
    r.cmd("origin")
    cam.H, cam.box = H, soft
    st0 = r.status()
    slip_used = st0["slip"]
    # World frame of the robot after "origin": centred on the start, x axis at board
    # angle (mount - front); yaw 0 = front pointing straight up.
    theta0 = math.radians(st0["mount"] - st0.get("front", 0))
    front_off = cam.cfg.get("front_from_beacon_deg", 66.0)
    latency = cam.cfg.get("camera_latency_ms", 80)
    fixed = 0

    def to_world(p):
        dx, dy = (p["x"] - start["x"]) / 100, (p["y"] - start["y"]) / 100
        c, s_ = math.cos(-theta0), math.sin(-theta0)
        return c * dx - s_ * dy, s_ * dx + c * dy, math.radians(((p["heading"] + front_off - 90) + 180) % 360 - 180)
    eraser = 8.0
    overlap = max(0.0, min(0.8, 1 - args.pitch / eraser))
    ack = r.cmd("cover", width=width / 100, height=height / 100, eraser=eraser / 100, overlap=overlap,
                speed=args.speed, vertical=0, sx=1, sy=1, mount=90)
    if not ack.get("ok"):
        raise SystemExit(f"擦板设置被拒绝: {ack.get('error')}")

    print("2. 开始整板")
    ack = r.cmd("start")
    if not ack.get("ok"):
        raise SystemExit(f"开始失败: {ack.get('error')}")
    track, t0, done_since, paused_out = [], time.time(), None, False
    while True:
        img, shot = cam.frame(with_time=True)
        _, p = vision.locate(img, cam.cfg, H)
        if p and not args.no_fix and len(track) % 2 == 0:
            wx, wy, wyaw = to_world(p)
            ack = r.cmd("fix", x=wx, y=wy, yaw=wyaw, age=int((time.time() - shot) * 1000 + latency))
            fixed += bool(ack.get("ok"))
        s = r.status()
        if p:
            track.append((time.time() - t0, p["x"], p["y"], p["heading"], s["path"]["i"] if s["src"] == "path" else -1))
            cam.track = [(a, b, c, d) for a, b, c, d, _ in track]
            cam.text = (f"Coverage run  {100 * s['path']['done'] / max(0.001, s['path']['total']):3.0f}%   "
                        f"X {p['x']:5.1f}  Y {p['y']:5.1f} cm   {time.time() - t0:4.0f} s")
            out = not (hard["x0"] < p["x"] < hard["x1"] and hard["y0"] < p["y"] < hard["y1"])
            if out and not paused_out:
                r.cmd("pause")
                paused_out = True
                print(f"  ! 车中心 ({p['x']:.1f}, {p['y']:.1f}) 越出安全范围，已暂停")
        if s["mode"] != "active":
            print("  ! 小车不在保持状态:", s["mode"], s["fault"])
            break
        if paused_out and s["path"]["paused"]:
            break
        if s["src"] != "path":
            done_since = done_since or time.time()
            if time.time() - done_since > 1.0:
                break
        if len(track) % 40 == 0 and track:
            print(f"  {track[-1][0]:5.0f} s  进度 {100 * s['path']['done'] / max(0.001, s['path']['total']):3.0f}%  "
                  f"位置 ({track[-1][1]:.1f}, {track[-1][2]:.1f})")

    time.sleep(1.0)
    _, end = vision.measure(cam, n=9, H=H)
    err = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
    turn = (end["heading"] - start["heading"] + 180) % 360 - 180
    print(f"3. 结束：回原点误差 {err:.1f} cm（X {end['x'] - start['x']:+.1f}, Y {end['y'] - start['y']:+.1f}），车头 {turn:+.1f}°，"
          f"用时 {time.time() - t0:.0f} s")

    cam.text = f"Back at start: error {err:.1f} cm, heading {turn:+.1f} deg"
    time.sleep(2.0)
    img = cam.frame()
    cam.close()
    Hinv = np.linalg.inv(H)

    def px(x, y):
        q = cv2.perspectiveTransform(np.array([[[x, y]]], dtype=np.float32), Hinv)[0, 0]
        return int(q[0]), int(q[1])
    cv2.rectangle(img, px(soft["x0"], soft["y1"]), px(soft["x1"], soft["y0"]), (255, 160, 0), 2)
    pts = [px(x, y) for _, x, y, _, _ in track]
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (0, 0, 255), 2)
    cv2.circle(img, px(start["x"], start["y"]), 8, (0, 200, 0), -1)
    cv2.imwrite(os.path.join(ROOT, "logs", f"board-{stamp}.jpg"), img)

    report = analyse(track, start, width, height, args.pitch, slip_used)
    report.update(home_cm=err, home_dx=end["x"] - start["x"], home_dy=end["y"] - start["y"], home_turn_deg=turn,
                  seconds=time.time() - t0, slip=slip_used, stamp=stamp)
    json.dump({"start": start, "end": end, "track": track, "soft": soft, "hard": hard, "report": report},
              open(os.path.join(ROOT, "logs", f"board-{stamp}.json"), "w"))
    with open(os.path.join(ROOT, "logs", "runs.jsonl"), "a") as f:
        f.write(json.dumps({k: v for k, v in report.items() if k != "segments"}) + "\n")
    print("录像:", video)
    print("轨迹图:", os.path.join(ROOT, "logs", f"board-{stamp}.jpg"))
    complete = not paused_out and report["segments"] and s["src"] != "path"
    print(f"相机校正: 发送并采用 {fixed} 次" if not args.no_fix else "相机校正: 关闭（开环）")
    if args.learn and not args.no_fix:
        print("开着相机校正时偏差已被实时消掉，不能用来学习补偿；要学习请加 --no-fix")
    elif args.learn and report.get("suggest") and not complete:
        print("这一轮没有完整跑完，不用它更新补偿")
    elif args.learn and report.get("suggest"):
        ack = r.cmd("slip", **report["suggest"])
        print("已按整板轨迹更新补偿:" if ack.get("ok") else "补偿更新被拒绝:", report["suggest"], ack.get("error", ""))


def analyse(track, start, width, height, pitch, slip, gain=0.7):
    """Per-segment comparison of the true track with the plan the robot executed.

    Segment i of the robot's path ends at plan point i. Residuals are measured
    with the compensation that was active, so updates are relative to it."""
    lanes = int(math.ceil(height / pitch - 1e-4)) + 1
    spacing = height / (lanes - 1)
    plan, at = [], 0.0
    for k in range(lanes):
        b = k * spacing
        if k:
            plan.append(("step", at, b))
        at = width if k % 2 == 0 else 0.0
        plan.append(("lane", at, b))
    if at > 1e-4:
        plan.append(("back", 0.0, (lanes - 1) * spacing))
    plan.append(("down", 0.0, 0.0))
    # Actual position at the end of each segment: last sample before the index moves on.
    ends = {}
    for t, x, y, h, i in track:
        if i >= 0:
            ends.setdefault(i, []).append((x, y, h))
    segs, prev = [], (start["x"], start["y"], start["heading"])
    planned_prev = (0.0, 0.0)
    for i, (kind, px_, py_) in enumerate(plan):
        nxt = ends.get(i + 1, [None])[0] if i + 1 in ends else None
        actual = nxt if nxt else (ends.get(i, [prev])[-1])
        dxp, dyp = px_ - planned_prev[0], py_ - planned_prev[1]
        dxa, dya = actual[0] - prev[0], actual[1] - prev[1]
        dh = (actual[2] - prev[2] + 180) % 360 - 180
        segs.append({"i": i, "kind": kind, "plan": [dxp, dyp], "actual": [dxa, dya], "turn_deg": dh,
                     "end_err": [actual[0] - (start["x"] + px_), actual[1] - (start["y"] + py_)]})
        prev, planned_prev = actual, (px_, py_)
    lane_s = [g for g in segs if g["kind"] in ("lane", "back") and abs(g["plan"][0]) > 5]
    up_s = [g for g in segs if g["kind"] == "step"]
    down_s = [g for g in segs if g["kind"] == "down"]
    mean = lambda v: sum(v) / len(v) if v else 0.0
    side_r = mean([g["actual"][0] / g["plan"][0] for g in lane_s])
    sag_res = mean([-g["actual"][1] / abs(g["plan"][0]) for g in lane_s])             # cm/cm
    yaw_res = mean([math.radians(g["turn_deg"]) / (g["plan"][0] / 100) for g in lane_s])  # rad per m right
    up_r = mean([g["actual"][1] / g["plan"][1] for g in up_s])
    up_side = mean([g["actual"][0] / g["plan"][1] for g in up_s])
    down_r = mean([g["actual"][1] / g["plan"][1] for g in down_s]) if down_s else 1.0
    down_side = mean([g["actual"][0] / -g["plan"][1] for g in down_s]) if down_s else 0.0
    lerp = lambda old, new: old + gain * (new - old)
    suggest = {
        "side": lerp(slip["side"], slip["side"] * side_r),
        "sag": lerp(slip["sag"], slip["sag"] + sag_res),
        "yaw": lerp(slip["yaw"], slip["yaw"] + yaw_res),
        "up": lerp(slip["up"], slip["up"] * up_r),
        "up_side": lerp(slip.get("up_side", 0), slip.get("up_side", 0) + up_side),
        "down": lerp(slip["down"], slip["down"] * down_r),
        "down_side": lerp(slip.get("down_side", 0), slip.get("down_side", 0) + down_side),
    }
    worst = max(math.hypot(*g["end_err"]) for g in segs)
    print(f"分段: 横线 {len(lane_s)} 条 横向比 {side_r:.3f} 下沉残差 {100 * sag_res:+.2f} cm/m 转角残差 {math.degrees(yaw_res):+.1f}°/m；"
          f"换条 {len(up_s)} 次 上行比 {up_r:.3f} 右漂 {100 * up_side:+.2f} cm/m；回程下行比 {down_r:.3f}；拐点最大偏差 {worst:.1f} cm")
    return {"segments": segs, "worst_corner_cm": worst, "side_ratio": side_r, "sag_residual": sag_res,
            "yaw_residual": yaw_res, "up_ratio": up_r, "up_side": up_side, "down_ratio": down_r,
            "down_side": down_side, "suggest": suggest}


if __name__ == "__main__":
    main()
