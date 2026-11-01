#!/usr/bin/env python3
"""Draw "67" with a pen fixed to the chassis (system python3 with OpenCV, camera + robot on Wi-Fi).

The pen cannot lift, so the whole drawing is ONE continuous line that starts where the pen is now.
The user's stroke order (2026-10-31): start where a 6 normally ends (the loop's closing point), go
round the loop, up the stem to the top, straight on into the 7's bar, then down the 7. Nothing is
retraced. The robot only translates, so the pen traces the robot-centre path and its mounting
offset does not matter, as long as the heading stays where it was at the start.
Every run records logs/draw67-*.mp4 (plus a full-resolution -raw.mp4) with the camera.

The robot need not be squared to the board (squaring it would turn it and draw an arc):
- A board-frame "move" v comes out as the physical move R(alpha) v, alpha = heading0 - 24 deg (the
  firmware maps board -> world with mount 90 / front 120, and the body sits at heading_cam + 66 - 120
  on the board). Verified 2026-10-31: first run measured alpha 90 +- 10 deg against 90.2 predicted.
- Live camera fixes (op fix, like run_board.py) are sent in the world frame that op origin creates,
  taken at heading0: position R(54 - heading0) (p - p0) in m, yaw heading - heading0. The firmware
  servo then holds both position and the start heading; without fixes the body twisted 8 deg in
  8 segments and stopped the first run.

    python3 tools/draw_digits.py --dry-run                       # plan + bounds only
    python3 tools/draw_digits.py                                 # draw from the pen's position now
    python3 tools/draw_digits.py --resume logs/draw67-X.json --from 9   # continue an interrupted run
"""
import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vision  # noqa: E402
from bench import Robot  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def glyph_67(h=15.0):
    """Pen path in cm, origin = bottom-left of the 6, first point = where the 6's loop closes (the pen at
    the start). Loop: ellipse centre (3.6, 3.7), radii 3.5 x 3.6, drawn from 160 deg over the top, the right
    side and the bottom back to the left side (1.2 cm chords). Stem: cubic Bezier from the loop's left side
    up to the top, ending horizontally so it runs straight into the 7's bar."""
    s = h / 15.0
    cx, cy, rx, ry = 3.6, 3.7, 3.5, 3.6
    loop = [(cx + rx * math.cos(math.radians(a)), cy + ry * math.sin(math.radians(a))) for a in range(160, -181, -20)]
    p0, p1, p2, p3 = (0.1, 4.2), (-0.2, 10.0), (1.5, 15.0), (6.2, 15.0)
    stem = []
    for i in range(11):
        t = i / 10
        stem.append(tuple((1 - t) ** 3 * a + 3 * (1 - t) ** 2 * t * b + 3 * (1 - t) * t ** 2 * c + t ** 3 * d
                          for a, b, c, d in zip(p0, p1, p2, p3)))
    seven = [(18.0, 15.0), (12.0, 0.0)]  # the 6's top runs on into the 7's bar, then the diagonal
    return [(x * s, y * s) for x, y in loop + stem + seven]


def rot(v, deg):
    a = math.radians(deg)
    return (v[0] * math.cos(a) - v[1] * math.sin(a), v[0] * math.sin(a) + v[1] * math.cos(a))


def dturn(a, b):
    return (b - a + 180) % 360 - 180


class Tracker:
    """Owns the camera: locates the robot on every frame, keeps the latest pose, streams op fix."""

    def __init__(self, cam, cfg, H, box):
        self.cam, self.cfg, self.H, self.box = cam, cfg, H, box
        self.r = Robot()  # own HTTP connection
        self.latency = cfg.get("camera_latency_ms", 80)
        self.world = None   # (x0, y0, heading0) once op origin has been sent
        self.pose = None
        self.track = []
        self.sent = self.rejected = self.errors = 0
        self.out = None
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        n = 0
        while self.running:
            try:
                img = self.cam.frame()
                shot = time.time()
                _, p = vision.locate(img, self.cfg, self.H)
            except Exception:
                continue
            if not p:
                continue
            with self.lock:
                self.pose = (shot, p["x"], p["y"], p["heading"])
                self.track.append(self.pose)
            x0, x1, y0, y1 = self.box
            if self.out is None and not (x0 - 2 <= p["x"] <= x1 + 2 and y0 - 2 <= p["y"] <= y1 + 2):
                self.r.cmd("pause")
                self.out = (p["x"], p["y"])
            n += 1
            if self.world:
                bx, by, h0 = self.world
                wx, wy = rot(((p["x"] - bx) / 100, (p["y"] - by) / 100), 54.0 - h0)
                yaw = math.radians(dturn(h0, p["heading"]))
                try:
                    ack = self.r.cmd("fix", x=wx, y=wy, yaw=yaw, age=int((time.time() - shot) * 1000 + self.latency))
                    self.sent += 1
                    self.rejected += not ack.get("ok")
                except Exception:
                    self.errors += 1

    def latest(self, newer_than=0.0, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if self.pose and self.pose[0] > newer_than:
                    return self.pose
            time.sleep(0.03)
        raise RuntimeError("看不到小车")

    def mean(self, n=5):
        """Average of the next n fresh poses."""
        got, t = [], time.time()
        while len(got) < n:
            p = self.latest(newer_than=t)
            got.append(p)
            t = p[0]
        return (sum(p[1] for p in got) / n, sum(p[2] for p in got) / n,
                got[-1][3] + sum(dturn(got[-1][3], p[3]) for p in got) / n)

    def close(self):
        self.running = False
        self.thread.join(timeout=2)


def wait_still(r, trk, timeout=60):
    end = time.time() + timeout
    time.sleep(0.3)
    while time.time() < end:
        if trk.out:
            raise SystemExit(f"车中心 ({trk.out[0]:.1f}, {trk.out[1]:.1f}) 出了安全范围，已暂停")
        s = r.status()
        if s["src"] == "none" and not s["moving"]:
            time.sleep(0.3)
            return s
        time.sleep(0.08)
    r.cmd("pause")
    raise SystemExit("移动超时，已停下")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, default=15.0, help="digit height, cm")
    ap.add_argument("--margin", type=float, default=11.0, help="robot centre to frame, cm")
    ap.add_argument("--speed", type=float, default=0.03, help="m/s (slip model was fitted at 3 cm/s)")
    ap.add_argument("--accel", type=float, default=0.08, help="m/s^2")
    ap.add_argument("--tol", type=float, default=0.3, help="correct a waypoint missed by more than this, cm")
    ap.add_argument("--resume", help="draw67-*.json of an interrupted run: reuse its start point and heading")
    ap.add_argument("--from", dest="first", type=int, default=1, help="first target index to drive to")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    cfg = vision.config()
    W, Hh = cfg.get("surface_cm", cfg["board_cm"])
    box = (args.margin, W - args.margin, args.margin, Hh - args.margin)
    from clean_dirty import ClipRecorder
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    stamp = time.strftime("%m%d-%H%M%S")
    video = os.path.join(ROOT, "logs", f"draw67-{stamp}.mp4")
    cam = vision.Camera(cfg["camera"])
    if not args.dry_run:
        cam = ClipRecorder(cam, video)  # same .frame()/.cfg/.cap interface; writes the video
    r = Robot()
    if not r.status().get("beacon"):
        r.cmd("beacon", on=True)
        time.sleep(1.0)
    for _ in range(8):
        cam.frame()
    H, here = vision.measure(cam, n=7)
    if not args.dry_run:
        cam.H, cam.text = H, "Drawing 67"
    if args.resume:
        old = json.load(open(args.resume))
        start, heading0 = tuple(old["start"]), old["heading0"]
    else:
        start, heading0 = (here["x"], here["y"]), here["heading"]
    pen = glyph_67(args.height)
    targets = [(start[0] + x - pen[0][0], start[1] + y - pen[0][1]) for x, y in pen]
    xs, ys = [p[0] for p in targets], [p[1] for p in targets]
    print(f"现在车中心 ({here['x']:.1f}, {here['y']:.1f})，航向 {here['heading']:.1f}°（起点航向 {heading0:.1f}°）；"
          f"路线 x {min(xs):.1f}..{max(xs):.1f}, y {min(ys):.1f}..{max(ys):.1f}；安全框 x {box[0]:.1f}..{box[1]:.1f}, y {box[2]:.1f}..{box[3]:.1f}")
    if min(xs) < box[0] or max(xs) > box[1] or min(ys) < box[2] or max(ys) > box[3]:
        raise SystemExit("路线超出安全框：把小车挪开一点再试")
    todo = list(range(args.first, len(targets)))
    print(f"要画第 {todo[0]}..{todo[-1]} 个点，共 {len(targets) - 1} 段")
    if args.dry_run:
        cam.close()
        return
    if abs(dturn(heading0, here["heading"])) > 30:
        raise SystemExit("车头比起点偏了 30° 以上：先手动摆回来")

    trk = Tracker(cam, cfg, H, box)
    log = {"start": start, "heading0": heading0, "steps": [], "resume": args.resume, "from": args.first}
    t0 = time.time()
    alpha = heading0 - 24.0
    try:
        s = r.status()
        if s["mode"] != "active":
            r.cmd("enable")
            time.sleep(0.5)
        r.cmd("abort")
        wait_still(r, trk)
        x, y, hd = trk.mean(5)
        for _ in range(2):  # the fixes only accept 10 deg at a time: turn back to the start heading first
            off = dturn(heading0, hd)
            if abs(off) < 3:
                break
            print(f"车头偏了 {off:+.1f}°，先转回起点航向（笔会画一小段弧）")
            r.cmd("turn", deg=-off)
            wait_still(r, trk)
            time.sleep(0.4)
            x, y, hd = trk.mean(5)
        ack = r.cmd("origin")
        if not ack.get("ok"):
            raise SystemExit(f"origin 被拒绝: {ack.get('error')}")
        trk.world = (x, y, heading0)  # world frame of the firmware, taken at the START heading
        # Let the fixes pull the heading back to the start heading before drawing on.
        for _ in range(40):
            time.sleep(0.25)
            _, _, hd = trk.mean(3)
            if abs(dturn(heading0, hd)) < 1.0:
                break
        print(f"相机校正已开 ({trk.sent} 次，拒绝 {trk.rejected})，车头 {dturn(heading0, hd):+.1f}°")

        def move_to(tgt):
            px, py, hd = trk.mean(4)
            u = (tgt[0] - px, tgt[1] - py)
            d = math.hypot(*u)
            if d < 0.2:
                return (px, py, hd)
            v = rot(u, -alpha)
            ack = r.cmd("move", x=v[0] / 100, y=v[1] / 100, speed=args.speed, accel=args.accel)
            if not ack.get("ok"):
                r.cmd("pause")
                raise SystemExit(f"move 被拒绝: {ack.get('error')}")
            wait_still(r, trk, timeout=15 + d / (100 * args.speed) * 3)
            time.sleep(0.4)  # let the last fixes settle
            nx, ny, nh = trk.mean(5)
            turn = dturn(heading0, nh)
            log["steps"].append({"tgt": tgt, "cmd": v, "meas": (nx, ny), "heading": nh, "t": time.time() - t0,
                                 "fixes": trk.sent, "rejected": trk.rejected})
            print(f"    目标 ({tgt[0]:.1f}, {tgt[1]:.1f}) 实到 ({nx:.1f}, {ny:.1f})  差 {math.dist(tgt, (nx, ny)):.2f} cm  车头 {turn:+.1f}°")
            if abs(turn) > 8:
                r.cmd("pause")
                raise SystemExit(f"车头转了 {turn:+.1f}°，已停下")
            if trk.errors > 5:
                r.cmd("pause")
                raise SystemExit("相机校正发送出错，已停下")
            return (nx, ny, nh)

        for k in todo:
            tgt = targets[k]
            print(f"{k}/{len(targets) - 1}")
            cam.text = f"Drawing 67   point {k}/{len(targets) - 1}   {time.time() - t0:4.0f} s"
            for _ in range(3):
                nx, ny, _ = move_to(tgt)
                if math.dist(tgt, (nx, ny)) <= args.tol:
                    break
    finally:
        try:
            r.cmd("abort")
        except Exception:
            pass
        trk_ = locals().get("trk")
        if trk_:
            log["fixes"], log["rejected"] = trk_.sent, trk_.rejected
            log["track"] = trk_.track[-5000:]
            trk_.world = None
            trk_.close()
        os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
        path = os.path.join(ROOT, "logs", f"draw67-{stamp}.json")
        try:
            cam.text = "Done" if not sys.exc_info()[0] else "Stopped"
            time.sleep(1.5)
            cam.close()
            print("视频:", video)
        except Exception as err:
            print("关闭录像出错:", err)
        json.dump(log, open(path, "w"), indent=1)
        print("记录:", path)
    print(f"画完了，用时 {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
