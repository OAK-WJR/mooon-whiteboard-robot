#!/usr/bin/env python3
"""Cut a presentation video from a clean_dirty.py run (system python3: OpenCV, PIL, ffmpeg).

    python3 tools/make_demo.py                         # newest logs/clean-*-session.json
    python3 tools/make_demo.py logs/clean-1028-230000-session.json --max-erase 20

clean_dirty.py records a clean full-resolution master (clean-*-raw.mp4), the wall time of every
frame, its phase events (scan, erase, fine wipe, final check) and the camera-measured track
(clean-*-session.json). This renders a 1920x1080 / 30 fps H.264 video:

    title card -> the board -> LOOK (ink found) -> PLAN (route drawn) -> ERASE (sped up, real
    track) -> later passes -> FINE WIPE -> CHECK -> before / after -> end card

Left half: the camera, cropped to the board. Right half: the robot's top-down map (the same
frame rectified with the corner-tape homography) with the ink, the route and the real track.
Output: logs/demo-<run>.mp4.
"""
import argparse
import glob
import json
import math
import os
import subprocess
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W, H_OUT, FPS = 1920, 1080, 30
PANEL = W // 2
FONT = "/System/Library/Fonts/HelveticaNeue.ttc"
BG = (36, 31, 28)            # BGR, dark slate
INK = (77, 77, 255)          # red outlines
ROUTE = (255, 190, 90)       # light blue
TRACK = (0, 150, 255)        # orange
ACCENT = (255, 200, 80)      # caption step colour (RGB for PIL)
SPEEDS = [1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48]


def font(size, bold=False):
    return ImageFont.truetype(FONT, size, index=1 if bold else 0)


# ------------------------------------------------------------------------------------- drawing
class Text:
    """Pre-rendered RGBA text strips, alpha-blended onto BGR frames."""

    def __init__(self):
        self.cache = {}

    def strip(self, key, w, h, draw_fn):
        if key not in self.cache:
            im = Image.new("RGBA", (w, h), (0, 0, 0, 0))
            draw_fn(ImageDraw.Draw(im))
            a = np.asarray(im).astype(np.float32)
            self.cache[key] = (a[:, :, 2::-1], a[:, :, 3:] / 255.0)  # RGB->BGR, alpha
        return self.cache[key]

    def blit(self, frame, key, x, y, w, h, draw_fn, fade=1.0):
        rgb, a = self.strip(key, w, h, draw_fn)
        a = a * fade
        roi = frame[y:y + h, x:x + w].astype(np.float32)
        frame[y:y + h, x:x + w] = (roi * (1 - a) + rgb * a).astype(np.uint8)


TEXT = Text()


def caption(frame, step, line, fade=1.0):
    """Lower third across the frame: step label in accent colour, one line of explanation."""
    h = 132
    roi = frame[H_OUT - h:].astype(np.float32)
    frame[H_OUT - h:] = (roi * (1 - 0.62 * fade)).astype(np.uint8)

    def draw(d):
        d.text((60, 22), step, font=font(40, True), fill=ACCENT + (255,))
        d.text((60, 76), line, font=font(32), fill=(240, 240, 240, 255))
    TEXT.blit(frame, ("cap", step, line), 0, H_OUT - h, W, h, draw, fade)


def badge(frame, text):
    def draw(d):
        d.rounded_rectangle((0, 0, 150, 64), 14, fill=(20, 20, 20, 190))
        d.text((75, 32), text, font=font(36, True), fill=(255, 255, 255, 255), anchor="mm")
    TEXT.blit(frame, ("badge", text), W - 175, 18, 151, 65, draw)  # top right, clear of the panel titles


def panel_titles(frame):
    def draw_l(d):
        d.text((24, 14), "Camera", font=font(26, True), fill=(235, 235, 235, 255))
    def draw_r(d):
        d.text((24, 14), "Robot's map (top-down)", font=font(26, True), fill=(235, 235, 235, 255))
    TEXT.blit(frame, ("pt", "l"), 0, 0, 400, 56, draw_l)
    TEXT.blit(frame, ("pt", "r"), PANEL, 0, 500, 56, draw_r)


def card(lines, fade=1.0):
    """Full-frame title / end card: list of (text, size, bold, rgb)."""
    f = np.full((H_OUT, W, 3), BG, np.uint8)
    total = sum(int(s * 1.5) for _, s, _, _ in lines)
    y0 = (H_OUT - total) // 2

    def draw(d):
        y = 0
        for text, size, bold, rgb in lines:
            d.text((W // 2, y + size // 2), text, font=font(size, bold), fill=rgb + (255,), anchor="mm")
            y += int(size * 1.5)
    TEXT.blit(f, ("card", tuple(l[0] for l in lines)), 0, y0, W, total + 10, draw)
    return (f.astype(np.float32) * fade + np.array(BG, np.float32) * (1 - fade)).astype(np.uint8)


# ------------------------------------------------------------------------------------- geometry
class Geometry:
    def __init__(self, H, surface, frame_shape):
        self.H = np.array(H, np.float64)
        self.Hi = np.linalg.inv(self.H)
        self.surface = surface
        fh, fw = frame_shape[:2]
        sw, sh = surface
        # Camera crop: the board plus a margin, at the panel's aspect.
        c = cv2.perspectiveTransform(np.array([[[0, 0], [sw, 0], [sw, sh], [0, sh]]], np.float32), self.Hi)[0]
        x0, y0 = c.min(0)
        x1, y1 = c.max(0)
        cx, cy, bw, bh = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) * 1.22, (y1 - y0) * 1.14
        aspect = PANEL / H_OUT
        if bw / bh < aspect:
            bw = bh * aspect
        else:
            bh = bw / aspect
        bw, bh = min(bw, fw), min(bh, fh)
        self.crop = (int(max(0, min(fw - bw, cx - bw / 2))), int(max(0, min(fh - bh, cy - bh / 2))), int(bw), int(bh))
        self.cs = PANEL / bw
        # Map: the board rectified at ppc px/cm, centred in the right panel above the caption.
        self.ppc = min((PANEL - 140) / sw, (H_OUT - 80 - 150) / sh)
        self.mw, self.mh = int(sw * self.ppc), int(sh * self.ppc)
        self.mx, self.my = PANEL + (PANEL - self.mw) // 2, 70
        S = np.array([[self.ppc, 0, 0], [0, -self.ppc, sh * self.ppc], [0, 0, 1]], np.float64)
        self.M = S @ self.H

    def cam_px(self, pts):
        """Board cm -> left panel px."""
        if not len(pts):
            return np.zeros((0, 2), np.int32)
        q = cv2.perspectiveTransform(np.array([pts], np.float32), self.Hi)[0]
        x0, y0, _, _ = self.crop
        return np.round((q - [x0, y0]) * self.cs).astype(np.int32)

    def map_px(self, pts):
        if not len(pts):
            return np.zeros((0, 2), np.int32)
        p = np.array(pts, np.float64)
        return np.round(np.stack([self.mx + p[:, 0] * self.ppc, self.my + (self.surface[1] - p[:, 1]) * self.ppc], 1)).astype(np.int32)

    def compose(self, raw):
        f = np.full((H_OUT, W, 3), BG, np.uint8)
        x0, y0, bw, bh = self.crop
        f[:, :PANEL] = cv2.resize(raw[y0:y0 + bh, x0:x0 + bw], (PANEL, H_OUT), interpolation=cv2.INTER_CUBIC)
        top = cv2.warpPerspective(raw, self.M, (self.mw, self.mh), flags=cv2.INTER_AREA)
        f[self.my:self.my + self.mh, self.mx:self.mx + self.mw] = top
        cv2.rectangle(f, (self.mx - 1, self.my - 1), (self.mx + self.mw, self.my + self.mh), (90, 90, 90), 1)
        return f


def polyline(f, px, color, width, closed=False, alpha=1.0):
    if len(px) < 2:
        return
    if alpha >= 1:
        cv2.polylines(f, [px.reshape(-1, 1, 2)], closed, color, width, cv2.LINE_AA)
    else:
        layer = f.copy()
        cv2.polylines(layer, [px.reshape(-1, 1, 2)], closed, color, width, cv2.LINE_AA)
        cv2.addWeighted(layer, alpha, f, 1 - alpha, 0, f)


# ---------------------------------------------------------------------------------------- source
class Source:
    """Sequential reader of the raw master by wall time (segments come in time order)."""

    def __init__(self, path, times):
        self.cap = cv2.VideoCapture(path)
        self.times = np.array(times)
        self.idx, self.frame = -1, None
        self.n = min(len(times), int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT)) or len(times))

    def at(self, t):
        want = int(np.clip(np.searchsorted(self.times, t, side="right") - 1, 0, self.n - 1))
        if want < self.idx:  # never rewinds far: reopen
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, want)
            self.idx = want - 1
        while self.idx < want:
            if not self.cap.grab():
                break
            self.idx += 1
            if self.idx == want:
                ok, f = self.cap.retrieve()
                if ok:
                    self.frame = f
        if self.frame is None:
            ok, self.frame = self.cap.read()
        return self.frame


# ------------------------------------------------------------------------------------------ main
def nice_speed(dur, budget):
    want = dur / budget
    return next((s for s in SPEEDS if s >= want), SPEEDS[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", nargs="?", help="clean-*-session.json (default: newest)")
    ap.add_argument("--max-erase", type=float, default=22.0, help="seconds of video per erase pass at most")
    ap.add_argument("--title", default="WhiteBoardBot")
    ap.add_argument("--subtitle", default="A wall-climbing robot that sees the ink and erases it")
    ap.add_argument("--credit", default="Jiarui Wu")
    ap.add_argument("--out")
    args = ap.parse_args()
    path = args.session or max(glob.glob(os.path.join(ROOT, "logs", "clean-*-session.json")), key=os.path.getmtime)
    ses = json.load(open(path))
    if not ses.get("H") or not ses.get("times"):
        raise SystemExit("This record has no camera calibration or frame times (record it with the new clean_dirty.py)")
    raw = os.path.join(os.path.dirname(path), ses["raw"])
    src = Source(raw, ses["times"])
    first = src.at(ses["times"][0])
    geo = Geometry(ses["H"], ses["surface"], first.shape)
    ev = ses["events"]
    track = np.array([q[:3] for q in ses["track"]], np.float64) if ses["track"] else np.zeros((0, 3))
    out = args.out or path.replace("clean-", "demo-").replace("-session.json", ".mp4")
    ff = subprocess.Popen(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
                           "-s", f"{W}x{H_OUT}", "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium",
                           "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    n_out = [0]

    def emit(f):
        ff.stdin.write(f.tobytes())
        n_out[0] += 1

    def track_px(t0, t1, mapping):
        if not len(track):
            return np.zeros((0, 2), np.int32)
        a, b = np.searchsorted(track[:, 0], [t0, t1])
        return mapping(track[a:b, 1:3].tolist())

    def robot_at(t):
        if not len(track):
            return None
        i = int(np.clip(np.searchsorted(track[:, 0], t) - 1, 0, len(track) - 1))
        return track[i, 1:3]

    def scene(t, outlines=(), route=(), route_frac=1.0, ink_alpha=1.0, trail=None, eraser=False):
        f = geo.compose(src.at(t))
        for poly in outlines:
            if ink_alpha > 0:
                polyline(f, geo.map_px(poly), INK, 2, True, ink_alpha)
        if len(route) > 1 and route_frac > 0:
            r = route[:max(2, int(math.ceil(len(route) * route_frac)))]
            polyline(f, geo.map_px(r), ROUTE, 2, False, 0.85)
        if trail is not None:
            polyline(f, track_px(trail, t, geo.map_px), TRACK, 4)
            polyline(f, track_px(trail, t, geo.cam_px), TRACK, 3, alpha=0.8)
        if eraser:
            p = robot_at(t)
            if p is not None:
                c = tuple(int(v) for v in geo.map_px([p.tolist()])[0])
                cv2.circle(f, c, int(1.0 * geo.ppc), (255, 255, 255), 2, cv2.LINE_AA)
        panel_titles(f)
        return f

    def hold(seconds, fn):
        n = int(seconds * FPS)
        for i in range(n):
            emit(fn(i / max(1, n - 1)))

    def span(t0, t1, speed, fn):
        n = max(1, int((t1 - t0) / speed * FPS))
        for i in range(n):
            emit(fn(t0 + i * speed / FPS))

    # 1. Title.
    title = [(args.title, 96, True, (255, 255, 255)), (args.subtitle, 40, False, (210, 210, 210))]
    hold(0.5, lambda a: card(title, a))
    hold(2.5, lambda a: card(title))
    scans = [e for e in ev if e["kind"] == "scan"]
    if not scans:
        raise SystemExit("The record has no scan")
    # 2. The board before (up to 3 s of real footage before the first look).
    t_scan = scans[0]["t"]
    span(max(ses["times"][0], t_scan - 3.0), t_scan, 1, lambda t: (lambda f: (caption(f, "The task", "Erase everything drawn on the board, by itself"), f)[1])(scene(t)))
    before_frame = src.at(t_scan).copy()
    after_frame, first_ink = before_frame, scans[0]["dirty_cm2"]
    for k, sc in enumerate(scans):
        t = sc["t"]
        outl, route = sc["outlines"], sc["route"]
        if k == 0:
            line = f"The camera finds {sc['dirty_cm2']:.0f} cm² of ink in {len(outl)} marks"
            hold(1.8, lambda a: (lambda f: (caption(f, "1 · LOOK", line), f)[1])(scene(t, outl, ink_alpha=a)))
            way = "left-right" if sc.get("sweep", "horizontal") == "horizontal" else "up-down"
            if sc.get("mode") == "strokes":
                plan_line = (f"Follow each pen stroke ({sc['strokes']}), sweep filled areas {way} ({sc['lanes']})"
                             f" · {sc['path_cm'] / 100:.1f} m")
            else:
                plan_line = (f"Sweep {way} in lanes {sc.get('pitch', 1.6):.1f} cm apart, from the bottom up"
                             f" · {sc['path_cm'] / 100:.1f} m")
            hold(2.8, lambda a: (lambda f: (caption(f, "2 · PLAN", plan_line), f)[1])(scene(t, outl, route, a)))
        elif route and len(route) > 1:
            line = f"Look again: {sc['dirty_cm2']:.0f} cm² left — plan pass {k + 1}"
            hold(2.0, lambda a: (lambda f: (caption(f, f"PASS {k + 1}", line), f)[1])(scene(t, outl, route, a)))
        es = next((e for e in ev if e["kind"] == "erase_start" and e.get("pass_") == k + 1), None)
        ee = next((e for e in ev if e["kind"] == "erase_end" and e.get("pass_") == k + 1), None)
        if es and ee:
            sp = nice_speed(ee["t"] - es["t"], args.max_erase if k == 0 else args.max_erase / 2)
            line = ("The camera corrects the robot's position about 20 times a second" if k == 0
                    else f"Pass {k + 1}: wipe what is left")

            def frame_erase(tt, outl=outl, route=route, es=es, sp=sp, line=line):
                f = scene(tt, outl, route, 1.0, 0.55, es["t"], True)
                caption(f, "3 · ERASE", line)
                badge(f, f"×{sp}")
                return f
            span(es["t"], ee["t"], sp, frame_erase)
    for fs in [e for e in ev if e["kind"] == "fine_start"]:
        fe = next((e for e in ev if e["kind"] == "fine_end" and e.get("pass_") == fs.get("pass_")), None)
        if not fe:
            continue
        sp = nice_speed(fe["t"] - fs["t"], args.max_erase)

        def frame_fine(tt, fs=fs, sp=sp):
            f = scene(tt, fs["area"], fs["route"], 1.0, 0.5, fs["t"], True)
            caption(f, "4 · FINE WIPE", "Slow, overlapping strokes over everywhere that had ink")
            badge(f, f"×{sp}")
            return f
        span(fs["t"], fe["t"], sp, frame_fine)
    fin = next((e for e in ev if e["kind"] == "final"), None)
    after_frame = src.at(ses["times"][-1]).copy()  # a run cut short still ends on its last frame
    if fin:
        after_frame = src.at(fin["t"]).copy()
        line = f"Ink found: {first_ink:.0f} cm² before, {fin['dirty_cm2']:.0f} cm² after"
        hold(2.5, lambda a: (lambda f: (caption(f, "5 · CHECK", line), f)[1])(scene(fin["t"], fin["outlines"])))
    # Before / after, camera crops side by side.
    x0, y0, bw, bh = geo.crop

    def ba(a):
        f = np.full((H_OUT, W, 3), BG, np.uint8)
        f[:, :PANEL] = cv2.resize(before_frame[y0:y0 + bh, x0:x0 + bw], (PANEL, H_OUT), interpolation=cv2.INTER_CUBIC)
        f[:, PANEL:] = cv2.resize(after_frame[y0:y0 + bh, x0:x0 + bw], (PANEL, H_OUT), interpolation=cv2.INTER_CUBIC)
        cv2.line(f, (PANEL, 0), (PANEL, H_OUT), (255, 255, 255), 3)
        for x, label in ((0, "BEFORE"), (PANEL, "AFTER")):
            TEXT.blit(f, ("ba", label), x, 30, PANEL, 80,
                      lambda d, label=label: d.text((PANEL // 2, 40), label, font=font(56, True),
                                                    fill=(255, 255, 255, 255), anchor="mm", stroke_width=3,
                                                    stroke_fill=(0, 0, 0, 200)))
        return f
    hold(3.5, ba)
    end = [(args.title, 72, True, (255, 255, 255))] + ([(args.credit, 40, False, (210, 210, 210))] if args.credit else [])
    hold(2.5, lambda a: card(end))
    hold(0.5, lambda a: card(end, 1 - a))
    ff.stdin.close()
    ff.wait()
    print(f"{out}  ({n_out[0] / FPS:.1f} s)")


if __name__ == "__main__":
    main()
