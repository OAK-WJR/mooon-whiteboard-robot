#!/usr/bin/env python3
"""Camera ground truth for the whiteboard robot (iPhone Continuity Camera via OpenCV).

The four red tape squares on the board corners give a homography to board
centimetres: the tapes' outer corners sit on the writable-surface corners
(origin bottom-left of the white area, X right, Y up; sizes in tools/vision.json).
Reported positions are the robot CENTRE: the LCD parallax (height above the board)
and the LCD-to-centre offset are compensated when configured. The robot LCD shows a
cyan/magenta target (op "beacon"); its midpoint is the robot position and the
cyan->magenta direction its heading.

    python3 tools/vision.py snap            # save annotated frame to logs/vision-*.jpg
    python3 tools/vision.py where           # print robot X, Y (cm), heading (deg)

Run with the system python3 (has OpenCV 5); the robot API helpers live in bench.py.
"""
import json
import math
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, "tools", "vision.json")
DEFAULT = {"camera": 0, "width": 1920, "height": 1440, "tape_dx_cm": 38.0, "tape_dy_cm": 58.0}


def config():
    c = dict(DEFAULT)
    if os.path.exists(CFG):
        c.update(json.load(open(CFG)))
    return c


class Camera:
    def __init__(self, cfg=None):
        self.cfg = cfg or config()
        self.cap = cv2.VideoCapture(self.cfg["camera"], cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            raise SystemExit("Cannot open the camera")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg["width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["height"])
        # Continuity Camera switches format a few frames after the request (4:3 1920x1440
        # shows more of the board than the default 16:9); wait for it.
        for _ in range(60):
            ok, f = self.cap.read()
            if ok and f.shape[1] == self.cfg["width"] and f.shape[0] == self.cfg["height"]:
                break
        for _ in range(4):
            self.cap.read()

    def frame(self):
        # Continuity Camera drops out now and then: reopen the device instead of failing.
        for attempt in range(5):
            for _ in range(2):  # drop a buffered frame
                self.cap.grab()
            ok, f = self.cap.read()
            if ok:
                return f
            self.cap.release()
            time.sleep(1.5)
            self.cap = cv2.VideoCapture(self.cfg["camera"], cv2.CAP_AVFOUNDATION)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg["width"])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["height"])
            for _ in range(30):
                ok, f = self.cap.read()
                if ok and f.shape[1] == self.cfg["width"]:
                    break
        raise RuntimeError("camera read failed")

    def close(self):
        self.cap.release()


def _blobs(mask, min_area, max_area):
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if min_area <= area <= max_area:
            out.append({"c": cents[i], "area": int(area), "box": (int(x), int(y), int(w), int(h)),
                        "fill": area / float(w * h), "aspect": max(w, h) / max(1, min(w, h))})
    return out


def find_tapes(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 120, 70), (7, 255, 255)) | cv2.inRange(hsv, (172, 120, 70), (180, 255, 255))
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cands = [b for b in _blobs(red, 60, 6000) if b["fill"] > 0.8 and b["aspect"] < 1.8]
    if len(cands) > 4:  # the four tapes have nearly the same size
        med = float(np.median([b["area"] for b in cands]))
        cands = [b for b in cands if 0.6 * med < b["area"] < 1.6 * med]
    if len(cands) < 4:
        return None
    pts = np.array([b["c"] for b in cands])
    # The four tapes are the candidates closest to the four extreme corners of their hull.
    s, d = pts.sum(1), pts[:, 0] - pts[:, 1]
    tl, br = pts[np.argmin(s)], pts[np.argmax(s)]
    tr, bl = pts[np.argmax(d)], pts[np.argmin(d)]
    return np.array([bl, br, tr, tl], dtype=np.float32)


def find_tape_corners(img):
    """Outer corner of each tape (the one touching the writable-surface corner): BL, BR, TR, TL."""
    centres = find_tapes(img)
    if centres is None:
        return None
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 120, 70), (7, 255, 255)) | cv2.inRange(hsv, (172, 120, 70), (180, 255, 255))
    mid = centres.mean(0)
    out = []
    for c in centres:
        x, y = int(c[0]), int(c[1])
        win = red[max(0, y - 40):y + 40, max(0, x - 40):x + 40]
        ys, xs = np.nonzero(win)
        pts = np.stack([xs + max(0, x - 40), ys + max(0, y - 40)], 1).astype(np.float32)
        # Take the tape pixels deepest in the direction away from the board centre.
        d = c - mid
        d /= np.linalg.norm(d)
        proj = pts @ d
        far = pts[proj >= np.percentile(proj, 99)]
        out.append(far.mean(0) + 0.5 * d)  # half a pixel to the outer pixel edge
    return np.array(out, dtype=np.float32)


def homography(tapes, cfg):
    if "tapes_cm" in cfg:  # measured tape positions (they need not form a perfect rectangle)
        board = np.array(cfg["tapes_cm"], dtype=np.float32)
    else:
        dx, dy = cfg["tape_dx_cm"], cfg["tape_dy_cm"]
        board = np.array([[0, 0], [dx, 0], [dx, dy], [0, dy]], dtype=np.float32)
    return cv2.getPerspectiveTransform(tapes, board)


def find_beacon(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # The camera renders the LCD cyan at hue ~91-97 and its magenta as violet (~125-133).
    # In bright daylight the violet half over-exposes to a pale tint (S 35-85, V ~255),
    # still far above the board's own saturation (5-10).
    cyan = cv2.inRange(hsv, (84, 90, 140), (108, 255, 255))
    mag = cv2.inRange(hsv, (113, 30, 170), (165, 255, 255))
    cb = sorted(_blobs(cyan, 40, 20000), key=lambda b: -b["area"])
    mb = sorted(_blobs(mag, 40, 20000), key=lambda b: -b["area"])
    best = None
    for c in cb[:5]:
        for m in mb[:5]:
            dist = np.hypot(*(c["c"] - m["c"]))
            ratio = c["area"] / m["area"]
            if dist < 6 * math.sqrt(c["area"] + m["area"]) and 0.2 < ratio < 5:
                score = c["area"] + m["area"]
                if not best or score > best[0]:
                    best = (score, c, m)
    if not best:
        return None
    return best[1]["c"], best[2]["c"]


def to_board(H, p):
    q = cv2.perspectiveTransform(np.array([[p]], dtype=np.float32), H)[0, 0]
    return float(q[0]), float(q[1])


CACHE = os.path.join(ROOT, "tools", "vision_cache.json")


def _red_centres(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    red = cv2.inRange(hsv, (0, 120, 70), (7, 255, 255)) | cv2.inRange(hsv, (172, 120, 70), (180, 255, 255))
    return [b["c"] for b in _blobs(red, 60, 6000) if b["fill"] > 0.8 and b["aspect"] < 1.8]


def cached_homography(img):
    """Last full calibration, reused while the robot hides a tape, if >=2 visible tapes still match."""
    if not os.path.exists(CACHE):
        return None
    c = json.load(open(CACHE))
    seen = _red_centres(img)
    match = sum(1 for t in c["tape_px"] if any(np.hypot(*(np.array(t) - p)) < 6 for p in seen))
    return np.array(c["H"], dtype=np.float64) if match >= 2 else None


def surface_homography(img, cfg):
    """Image -> writable-surface cm (origin bottom-left corner of the white area, X right, Y up)."""
    if "surface_cm" in cfg:
        corners = find_tape_corners(img)
        if corners is None:
            return cached_homography(img)
        w, h = cfg["surface_cm"]
        H = cv2.getPerspectiveTransform(corners, np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32))
        json.dump({"H": H.tolist(), "tape_px": find_tapes(img).tolist()}, open(CACHE, "w"))
        return H
    tapes = find_tapes(img)
    return None if tapes is None else homography(tapes, cfg)


def locate(img, cfg, H=None):
    if H is None:
        H = surface_homography(img, cfg)
        if H is None:
            return None, None
    b = find_beacon(img)
    if b is None:
        return H, None
    c, m = to_board(H, b[0]), to_board(H, b[1])
    x, y = (c[0] + m[0]) / 2, (c[1] + m[1]) / 2
    heading = math.degrees(math.atan2(m[1] - c[1], m[0] - c[0]))
    # The LCD sits above the board, closer to the camera: its image is magnified by
    # D/(D-h) about the camera's foot point. Undo that, then step to the robot centre.
    if cfg.get("camera_distance_cm") and cfg.get("beacon_height_cm"):
        D, h = cfg["camera_distance_cm"], cfg["beacon_height_cm"]
        hi, wi = img.shape[:2]
        fx, fy = to_board(H, (wi / 2, hi / 2))
        k = (D - h) / D
        x, y = fx + (x - fx) * k, fy + (y - fy) * k
    off = cfg.get("beacon_to_centre_cm")
    if off:
        a = math.radians(heading)
        x += math.cos(a) * off[0] - math.sin(a) * off[1]
        y += math.sin(a) * off[0] + math.cos(a) * off[1]
    return H, {"x": x, "y": y, "heading": heading, "sep": math.hypot(m[0] - c[0], m[1] - c[1])}


def measure(cam, n=8, H=None, tries=3):
    """Median robot pose over n frames, retried while someone blocks the view."""
    for attempt in range(tries):
        try:
            return _measure(cam, n, H)
        except RuntimeError as err:
            if attempt == tries - 1:
                raise
            print(f"  ({err}; retrying in 2 s)")
            time.sleep(2)


def _measure(cam, n=8, H=None):
    """Median robot pose over n frames. Returns (H, pose dict) or raises."""
    cfg = cam.cfg
    xs, ys, hs = [], [], []
    for _ in range(n):
        img = cam.frame()
        H2, p = locate(img, cfg, H)
        H = H if H is not None else H2
        if p:
            xs.append(p["x"]); ys.append(p["y"]); hs.append(p["heading"])
    if len(xs) < max(2, n // 2):
        raise RuntimeError(f"Found the robot in only {len(xs)}/{n} frames")
    ref = hs[0]
    hs = [ref + ((h - ref + 180) % 360 - 180) for h in hs]
    return H, {"x": float(np.median(xs)), "y": float(np.median(ys)), "heading": float(np.median(hs)),
               "spread_cm": float(max(np.ptp(xs), np.ptp(ys))), "frames": len(xs)}


def annotate(img, cfg):
    tapes = find_tapes(img)
    out = img.copy()
    if tapes is not None:
        for i, p in enumerate(tapes):
            cv2.circle(out, tuple(int(v) for v in p), 12, (0, 255, 0), 3)
            cv2.putText(out, ["BL", "BR", "TR", "TL"][i], (int(p[0]) + 14, int(p[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    b = find_beacon(img)
    if b:
        cv2.line(out, tuple(int(v) for v in b[0]), tuple(int(v) for v in b[1]), (0, 0, 255), 3)
        cv2.circle(out, tuple(int(v) for v in b[0]), 8, (255, 255, 0), -1)
    H, p = locate(img, cfg)
    if p:
        cv2.putText(out, f"X {p['x']:.1f} Y {p['y']:.1f} cm  {p['heading']:.1f} deg", (40, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 255), 3)
    return out, tapes, b, p


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "where"
    cfg = config()
    if cmd == "file":
        img = cv2.imread(sys.argv[2])
    else:
        cam = Camera(cfg)
        img = cam.frame()
    out, tapes, b, p = annotate(img, cfg)
    print("tapes:", None if tapes is None else tapes.round(1).tolist())
    print("beacon:", None if b is None else [np.round(v, 1).tolist() for v in b])
    print("robot:", p)
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    path = os.path.join(ROOT, "logs", time.strftime("vision-%m%d-%H%M%S.jpg"))
    cv2.imwrite(path, out)
    print(path)


if __name__ == "__main__":
    main()
