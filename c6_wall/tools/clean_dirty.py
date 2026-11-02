#!/usr/bin/env python3
"""Find the dirty parts of the whiteboard and erase only those (system python3 with OpenCV).

The eraser is the felt under the magnet in the middle of the robot: about 3 cm across, but one
pass only cleans 2 cm (sysid_erase.py, 2026-10-28), centred on the robot centre (--eraser,
--eraser-offset). The default plan follows the pen: the eraser drives along the centre line of
every stroke (--lanes strokes). The default (--lanes sweep) sweeps every mark with left-right lanes
1.6 cm apart from the bottom up: up/down travel makes the robot jitter (--sweep, --top-down).

Modes
    python3 tools/clean_dirty.py scan                    # camera: detect + plan, draw it, do not move
    python3 tools/clean_dirty.py scan --image photo.jpg  # same, from a saved photo
    python3 tools/clean_dirty.py vision                  # camera: detect, plan, erase, look again (--passes)
    python3 tools/clean_dirty.py blind --at 15,20 --rect 20,30,40,50   # no camera: erase given rectangles
    python3 tools/clean_dirty.py blind --at 15,20 --full                # no camera: the whole reachable board

Board frame (same as vision.py): cm, origin at the bottom-left corner of the writable surface,
X right, Y up. Every robot target stays inside the safe box (--margin cm from the frame to the
robot centre, 11 cm by default, see README); dirt closer to the frame than that is reported as
unreachable.

Vision mode streams the camera pose to the firmware (op fix) the whole time, so its own servo
corrects slip while driving, and drives the route with the firmware "move" op, measuring the real
pose after every waypoint; a waypoint missed by more than --tol cm gets correction moves.
A body rotation over 1.5 degrees is squared up again. It stops (the robot holds) after three
3 cm+ steps in a row that travel under half the command, a step over 160 %, a rotation over
12 degrees, or the robot centre leaving the safe box. Results: logs/clean-*.jpg / .json / .mp4.
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

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PPC = 4.0          # pixels per cm of the rectified board image used for detection
GRID = 0.5         # cm per cell of the planning grid


# ------------------------------------------------------------------------------------ detection
def rectify(img, H, surface, ppc=PPC):
    """Top-down board image: column = X * ppc, row = (height - Y) * ppc."""
    w, h = surface
    S = np.array([[ppc, 0, 0], [0, -ppc, h * ppc], [0, 0, 1]], dtype=np.float64)
    M = S @ H
    return cv2.warpPerspective(img, M, (int(round(w * ppc)), int(round(h * ppc))), flags=cv2.INTER_AREA), M


def robot_mask(shape, pose, radius_cm, surface, ppc=PPC):
    m = np.zeros(shape[:2], np.uint8)
    if pose:
        c = (int(round(pose["x"] * ppc)), int(round((surface[1] - pose["y"]) * ppc)))
        cv2.circle(m, c, int(round(radius_cm * ppc)), 255, -1)
    return m


def find_robot_blob(top, ppc=PPC):
    """Fallback when the beacon is not visible: the robot is the largest compact dark blob
    (black wheels, PCB) of at least 25 cm^2. Returns a pose-like dict or None."""
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    dark = cv2.inRange(hsv, (0, 0, 0), (180, 255, 90))
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((int(3 * ppc), int(3 * ppc)), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(dark)
    best = None
    for i in range(1, n):
        area_cm2 = st[i, cv2.CC_STAT_AREA] / ppc ** 2
        w, h = st[i, cv2.CC_STAT_WIDTH] / ppc, st[i, cv2.CC_STAT_HEIGHT] / ppc
        if area_cm2 >= 25 and max(w, h) < 20 and (not best or area_cm2 > best[0]):
            best = (area_cm2, cen[i])
    if not best:
        return None
    hpx = top.shape[0]
    return {"x": best[1][0] / ppc, "y": (hpx - best[1][1]) / ppc, "heading": 0.0, "blob": True}


def colour_distance(top, ppc=PPC):
    """Lab distance of every pixel from the board around it (8 cm median), chroma weighted 1.6."""
    lab8 = cv2.cvtColor(top, cv2.COLOR_BGR2LAB)
    k = int(8 * ppc) | 1
    bg = np.dstack([cv2.medianBlur(lab8[:, :, i], k) for i in range(3)]).astype(np.float32)
    d = lab8.astype(np.float32) - bg
    return np.sqrt(d[:, :, 0] ** 2 + (1.6 * d[:, :, 1]) ** 2 + (1.6 * d[:, :, 2]) ** 2)


def detect_dirt(img, cfg, H, pose=None, sensitivity=1.0, robot_radius=10.0, border=1.5, min_area=0.4):
    """Returns (dirt mask on the rectified image, rectified image, info).

    Ink is either clearly coloured (marker) or clearly darker than the board around it. The board
    background is estimated with a large median filter, so uneven light and the frame shadow do not
    count. Faint ghosting stays below the default thresholds; raise --sensitivity to include it."""
    surface = cfg["surface_cm"]
    top, M = rectify(img, H, surface)
    # Colour distance (Lab, chroma weighted up) from the board around each pixel. Thin light
    # marker lines are neither very dark nor very saturated, but they differ clearly in colour;
    # ghosting differs by 3-10, marker lines by 25-120 (photo of 2026-10-27).
    dE = colour_distance(top)
    dirt = (dE > 22.0 / sensitivity).astype(np.uint8) * 255

    # Things that are not dirt: the frame band, the corner tapes and the robot (with its shadow).
    keep = np.zeros_like(dirt)
    b = int(round(border * PPC))
    keep[b:-b, b:-b] = 255
    t = int(round(4.0 * PPC))
    for (cx, cy) in ((0, 0), (keep.shape[1], 0), (0, keep.shape[0]), (keep.shape[1], keep.shape[0])):
        cv2.rectangle(keep, (cx - t, cy - t), (cx + t, cy + t), 0, -1)
    beacon = pose is not None
    if pose is None:
        pose = find_robot_blob(top)
    keep[robot_mask(dirt.shape, pose, robot_radius, surface) > 0] = 0
    dirt &= keep

    dirt = cv2.morphologyEx(dirt, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(dirt)
    drop = []
    for i in range(1, n):
        area = st[i, cv2.CC_STAT_AREA] / PPC ** 2
        w, h = st[i, cv2.CC_STAT_WIDTH] / PPC, st[i, cv2.CC_STAT_HEIGHT] / PPC
        if area < min_area:
            drop.append(i)
        # Without the beacon, a big compact blob is taken for the robot (or a hand). With the beacon
        # the robot is already masked, and a small filled drawing looks just like that blob.
        elif not beacon and area > 25 and max(w, h) < 22 and area / (w * h) > 0.3:
            drop.append(i)
    if drop:
        dirt[np.isin(lab, drop)] = 0
    info = {"dirty_cm2": float(np.count_nonzero(dirt) / PPC ** 2), "robot": pose}
    return dirt, top, info


# ------------------------------------------------------------------------------------- planning
def to_grid(dirt_px, ppc=PPC, cell=GRID):
    """Rectified-image mask -> planning grid (row 0 = bottom, Y up)."""
    f = int(round(ppc * cell))
    h, w = dirt_px.shape
    hh, ww = h // f, w // f
    g = dirt_px[:hh * f, :ww * f].reshape(hh, f, ww, f).max(axis=(1, 3)) > 0
    return g[::-1]  # flip so that row index grows with Y


def reachable_box(surface, margin):
    w, h = surface
    return margin, w - margin, margin, h - margin


def _lanes(cells, axis, d, pitch, r_ext):
    """Boustrophedon lanes covering the given dirty cell centres (N x 2, cm).
    axis 0: lanes run along X (stacked in Y); axis 1: lanes run along Y."""
    a = cells[:, axis]          # along the lane
    c = cells[:, 1 - axis]      # across the lanes
    c0, c1 = c.min(), c.max()
    n = max(1, int(math.ceil((c1 - c0) / pitch - 1e-6)) + 1) if c1 - c0 > 1e-6 else 1
    pos = [c0] if n == 1 else list(np.linspace(c0, c1, n))
    lanes = []
    for cp in pos:
        sel = np.abs(c - cp) <= d / 2 + GRID / 2
        if not np.any(sel):
            continue
        lo, hi = a[sel].min() - r_ext, a[sel].max() + r_ext
        lanes.append((cp, lo, hi))
    return lanes


def _snake(lanes, axis, start_low_across, start_low_along):
    seq = lanes if start_low_across else lanes[::-1]
    pts, low = [], start_low_along
    for cp, lo, hi in seq:
        a0, a1 = (lo, hi) if low else (hi, lo)
        p0 = (a0, cp) if axis == 0 else (cp, a0)
        p1 = (a1, cp) if axis == 0 else (cp, a1)
        pts += [p0, p1]
        low = not low
    return pts


def _simplify(pts, tol):
    """Douglas-Peucker on a polyline (list of (x, y))."""
    if len(pts) < 3:
        return list(pts)
    a, b = np.array(pts[0]), np.array(pts[-1])
    ab = b - a
    L = np.hypot(*ab)
    P = np.array(pts[1:-1])
    if L < 1e-9:
        d = np.hypot(*(P - a).T)
    else:
        d = np.abs(ab[0] * (P[:, 1] - a[1]) - ab[1] * (P[:, 0] - a[0])) / L
    i = int(np.argmax(d))
    if d[i] <= tol:
        return [pts[0], pts[-1]]
    left = _simplify(pts[:i + 2], tol)
    return left[:-1] + _simplify(pts[i + 1:], tol)


def trace_stroke(cells, eraser, step=1.0, tol=0.3):
    """Centre line of a pen stroke, or None when one pass of the eraser would not cover it.

    The stroke is cut into slices `step` cm wide along its main direction; the middle of the dirt
    in each slice is a waypoint. Accepted only if every dirty cell lies within 85 % of the eraser
    radius of the simplified line (S-bends that fold back, blobs and filled areas fall back to lanes)."""
    if len(cells) < 3:
        c = cells.mean(0)
        return [(float(c[0]), float(c[1]))]
    mu = cells.mean(0)
    _, _, vt = np.linalg.svd(cells - mu, full_matrices=False)
    u = vt[0]
    t = (cells - mu) @ u
    order = np.floor((t - t.min()) / step).astype(int)
    pts = []
    for k in range(order.max() + 1):
        sel = order == k
        if np.any(sel):
            c = cells[sel]
            pts.append((float(c[:, 0].mean()), float(c[:, 1].mean())))
    line = _simplify(pts, tol)
    # Extend both ends a little so the eraser runs past the stroke ends.
    if len(line) >= 2:
        for i, j in ((0, 1), (-1, -2)):
            a, b = np.array(line[i]), np.array(line[j])
            d = a - b
            n = np.hypot(*d)
            if n > 1e-6:
                line[i] = tuple(a + d / n * eraser * 0.25)
    # Coverage check: distance of every dirty cell to the polyline.
    P = cells
    best = np.full(len(P), np.inf)
    for a, b in zip(line, line[1:]):
        a, b = np.array(a), np.array(b)
        ab = b - a
        L2 = max(ab @ ab, 1e-12)
        s = np.clip(((P - a) @ ab) / L2, 0, 1)
        q = a + s[:, None] * ab
        best = np.minimum(best, np.hypot(*(P - q).T))
    if len(line) == 1:
        best = np.hypot(*(P - np.array(line[0])).T)
    return line if best.max() <= 0.85 * eraser / 2 else None


def _length(pts, start=None):
    pts = ([start] if start else []) + list(pts)
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def plan(grid, surface, start, eraser=2.0, overlap=0.2, margin=11.0, merge=1.0, cell=GRID, lanes="horizontal",
         items=False, bottom_up=True):
    """Grid of dirty cells -> list of target points (cm) for the robot centre.

    Dirt is grouped into regions (cells closer than `merge` cm belong together); every region is
    swept with straight lanes `eraser * (1 - overlap)` apart and regions are visited nearest-first.
    lanes="horizontal" / "vertical": left-right / up-down lanes only. lanes="auto": a traced line along a thin stroke when that
    covers it, else lanes in whichever direction gives the shorter path. Returns (points, report)."""
    x0, x1, y0, y1 = reachable_box(surface, margin)
    r = eraser / 2
    pitch = eraser * (1 - overlap)
    ys, xs = np.nonzero(grid)
    if not len(xs):
        return ([], {}) if items else ([], {"regions": 0, "unreachable_cm2": 0.0, "covered_cm2": 0.0, "path_cm": 0.0,
                                            "pitch_cm": pitch, "safe_box": [x0, x1, y0, y1]})
    pts = np.stack([(xs + 0.5) * cell, (ys + 0.5) * cell], 1)
    # Dirt the eraser can reach: within r of the safe box of the robot centre.
    ok = (pts[:, 0] >= x0 - r) & (pts[:, 0] <= x1 + r) & (pts[:, 1] >= y0 - r) & (pts[:, 1] <= y1 + r)
    unreachable = float((~ok).sum() * cell * cell)
    g = np.zeros_like(grid, np.uint8)
    g[ys[ok], xs[ok]] = 255
    k = max(1, int(round(merge / cell)))
    joined = cv2.dilate(g, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
    n, lab = cv2.connectedComponents(joined)
    regions = []
    for i in range(1, n):
        sel = (lab == i) & (g > 0)
        ry, rx = np.nonzero(sel)
        if len(rx):
            regions.append(np.stack([(rx + 0.5) * cell, (ry + 0.5) * cell], 1))

    def clamp(p):
        return (min(max(p[0], x0), x1), min(max(p[1], y0), y1))

    # Every region: a traced line (pen strokes are much thinner than the eraser) when that covers
    # it, otherwise lanes in both directions with four entry corners each.
    options = []
    for reg in regions:
        opts = []
        line = trace_stroke(reg, eraser) if lanes == "auto" else None
        if line is not None:
            line = [clamp(q) for q in line]
            options.append([line, line[::-1]])
            continue
        for axis in {"auto": (0, 1), "horizontal": (0,), "vertical": (1,)}[lanes]:
            lane_list = _lanes(reg, axis, eraser, pitch, r * 0.5)
            # Bottom-up: left-right lanes start with the lowest one (only short up-steps in between).
            for a in ((True,) if bottom_up and axis == 0 else (True, False)):
                for b in (True, False):
                    p = [clamp(q) for q in _snake(lane_list, axis, a, b)]
                    opts.append(p)
        options.append(opts)
    if items:  # the stroke planner orders these together with its strokes
        return options, {}
    # Nearest-first tour (bottom-up: lowest region first); within a region the variant with the
    # shortest approach + sweep.
    path, here, todo = [], start, list(range(len(options)))
    while todo:
        best = None
        for i in ([min(todo, key=lambda j: regions[j][:, 1].min())] if bottom_up else todo):
            for p in options[i]:
                cost = _length(p, here)
                if not best or cost < best[0]:
                    best = (cost, i, p)
        _, i, p = best
        path += p
        here = p[-1]
        todo.remove(i)
    # Drop zero-length steps.
    out = []
    for p in path:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 0.2:
            out.append(p)
    report = {"regions": len(regions), "unreachable_cm2": unreachable, "covered_cm2": float(ok.sum() * cell * cell),
              "path_cm": _length(out, start), "pitch_cm": pitch, "safe_box": [x0, x1, y0, y1]}
    return out, report


def thin(mask):
    """Zhang-Suen thinning of a binary mask (numpy, no contrib modules)."""
    img = np.pad((mask > 0).astype(np.uint8), 1)
    while True:
        changed = False
        for step in (0, 1):
            P = img
            p2, p3, p4, p5 = P[:-2, 1:-1], P[:-2, 2:], P[1:-1, 2:], P[2:, 2:]
            p6, p7, p8, p9 = P[2:, 1:-1], P[2:, :-2], P[1:-1, :-2], P[:-2, :-2]
            ring = [p2, p3, p4, p5, p6, p7, p8, p9, p2]
            B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            A = sum(((ring[i] == 0) & (ring[i + 1] == 1)).astype(np.uint8) for i in range(8))
            m = (P[1:-1, 1:-1] == 1) & (B >= 2) & (B <= 6) & (A == 1)
            if step == 0:
                m &= ((p2 * p4 * p6) == 0) & ((p4 * p6 * p8) == 0)
            else:
                m &= ((p2 * p4 * p8) == 0) & ((p2 * p6 * p8) == 0)
            if m.any():
                img[1:-1, 1:-1][m] = 0
                changed = True
        if not changed:
            return img[1:-1, 1:-1]


def _walk(skel):
    """Skeleton pixels -> pixel paths. Each path starts at a free end and keeps going straight
    through junctions; side branches become their own paths later."""
    left = set(zip(*np.nonzero(skel)))
    steps = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]

    def free(p):
        return [(p[0] + dr, p[1] + dc) for dr, dc in steps if (p[0] + dr, p[1] + dc) in left]

    paths = []
    while left:
        ends = [p for p in left if len(free(p)) <= 1]
        cur = min(ends) if ends else min(left)
        left.discard(cur)
        path, d = [cur], None
        while True:
            cand = free(cur)
            if not cand:
                break
            if d is None:
                nxt = min(cand, key=lambda q: abs(q[0] - cur[0]) + abs(q[1] - cur[1]))
            else:
                nxt = max(cand, key=lambda q: ((q[0] - cur[0]) * d[0] + (q[1] - cur[1]) * d[1])
                          / math.hypot(q[0] - cur[0], q[1] - cur[1]))
            d = (nxt[0] - cur[0], nxt[1] - cur[1])
            left.discard(nxt)
            path.append(nxt)
            cur = nxt
        paths.append(path)
    return paths


def plan_strokes(dirt, surface, start, eraser=2.0, overlap=0.2, margin=11.0, join=0.5, fill=2.0, ppc=PPC,
                 sweep="horizontal"):
    """Follow the pen: the robot centre drives along the centre line of every stroke.

    Lines closer than 2 * `join` cm are merged first (double strokes, hatching). Merged ink wider
    than `fill` cm (a filled or densely hatched area) is swept with `sweep` lanes; the thin rest is
    thinned to centre lines, which are walked into paths. A path is dropped when the ink around it
    is already within reach of longer paths. Ink still farther than 85 % of the eraser radius from
    every path gets lanes too. Visits nearest-first."""
    x0, x1, y0, y1 = reachable_box(surface, margin)
    r = eraser / 2
    reach = 0.85 * r
    h = surface[1]
    if not np.any(dirt):
        return [], {"regions": 0, "strokes": 0, "lane_regions": 0, "unreachable_cm2": 0.0, "covered_cm2": 0.0, "path_cm": 0.0,
                    "pitch_cm": eraser * (1 - overlap), "safe_box": [x0, x1, y0, y1]}
    k = max(1, int(round(join * ppc)))
    merged = cv2.dilate(dirt, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
    kf = max(1, int(round(fill / 2 * ppc)))
    wide = cv2.morphologyEx(merged, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * kf + 1, 2 * kf + 1)))
    wide = cv2.dilate(wide, np.ones((3, 3), np.uint8))
    fill_ink = (dirt > 0) & (wide > 0)
    paths = _walk(thin(merged & ~wide))

    def to_cm(path):
        return [((c + 0.5) / ppc, h - (rr + 0.5) / ppc) for rr, c in path]

    def clamp(p):
        return (min(max(p[0], x0), x1), min(max(p[1], y0), y1))

    def footprint(line):
        m = np.zeros_like(dirt)
        px = [(int(round(p[0] * ppc)), int(round((h - p[1]) * ppc))) for p in line]
        rad = max(1, int(round(reach * ppc)))
        if len(px) == 1:
            cv2.circle(m, px[0], rad, 255, -1)
        else:
            cv2.polylines(m, [np.array(px, np.int32)], False, 255, 2 * rad, cv2.LINE_8)
            for q in (px[0], px[-1]):
                cv2.circle(m, q, rad, 255, -1)
        return m

    ink = (dirt > 0) & ~fill_ink
    covered = np.zeros_like(dirt)
    strokes = []

    def take(paths):
        nonlocal covered
        for path in sorted(paths, key=len, reverse=True):
            line = [clamp(q) for q in _simplify(to_cm(path), 0.25)]
            fp = footprint(line) > 0
            mine = ink & fp
            if not mine.any() or np.count_nonzero(mine & (covered > 0)) >= 0.95 * np.count_nonzero(mine):
                continue
            strokes.append(line)
            covered |= fp.astype(np.uint8) * 255

    take(paths)
    # Ink the merged centre lines miss (edges of hatching, a stroke next to another) gets centre
    # lines of its own, unmerged, before anything falls back to lanes.
    for _ in range(3):
        left_ink = (ink & (covered == 0)).astype(np.uint8)
        if np.count_nonzero(left_ink) / ppc ** 2 < 0.1:
            break
        take(_walk(thin(cv2.dilate(left_ink, np.ones((3, 3), np.uint8)))))
    # Reachability: ink farther than r from the safe box can never be under the eraser.
    ys, xs = np.nonzero(dirt)
    cx, cy = (xs + 0.5) / ppc, h - (ys + 0.5) / ppc
    ok = (cx >= x0 - r) & (cx <= x1 + r) & (cy >= y0 - r) & (cy <= y1 + r)
    unreachable = float((~ok).sum() / ppc ** 2)
    rest = (ink & (covered == 0)) | fill_ink
    rest[ys[~ok], xs[~ok]] = False
    options = [[s_, s_[::-1]] for s_ in strokes]
    lane_regions = 0
    if np.count_nonzero(rest) / ppc ** 2 > 0.3:
        sub, rep = plan(to_grid(rest.astype(np.uint8) * 255), surface, start, eraser, overlap, margin, merge=1.0,
                        lanes=sweep, items=True)
        options += sub
        lane_regions = len(sub)
    path, here, todo = [], start, list(range(len(options)))
    while todo:
        best = None
        for i in todo:
            for p in options[i]:
                approach = math.hypot(p[0][0] - here[0], p[0][1] - here[1])
                cost = (approach, _length(p, here))
                if not best or cost < best[0]:
                    best = (cost, i, p)
        _, i, p = best
        path += p
        here = p[-1]
        todo.remove(i)
    out = []
    for p in path:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 0.2:
            out.append(p)
    report = {"regions": len(strokes) + lane_regions, "strokes": len(strokes), "lane_regions": lane_regions,
              "unreachable_cm2": unreachable, "covered_cm2": float(ok.sum() / ppc ** 2),
              "path_cm": _length(out, start), "pitch_cm": eraser * (1 - overlap), "safe_box": [x0, x1, y0, y1],
              "strokes_cm": [s_ for s_ in strokes]}
    return out, report


def plan_rects(rects, surface, start, eraser=2.0, overlap=0.2, margin=11.0, cell=GRID):
    """Blind mode: the same planner over user rectangles (cm, x0,y0,x1,y1)."""
    w, h = surface
    grid = np.zeros((int(h / cell), int(w / cell)), bool)
    for x0, y0, x1, y1 in rects:
        grid[int(max(0, min(y0, y1)) / cell):int(min(h, max(y0, y1)) / cell),
             int(max(0, min(x0, x1)) / cell):int(min(w, max(x0, x1)) / cell)] = True
    return plan(grid, surface, start, eraser, overlap, margin, merge=0.0, cell=cell)


# -------------------------------------------------------------------------------------- drawing
def draw(top, dirt, path, start, surface, report, ppc=PPC):
    out = top.copy()
    red = np.zeros_like(out)
    red[:, :, 2] = 255
    m = dirt > 0
    out[m] = (0.45 * out[m] + 0.55 * red[m]).astype(np.uint8)
    h = surface[1]
    px = lambda p: (int(round(p[0] * ppc)), int(round((h - p[1]) * ppc)))
    x0, x1, y0, y1 = report.get("safe_box", reachable_box(surface, 11.0))
    cv2.rectangle(out, px((x0, y1)), px((x1, y0)), (255, 160, 0), 1)
    pts = [start] + list(path) if start else list(path)
    for a, b in zip(pts, pts[1:]):
        cv2.line(out, px(a), px(b), (255, 80, 0), 1, cv2.LINE_AA)
    if start:
        cv2.circle(out, px(start), 5, (0, 200, 0), -1)
    return out


def side_by_side(img, H, top, overlay):
    hi = img.shape[0]
    s = hi / top.shape[0]
    big = cv2.resize(overlay, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST)
    return np.hstack([img, big])



# ------------------------------------------------------------------------------------ recording
class ClipRecorder:
    """Camera thread that writes logs/clean-*.mp4 with the detected dirt (red outline), the planned
    route (blue), the true track (orange) and the current target drawn on the board. Same
    .frame()/.cfg/.cap interface as vision.Camera, so vision.measure() can use it."""

    def __init__(self, cam, path, scale=0.6667):
        import threading
        self.cam, self.cfg, self.scale = cam, cam.cfg, scale
        self.lock = threading.Lock()
        self.latest, self.seq, self.running, self.stamp = None, 0, True, 0.0
        self.H, self.route, self.track, self.dirt, self.target, self.box, self.text = None, [], [], [], None, None, ""
        self.writer, self.path, self.frames, self.t_first = None, path, 0, None
        # Clean full-resolution master, the wall time of every frame and the run's events, so
        # tools/make_demo.py can cut a presentation video afterwards.
        self.raw_path, self.raw, self.times, self.events, self.full_track = path.replace(".mp4", "-raw.mp4"), None, [], [], None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    @property
    def cap(self):
        return self.cam.cap

    def _px(self, H, p):
        q = cv2.perspectiveTransform(np.array([[[p[0], p[1]]]], dtype=np.float32), np.linalg.inv(H))[0, 0]
        return int(q[0] * self.scale), int(q[1] * self.scale)

    def _run(self):
        while self.running:
            ok, f = self.cam.cap.read()
            if not ok:
                continue
            with self.lock:
                self.latest, self.seq, self.stamp = f, self.seq + 1, time.time()
                H, route, track, dirt, target, box, text = (self.H, list(self.route), list(self.track), list(self.dirt),
                                                            self.target, self.box, self.text)
            if self.raw is None:
                for code in ("avc1", "mp4v"):
                    self.raw = cv2.VideoWriter(self.raw_path, cv2.VideoWriter_fourcc(*code), 30, (f.shape[1], f.shape[0]))
                    if self.raw.isOpened():
                        break
            self.raw.write(f)
            self.times.append(self.stamp)
            out = cv2.resize(f, None, fx=self.scale, fy=self.scale)
            if H is not None:
                for poly in dirt:
                    pts = np.array([self._px(H, p) for p in poly], np.int32)
                    cv2.polylines(out, [pts], True, (40, 40, 255), 2, cv2.LINE_AA)
                if box:
                    cv2.rectangle(out, self._px(H, (box[0], box[3])), self._px(H, (box[1], box[2])), (255, 160, 0), 1)
                rp = [self._px(H, p) for p in route]
                for a, b in zip(rp, rp[1:]):
                    cv2.line(out, a, b, (255, 120, 20), 2, cv2.LINE_AA)
                tp = [self._px(H, p) for p in track]
                for a, b in zip(tp, tp[1:]):
                    cv2.line(out, a, b, (0, 140, 255), 3, cv2.LINE_AA)
                if target:
                    cv2.circle(out, self._px(H, target), 7, (0, 220, 0), 2)
            cv2.rectangle(out, (0, 0), (out.shape[1], 52), (255, 255, 255), -1)
            cv2.putText(out, text, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (20, 20, 20), 2, cv2.LINE_AA)
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

    def event(self, kind, **data):
        self.events.append(dict(t=time.time(), kind=kind, **data))

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        if self.writer:
            self.writer.release()
        if self.raw:
            self.raw.release()
        self.cam.close()
        H = self.H.tolist() if self.H is not None else None
        json.dump({"H": H, "surface": self.cfg["surface_cm"], "raw": os.path.basename(self.raw_path),
                   "times": self.times, "events": self.events, "track": self.full_track or []},
                  open(self.path.replace(".mp4", "-session.json"), "w"), default=float)
        # The phone delivers ~21-24 fps but the file says 30: restore real-time speed.
        if self.frames > 30 and self.t_first:
            fps = self.frames / (time.time() - self.t_first)
            tmp = self.path.replace(".mp4", ".tmp.mp4")
            if os.system(f'ffmpeg -hide_banner -loglevel error -y -itsscale {30 / fps:.4f} -i "{self.path}" -c copy "{tmp}"') == 0:
                os.replace(tmp, self.path)


def dturn(a, b):
    """Signed angle from a to b, degrees, in (-180, 180]."""
    return (b - a + 180) % 360 - 180


class Tracker:
    """Second camera consumer: locates the robot on every frame, keeps the true track, streams
    `fix` while .fix is on, and pauses the robot as soon as its centre leaves the safe box."""

    def __init__(self, rec, H, box):
        self.rec, self.H, self.box, self.cfg = rec, H, box, rec.cfg
        from bench import Robot
        self.r = Robot()  # own HTTP connection: bench.Robot is not thread safe
        self.track, self.fix, self.world, self.mark = [], False, None, 0.0
        self.sent = self.rejected = self.errors = 0
        self.last_error = ""
        self.out, self.running = None, True
        self.latency = self.cfg.get("camera_latency_ms", 80)
        self.front = self.cfg.get("front_from_beacon_deg", 66.0)
        import threading
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set_origin(self, r, pose):
        """Call right after op origin: maps camera poses to the firmware world frame."""
        s = r.status()
        self.world = (pose["x"], pose["y"], math.radians(s["mount"] - s.get("front", 0)))

    def _to_world(self, p):
        x0, y0, th = self.world
        dx, dy = (p["x"] - x0) / 100, (p["y"] - y0) / 100
        c, s = math.cos(-th), math.sin(-th)
        return c * dx - s * dy, s * dx + c * dy, math.radians(dturn(0, p["heading"] + self.front - 90))

    def _run(self):
        n = 0
        while self.running:
            try:
                img, shot = self.rec.frame(with_time=True)
                _, p = vision.locate(img, self.cfg, self.H)
            except Exception:
                continue
            if not p:
                continue
            self.track.append((shot, p["x"], p["y"], p["heading"]))
            self.rec.track = [(x, y) for t, x, y, _ in self.track[-3000:] if t >= self.mark]
            x0, x1, y0, y1 = self.box
            if self.out is None and not (x0 - 2 <= p["x"] <= x1 + 2 and y0 - 2 <= p["y"] <= y1 + 2):
                self.r.cmd("pause")
                self.out = (p["x"], p["y"])
            n += 1
            if self.fix and self.world and n % 2 == 0:
                try:
                    wx, wy, wyaw = self._to_world(p)
                    ack = self.r.cmd("fix", x=wx, y=wy, yaw=wyaw, age=int((time.time() - shot) * 1000 + self.latency))
                    self.sent += 1
                    self.rejected += not ack.get("ok")
                except Exception as err:  # never let a fix problem kill the safety watch
                    self.errors += 1
                    self.last_error = repr(err)

    def since(self, t):
        return [q for q in self.track if q[0] >= t]

    def close(self):
        self.running = False
        self.thread.join(timeout=2)


def dirt_outlines(dirt, surface, ppc=PPC):
    """Dirt mask (rectified image) -> outlines in board cm, for drawing on the camera view."""
    cs, _ = cv2.findContours(cv2.dilate(dirt, np.ones((3, 3), np.uint8)), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h = surface[1]
    return [[(float(p[0][0]) / ppc, h - float(p[0][1]) / ppc) for p in c] for c in cs if len(c) >= 3]


# ------------------------------------------------------------------------------------ execution
def wait_still(r, timeout=60, trk=None):
    end = time.time() + timeout
    time.sleep(0.3)
    while time.time() < end:
        if trk and trk.out:
            raise SystemExit(f"Robot center ({trk.out[0]:.1f}, {trk.out[1]:.1f}) left the safe area; paused")
        if trk and (not trk.thread.is_alive() or trk.errors > 5):
            r.cmd("pause")
            raise SystemExit(f"Camera correction thread failed ({trk.last_error or 'thread exited'}); stopped")
        s = r.status()
        if s["src"] == "none" and not s["moving"]:
            time.sleep(0.3)
            return s
        time.sleep(0.08)
    r.cmd("pause")
    raise SystemExit("Move timed out; stopped")


def square_up(r, cam, H):
    front = cam.cfg.get("front_from_beacon_deg", 66.0)
    for _ in range(3):
        H, v = vision.measure(cam, n=7, H=H)
        turn = (90 - (v["heading"] + front) + 180) % 360 - 180
        if abs(turn) < 1.0:
            return H, v
        r.cmd("turn", deg=turn)
        wait_still(r)
    return vision.measure(cam, n=7, H=H)


def run_vision_path(r, cam, H, path, box, speed, accel, step=30.0, tol=0.5, trk=None):
    """Drive the route with the camera in the loop. A segment longer than `step` cm is cut into steps
    along the planned line, each step is aimed from the measured position (so the robot is pulled back
    onto the line instead of drifting parallel to it), and a waypoint missed by more than `tol` cm gets
    up to two correction moves."""
    x0, x1, y0, y1 = box
    H, here = vision.measure(cam, n=7, H=H)
    pos, heading0 = (here["x"], here["y"]), here["heading"]
    rec = isinstance(cam, ClipRecorder)
    t0 = time.time()
    short = 0  # consecutive real steps that travelled under half the command

    def go(tgt):
        nonlocal H, pos, short
        dx, dy = tgt[0] - pos[0], tgt[1] - pos[1]
        dist = math.hypot(dx, dy)
        ack = r.cmd("move", x=dx / 100, y=dy / 100, speed=speed, accel=accel)
        if not ack.get("ok"):
            raise SystemExit(f"move rejected: {ack.get('error')}")
        wait_still(r, timeout=10 + dist / (100 * speed) * 2, trk=trk)
        for attempt in range(30):  # someone in front of the camera: hold and wait, do not drive blind
            try:
                H, now = vision.measure(cam, n=5, H=H)
                break
            except RuntimeError:
                if attempt == 0:
                    r.cmd("pause")
                    print("  Cannot see the robot (someone blocking the camera?); waiting in place")
                time.sleep(2)
        else:
            r.cmd("pause")
            raise SystemExit("Robot not seen for a whole minute; stopped")
        moved = ((now["x"] - pos[0]) * dx + (now["y"] - pos[1]) * dy) / dist
        turn = (now["heading"] - heading0 + 180) % 360 - 180
        # Moves under ~3 cm lose much of their length to wheel slack, so only longer ones judge "stuck".
        if dist >= 3:
            short = short + 1 if moved < 0.5 * dist else 0
            if moved > 1.6 * dist:
                r.cmd("pause")
                raise SystemExit(f"This step moved {100 * moved / dist:.0f}% (slid down?); stopped")
            if short >= 3:
                r.cmd("pause")
                raise SystemExit(f"Three steps in a row moved less than half (this one {100 * moved / dist:.0f}%); may be stuck; stopped")
        if abs(turn) > 12:
            r.cmd("pause")
            raise SystemExit(f"Heading turned {turn:+.1f}°; stopped")
        if not (x0 - 2 <= now["x"] <= x1 + 2 and y0 - 2 <= now["y"] <= y1 + 2):
            r.cmd("pause")
            raise SystemExit(f"Robot center ({now['x']:.1f}, {now['y']:.1f}) left the safe area; stopped")
        if abs(turn) > 1.5:  # slip on the wall rotates the body; square it again so board-frame moves stay true
            r.cmd("turn", deg=-turn)
            wait_still(r, trk=trk)
            H, now = vision.measure(cam, n=5, H=H)
            print(f"      Heading off by {turn:+.1f}°; straightened")
        pos = (now["x"], now["y"])

    prev = pos
    for k, tgt in enumerate(path):
        if rec:
            cam.target = tgt
            cam.text = f"Erasing: point {k + 1}/{len(path)}   {time.time() - t0:4.0f} s"
        n = max(1, math.ceil(math.hypot(tgt[0] - prev[0], tgt[1] - prev[1]) / step))
        for j in range(1, n + 1):
            sub = (prev[0] + (tgt[0] - prev[0]) * j / n, prev[1] + (tgt[1] - prev[1]) * j / n)
            if math.hypot(sub[0] - pos[0], sub[1] - pos[1]) >= 0.3:
                go(sub)
        for _ in range(2):
            if math.hypot(tgt[0] - pos[0], tgt[1] - pos[1]) <= tol:
                break
            go(tgt)
        print(f"  {k + 1}/{len(path)}  target ({tgt[0]:.1f}, {tgt[1]:.1f})  actual ({pos[0]:.1f}, {pos[1]:.1f})  "
              f"error {math.hypot(pos[0] - tgt[0], pos[1] - tgt[1]):.1f} cm")
        prev = tgt
    return H


def run_blind_path(r, path, start, speed, accel):
    pos = start
    for k, tgt in enumerate(path):
        dx, dy = tgt[0] - pos[0], tgt[1] - pos[1]
        dist = math.hypot(dx, dy)
        if dist < 0.3:
            continue
        ack = r.cmd("move", x=dx / 100, y=dy / 100, speed=speed, accel=accel)
        if not ack.get("ok"):
            raise SystemExit(f"move rejected: {ack.get('error')}")
        wait_still(r, timeout=10 + dist / (100 * speed) * 2)
        print(f"  {k + 1}/{len(path)}  to ({tgt[0]:.1f}, {tgt[1]:.1f})")
        pos = tgt


# ------------------------------------------------------------------------------------ fine wipe
def area_mask(outlines, surface, margin, eraser=2.0, grow=1.0, ppc=PPC):
    """Where ink was: the filled outlines (board cm) grown by `grow` cm, cut to what the eraser can
    reach, specks under 1 cm^2 dropped. Rectified grid."""
    w, h = surface
    m = np.zeros((int(round(h * ppc)), int(round(w * ppc))), np.uint8)
    for poly in outlines:
        pts = np.array([[p[0] * ppc, (h - p[1]) * ppc] for p in poly], np.int32)
        cv2.fillPoly(m, [pts], 255)
    k = max(1, int(round(grow * ppc)))
    m = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
    x0, x1, y0, y1 = reachable_box(surface, margin)
    r = eraser / 2
    reach = np.zeros_like(m)
    cv2.rectangle(reach, (int((x0 - r) * ppc), int((h - y1 - r) * ppc)), (int((x1 + r) * ppc), int((h - y0 + r) * ppc)), 255, -1)
    m &= reach
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    for i in range(1, n):
        if st[i, cv2.CC_STAT_AREA] / ppc ** 2 < 1.0:
            m[lab == i] = 0
    return m


def plan_fine(area, surface, start, pitch=1.0, margin=11.0, ppc=PPC, sweep="horizontal", bottom_up=True):
    """Fine wipe: lanes `pitch` cm apart over the area (left-right or up-down), each wiped there and
    back, so with the 2 cm cleaned width every point is under the felt about 4 times."""
    x0, x1, y0, y1 = reachable_box(surface, margin)
    ys, xs = np.nonzero(area)
    if not len(xs):
        return []
    cells = np.stack([(xs + 0.5) / ppc, surface[1] - (ys + 0.5) / ppc], 1)
    axis = 0 if sweep == "horizontal" else 1  # 0: lanes run along X, stacked in Y
    lanes = _lanes(cells, axis, pitch, pitch, 0.0)
    across, along = start[1 - axis], start[axis]
    if not (bottom_up and axis == 0) and abs(lanes[-1][0] - across) < abs(lanes[0][0] - across):
        lanes = lanes[::-1]  # lanes come out lowest first
    low = abs(min(lo for _, lo, _ in lanes) - along) <= abs(max(hi for _, _, hi in lanes) - along)
    pts = []
    for cp, lo, hi in lanes:
        a_, b_ = (lo, hi) if low else (hi, lo)
        for v in (a_, b_, a_):
            q = (v, cp) if axis == 0 else (cp, v)
            q = (min(max(q[0], x0), x1), min(max(q[1], y0), y1))
            if not pts or math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 0.2:
                pts.append(q)
    return pts


def park_spot(area, surface, margin, near, clear=12.0, ppc=PPC):
    """Nearest robot position (cm) whose body leaves the whole area visible to the camera."""
    x0, x1, y0, y1 = reachable_box(surface, margin)
    dist = cv2.distanceTransform(cv2.bitwise_not(area), cv2.DIST_L2, 5) / ppc
    best = None
    for x in np.arange(x0, x1 + 1e-6, 1.0):
        for y in np.arange(y0, y1 + 1e-6, 1.0):
            r_, c_ = int((surface[1] - y) * ppc), int(x * ppc)
            if 0 <= r_ < dist.shape[0] and 0 <= c_ < dist.shape[1] and dist[r_, c_] >= clear:
                d = math.hypot(x - near[0], y - near[1])
                if best is None or d < best[0]:
                    best = (d, (float(x), float(y)))
    return best[1] if best else None


def still(cam, n=5):
    """Pixel median of n frames (camera noise, beacon flicker)."""
    return np.median(np.stack([cam.frame() for _ in range(n)]), axis=0).astype(np.uint8)


def clear_still(cam, H, tries=15):
    """A still in which the robot's beacon is visible, i.e. nobody stands in front of the board."""
    for _ in range(tries):
        img = still(cam)
        _, p = vision.locate(img, cam.cfg, H)
        if p:
            return img, p
        print("  Something is blocking the camera; waiting 2 s")
        time.sleep(2)
    raise SystemExit("The camera stays blocked")


def park(r, cam, H, cfg, args, area, trk, text="Moving clear of the drawing"):
    """Drive to the nearest spot where the robot body hides none of `area`; returns H."""
    H, here = vision.measure(cam, n=7, H=H)
    spot = park_spot(area, cfg["surface_cm"], args.margin, (here["x"], here["y"]))
    if spot and math.hypot(spot[0] - here["x"], spot[1] - here["y"]) > 1.0:
        cam.text = text
        cam.event("park", to=spot)
        H = run_vision_path(r, cam, H, [spot], reachable_box(cfg["surface_cm"], args.margin), 0.05, args.accel,
                            args.step, args.tol, trk)
    return H


def residue(img, H, surface, area, pose=None, robot_radius=11.0, ppc=8.0):
    """Faint marks left in the area: mean colour distance above that of the clean board around it,
    and the area of pixels clearly off (dE > 8) compared with the same density outside. Measured
    at 1.25 mm pixels: the ghost dashes black hatching leaves are ~1 mm wide and vanish at 2.5 mm."""
    top, _ = rectify(img, H, surface, ppc=ppc)
    dE = colour_distance(top, ppc)
    area = cv2.resize(area, (top.shape[1], top.shape[0]), interpolation=cv2.INTER_NEAREST)
    inside = area > 0
    k = int(4 * ppc) | 1
    ring = (cv2.dilate(area, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))) > 0) & ~inside
    if pose:
        hide = robot_mask(area.shape, pose, robot_radius, surface, ppc) > 0
        inside &= ~hide
        ring &= ~hide
    a, b = dE[inside], dE[ring]
    return {"excess": float(a.mean() - b.mean()) if a.size and b.size else float("nan"),
            "faint_cm2": float(np.count_nonzero(a > 8) / ppc ** 2),
            "faint_pct": float(100 * np.mean(a > 8)) if a.size else 0.0,
            "ring_pct": float(100 * np.mean(b > 8)) if b.size else 0.0,
            "seen_cm2": float(a.size / ppc ** 2)}


def fine_wipe(r, cam, H, cfg, args, area, trk):
    """Park clear of the area, photograph, wipe it finely --fine-passes times, photograph again."""
    surface = cfg["surface_cm"]
    box = reachable_box(surface, args.margin)
    H, here = vision.measure(cam, n=7, H=H)
    spot = park_spot(area, surface, args.margin, (here["x"], here["y"]))
    cam.dirt = dirt_outlines(area, surface)
    cam.route, cam.target = [], None
    if spot:
        cam.text = "Fine wipe: parking clear of the area for a photo"
        H = run_vision_path(r, cam, H, [spot], box, 0.05, args.accel, args.step, args.tol, trk)
    img, here = clear_still(cam, H)
    res = [residue(img, H, surface, area, here)]
    print(f"Before fine erase: residue {res[0]['excess']:.2f} ΔE, visible spots {res[0]['faint_cm2']:.1f} cm² "
          f"({res[0]['faint_pct']:.1f}%, clean board nearby {res[0]['ring_pct']:.1f}%), seen {res[0]['seen_cm2']:.0f} cm²")
    for k in range(args.fine_passes):
        path = plan_fine(area, surface, (here["x"], here["y"]), args.fine_pitch, args.margin, sweep=args.sweep,
                         bottom_up=args.bottom_up)
        if spot:
            path.append(spot)
        length = _length(path, (here["x"], here["y"]))
        print(f"Fine erase pass {k + 1}: {len(path)} points, {length / 100:.2f} m, {'left-right' if args.sweep == 'horizontal' else 'up-down'}"
              f" lane spacing {args.fine_pitch:g} cm, {100 * args.fine_speed:g} cm/s, each lane back and forth")
        if trk:
            trk.mark = time.time()
        full = [(here["x"], here["y"])] + path
        for i in range(2, len(full) + 1, max(1, len(full) // 20)):
            cam.route = full[:i]
            time.sleep(0.08)
        cam.route = full
        cam.text = f"Fine wipe {k + 1}: lanes {args.fine_pitch:g} cm apart, there and back, {length / 100:.1f} m"
        time.sleep(1.5)
        cam.event("fine_start", pass_=k + 1, area=dirt_outlines(area, surface), route=full, path_cm=length)
        H = run_vision_path(r, cam, H, path, box, args.fine_speed, args.accel, args.step, args.tol, trk)
        cam.event("fine_end", pass_=k + 1)
        img, here = clear_still(cam, H)
        res.append(residue(img, H, surface, area, here))
        q = res[-1]
        print(f"After fine erase pass {k + 1}: residue {q['excess']:.2f} ΔE, visible spots {q['faint_cm2']:.1f} cm² "
              f"({q['faint_pct']:.1f}%, clean board nearby {q['ring_pct']:.1f}%)")
        cam.text = f"Fine wipe {k + 1} done: residue {res[0]['excess']:.2f} -> {q['excess']:.2f} dE"
    return H, res


# ------------------------------------------------------------------------------------------ CLI
def save(tag, img, data):
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    stamp = time.strftime("%m%d-%H%M%S")
    jp = os.path.join(ROOT, "logs", f"clean-{stamp}-{tag}.jpg")
    cv2.imwrite(jp, img)
    json.dump(data, open(jp.replace(".jpg", ".json"), "w"), indent=1)
    return jp


def scan_once(img, cfg, args, H=None, pose=None):
    H = H if H is not None else vision.surface_homography(img, cfg)
    if H is None:
        raise SystemExit("Cannot find the red tape on the four corners")
    if pose is None:
        _, pose = vision.locate(img, cfg, H)
    dirt, top, info = detect_dirt(img, cfg, H, pose, args.sensitivity, args.robot_radius)
    start = (pose["x"], pose["y"]) if pose else (cfg["surface_cm"][0] / 2, args.margin)
    if args.lanes == "strokes":
        path, rep = plan_strokes(dirt, cfg["surface_cm"], start, args.eraser, args.overlap, args.margin,
                                 sweep=args.sweep)
    else:
        path, rep = plan(to_grid(dirt), cfg["surface_cm"], start, args.eraser, args.overlap, args.margin, args.merge,
                         lanes=args.sweep if args.lanes == "sweep" else "auto", bottom_up=args.bottom_up)
    rep.update(dirty_cm2=info["dirty_cm2"], robot=pose)
    over = draw(top, dirt, path, start, cfg["surface_cm"], rep)
    rep["outlines"] = dirt_outlines(dirt, cfg["surface_cm"])
    rep["start"] = start
    return H, pose, path, rep, side_by_side(img, H, top, over)


def describe(rep, path):
    kind = (f"along {rep['strokes']} strokes + sweep {rep.get('lane_regions', 0)} regions" if "strokes" in rep
            else f"in {rep['regions']} region(s)")
    print(f"dirty area {rep['dirty_cm2']:.0f} cm², {kind}; reachable {rep['covered_cm2']:.0f} cm², "
          f"too close to the frame to reach {rep['unreachable_cm2']:.0f} cm²; route {len(path)} points, {rep['path_cm'] / 100:.2f} m total, "
          f"lane spacing {rep.get('pitch_cm', 0):.1f} cm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["scan", "vision", "fine", "blind"])
    ap.add_argument("--image", help="scan: plan from this photo instead of the camera")
    # sysid_erase.py 2026-10-28: one pass clears >=97 % within +-0.9 cm of the centre, ~50 % at
    # +-1.1 cm and nothing beyond +-1.4 cm, so the felt cleans 2 cm, not its 3 cm size.
    ap.add_argument("--eraser", type=float, default=2.0, help="cleaned width of one pass, cm (measured)")
    ap.add_argument("--eraser-offset", default="0,0", help="eraser centre minus robot centre in the board frame, cm")
    ap.add_argument("--overlap", type=float, default=0.2, help="lane overlap as a fraction of the eraser width")
    # 2026-10-28 the user: left-right only ("otherwise it jitters easily": up/down travel jitters), bottom to top.
    ap.add_argument("--lanes", choices=["strokes", "sweep", "auto"], default="sweep",
                    help="strokes: drive along the pen lines (lanes only for wide filled areas); "
                         "sweep: lanes everywhere; auto: one traced line per region, else the shorter lane direction")
    ap.add_argument("--sweep", choices=["horizontal", "vertical"], default="horizontal",
                    help="lane direction for filled areas and the fine wipe (the user wants left-right, 2026-10-28)")
    ap.add_argument("--top-down", dest="bottom_up", action="store_false", help="do not force bottom-to-top lane order")
    ap.add_argument("--margin", type=float, default=11.0, help="robot centre to frame, cm")
    ap.add_argument("--merge", type=float, default=1.0, help="dirt closer than this (cm) is erased as one region")
    ap.add_argument("--sensitivity", type=float, default=1.5,
                    help=">1 also picks up fainter marks (1.5: faint outlines yes, ghosting no, 2026-10-28)")
    ap.add_argument("--robot-radius", type=float, default=10.0, help="area around the robot ignored by detection, cm")
    ap.add_argument("--speed", type=float, default=0.05, help="m/s while erasing")
    ap.add_argument("--accel", type=float, default=0.15, help="m/s^2")
    ap.add_argument("--passes", type=int, default=3, help="vision: look again and repeat up to this many times")
    ap.add_argument("--step", type=float, default=30.0, help="vision: longest move between camera checks, cm")
    ap.add_argument("--tol", type=float, default=0.5, help="vision: correct a waypoint missed by more than this, cm")
    ap.add_argument("--dry-run", action="store_true", help="vision: plan and draw, do not move")
    ap.add_argument("--no-fix", action="store_true", help="vision: do not stream camera fixes (open loop moves)")
    ap.add_argument("--fine-passes", type=int, default=1,
                    help="vision/fine: fine wipes over everywhere ink was seen, after the normal passes")
    ap.add_argument("--fine-pitch", type=float, default=1.0, help="fine wipe lane spacing, cm")
    ap.add_argument("--fine-speed", type=float, default=0.03, help="fine wipe speed, m/s")
    ap.add_argument("--area-from", help="fine: clean-*-pass1.json whose ink area to wipe (default: newest)")
    ap.add_argument("--no-park", dest="park", action="store_false",
                    help="vision: do not move clear of the drawing for the first and the last photo")
    ap.add_argument("--at", help="blind: robot centre now, cm, 'x,y'")
    ap.add_argument("--rect", action="append", default=[], help="blind: area to erase, cm, 'x0,y0,x1,y1' (repeatable)")
    ap.add_argument("--full", action="store_true", help="blind: the whole reachable board")
    args = ap.parse_args()
    cfg = vision.config()
    off = tuple(float(v) for v in args.eraser_offset.split(","))

    def robot_targets(path):  # eraser targets -> robot centre targets
        return [(x - off[0], y - off[1]) for x, y in path]

    if args.mode == "scan":
        if args.image:
            img = cv2.imread(args.image)
            if img is None:
                raise SystemExit(f"Cannot read image {args.image}")
        else:
            cam = vision.Camera(cfg)
            img = cam.frame()
            cam.close()
        H, pose, path, rep, vis = scan_once(img, cfg, args)
        describe(rep, path)
        print("Result image:", save("scan", vis, {"path": path, "report": rep}))
        return

    from bench import Robot
    r = Robot()
    s = r.status()
    if s["mode"] != "active":
        r.cmd("enable")
        time.sleep(1.0)
    r.cmd("abort")

    if args.mode == "blind":
        if not args.at:
            raise SystemExit("blind mode needs --at x,y to tell me where the robot is now (whiteboard coordinates, cm)")
        start = tuple(float(v) for v in args.at.split(","))
        w, h = cfg["surface_cm"]
        rects = [tuple(float(v) for v in t.split(",")) for t in args.rect]
        if args.full:
            rects.append((0, 0, w, h))
        if not rects:
            raise SystemExit("blind mode needs --rect or --full")
        path, rep = plan_rects(rects, cfg["surface_cm"], start, args.eraser, args.overlap, args.margin)
        print(f"Route {len(path)} points, {rep['path_cm'] / 100:.2f} m total")
        run_blind_path(r, robot_targets(path), start, args.speed, args.accel)
        print("Done (open loop, no camera check)")
        return

    # vision / fine
    area_polys = []
    if args.mode == "fine":
        import glob
        src = args.area_from or max(glob.glob(os.path.join(ROOT, "logs", "clean-*-pass1.json")), key=os.path.getmtime)
        area_polys = json.load(open(src))["report"]["outlines"]
        print("Fine erase area taken from:", src)
    r.cmd("beacon", on=True)
    time.sleep(0.5)
    os.makedirs(os.path.join(ROOT, "logs"), exist_ok=True)
    video = os.path.join(ROOT, "logs", time.strftime("clean-%m%d-%H%M%S.mp4"))
    cam = ClipRecorder(vision.Camera(cfg), video)
    cam.text = "WhiteBoardBot: looking for dirt"
    trk = None
    try:
        H = vision.surface_homography(cam.frame(), cfg)
        if H is None:
            raise SystemExit("Cannot find the red tape on the four corners")
        cam.H, cam.box = H, reachable_box(cfg["surface_cm"], args.margin)
        if not args.dry_run:
            H, v = square_up(r, cam, H)
            # Live camera fixes (sysid_erase.py 2026-10-28): 8 cm moves land within 0.03 cm
            # instead of 0.5 cm open loop, also on ink.
            trk = Tracker(cam, H, reachable_box(cfg["surface_cm"], args.margin))
            r.cmd("origin")
            trk.set_origin(r, v)
            trk.fix = not args.no_fix
            time.sleep(1.5)
            if trk.fix and (trk.sent < 3 or trk.errors or trk.rejected > trk.sent // 2):
                raise SystemExit(f"Camera correction did not start: sent {trk.sent}, rejected {trk.rejected}, errors {trk.errors} "
                                 f"{trk.last_error}")
            print(f"Camera correction on: sent {trk.sent} times in 1.5 s, rejected {trk.rejected} times")
            cam.event("start")
            if args.mode == "vision" and args.park:
                # Start clear of the drawing so the first photo shows all of it (ink under the robot
                # would only be found in pass 2).
                H, pose = vision.measure(cam, n=7, H=H)
                dirt0, _, _ = detect_dirt(cam.frame(), cfg, H, pose, args.sensitivity, args.robot_radius)
                if np.any(dirt0):
                    H = park(r, cam, H, cfg, args, cv2.dilate(dirt0, np.ones((9, 9), np.uint8)), trk)
        for k in range(args.passes if args.mode == "vision" else 0):
            H, pose = vision.measure(cam, n=7, H=H)
            img = cam.frame()
            H, pose, path, rep, vis = scan_once(img, cfg, args, H, pose)
            print(f"Pass {k + 1}: ", end="")
            describe(rep, path)
            print("  Image:", save(f"pass{k + 1}", vis, {"path": path, "report": rep}))
            area_polys += rep["outlines"]
            cam.event("scan", pass_=k + 1, outlines=rep["outlines"], route=[rep["start"]] + robot_targets(path),
                      dirty_cm2=rep["dirty_cm2"], strokes=rep.get("strokes", 0), lanes=rep.get("lane_regions", 0),
                      sweep=args.sweep, mode=args.lanes, pitch=rep.get("pitch_cm", 0),
                      path_cm=rep["path_cm"])
            # Show the detection and the plan in the video before driving.
            cam.H, cam.dirt, cam.box, cam.track, cam.target = H, rep["outlines"], rep["safe_box"], [], None
            if trk:
                trk.mark = time.time()
            cam.route = []
            cam.text = (f"Pass {k + 1}: {rep['regions']} dirty regions, {rep['dirty_cm2']:.0f} cm^2 - planning route"
                        if path else f"Pass {k + 1}: board clean")
            time.sleep(1.5)
            full = [rep["start"]] + robot_targets(path)
            for i in range(2, len(full) + 1, max(1, len(full) // 20)):  # draw the route growing
                cam.route = full[:i]
                time.sleep(0.08)
            cam.route = full
            if path:
                cam.text = f"Pass {k + 1}: route {len(path)} points, {rep['path_cm'] / 100:.2f} m (eraser {args.eraser:.0f} cm)"
            time.sleep(2.0)
            if not path:
                print("Nothing left to erase")
                break
            if args.dry_run:
                break
            cam.event("erase_start", pass_=k + 1)
            H = run_vision_path(r, cam, H, robot_targets(path), rep["safe_box"], args.speed, args.accel, args.step,
                                args.tol, trk)
            cam.event("erase_end", pass_=k + 1)
        if not args.dry_run and args.fine_passes > 0 and area_polys:
            area = area_mask(area_polys, cfg["surface_cm"], args.margin, args.eraser)
            print(f"Fine erase area {np.count_nonzero(area) / PPC ** 2:.0f} cm²")
            H, res = fine_wipe(r, cam, H, cfg, args, area, trk)
            save("fine", cv2.cvtColor(area, cv2.COLOR_GRAY2BGR), {"residue": res, "outlines": area_polys})
        if not args.dry_run:
            if area_polys and args.park:
                H = park(r, cam, H, cfg, args, area_mask(area_polys, cfg["surface_cm"], args.margin, args.eraser), trk,
                         "Final check: moving clear of the area")
            img, pose = clear_still(cam, H)
            H, pose, path, rep, vis = scan_once(img, cfg, args, H, pose)
            cam.event("final", outlines=rep["outlines"], dirty_cm2=rep["dirty_cm2"])
            print("Final check: ", end="")
            describe(rep, path)
            print("  Image:", save("final", vis, {"path": path, "report": rep}))
            cam.dirt, cam.route, cam.target = rep["outlines"], [], None
            cam.text = f"Done: {rep['dirty_cm2']:.0f} cm^2 of marks left"
            time.sleep(2.0)
    finally:
        r.cmd("pause")  # hold wherever it is, also after an error
        if trk:
            cam.full_track = list(trk.track)
            trk.close()
            print(f"Camera correction {trk.sent} times, rejected {trk.rejected} times")
        cam.close()
        print("Video:", video)


if __name__ == "__main__":
    main()
