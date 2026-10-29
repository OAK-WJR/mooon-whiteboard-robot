#!/usr/bin/env python3
"""Rebuild the design-history figures from the EasyEDA project in git.

EasyEDA Pro stores a preview image of every schematic page and PCB inside the
project file (IMAGE/<document id>.webp) each time the project is saved. This
script pulls those previews out of hardware/top_board_v2/motor_controller.epro2
at each commit that changed it, so every figure can be traced to a commit.

    python3 docs/design-history/make_figures.py      (needs Pillow)
"""
import io
import os
import subprocess
import zipfile

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
EPRO = "hardware/top_board_v2/motor_controller.epro2"
FONT = "/System/Library/Fonts/Helvetica.ttc"

# EasyEDA document ids in project "motor_controller"
LOGIC_PCB, LOGIC_SCH = "d66fa5125fd977da", "047e235f224c2688"
FAMILY = [  # (document id, caption, sub-caption)
    ("713d18881d67b0a9", "Mar 2026 - v1 board", "socket for an ESP32 DevKit, 3x RZ7899"),
    ("a7681f7930a373e5", "Jul 2026 - base board", "3x RZ7889 H-bridge, battery (bottom side)"),
    ("29868d186e4fc126", "Jul 2026 - ESP32-C6 adapter", "plugs the C6 LCD module onto the base"),
    ("c2976160f80574ea", "Sep 2026 - top_board_v2", "first single-board controller (not built)"),
    ("1451795a9a99c5a7", "Sep 2026 - top_board_v2 R2", "after a KiCad round trip (not built)"),
    (LOGIC_PCB, "Oct 2026 - top_board_v2_logic", "current design, routing in progress"),
]
EVOLUTION = [  # (commit, short caption)
    ("170676d", "imported; schematic redrawn on one A3 sheet"),
    ("1f5815d", "sensor parts moved to a separate head board"),
    ("ae20f6c", "ESP32 escape rerouted, routing curved"),
    ("8227cd7", "tidy, uniform arcs"),
    ("c1aba64", "tried the EasyEDA autorouter (40/46 nets)"),
    ("766cd35", "MOOON-style re-route (v5)"),
    ("a0c2144", "LCSC parts, copper pour; charger in schematic"),
]


def font(size):
    try:
        return ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default(size=size)


def git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, check=True).stdout


def previews(commit):
    z = zipfile.ZipFile(io.BytesIO(git("show", f"{commit}:{EPRO}")))
    return {n[6:-5]: z.read(n) for n in z.namelist() if n.startswith("IMAGE/") and n.endswith(".webp")}


def commit_info(commit):
    line = git("log", "-1", "--date=format:%Y-%m-%d %H:%M", "--format=%h|%ad|%s", commit).decode().strip()
    h, date, subject = line.split("|", 2)
    return h, date, subject.removeprefix("hardware: ")


def crop_dark(im, pad=12, density=0.03):
    """Trim the black canvas around a PCB preview, ignoring thin off-board notes."""
    mask = im.convert("L").point(lambda v: 255 if v > 40 else 0)
    cols = list(mask.resize((im.width, 1), Image.BOX).getdata())
    rows = list(mask.resize((1, im.height), Image.BOX).getdata())
    xs = [i for i, v in enumerate(cols) if v > 255 * density] or [0, im.width - 1]
    ys = [i for i, v in enumerate(rows) if v > 255 * density] or [0, im.height - 1]
    return im.crop((max(xs[0] - pad, 0), max(ys[0] - pad, 0),
                    min(xs[-1] + pad, im.width), min(ys[-1] + pad, im.height)))


def tile(im, w, h, bg):
    im = im.copy()
    im.thumbnail((w, h), Image.LANCZOS)
    out = Image.new("RGB", (w, h), bg)
    out.paste(im, ((w - im.width) // 2, (h - im.height) // 2))
    return out


def grid(cells, cols, w, h, cap_h, bg, path):
    rows = (len(cells) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * w, rows * (h + cap_h)), "white")
    d = ImageDraw.Draw(sheet)
    for k, (im, title, sub) in enumerate(cells):
        x, y = (k % cols) * w, (k // cols) * (h + cap_h)
        sheet.paste(tile(im, w, h, bg), (x, y))
        d.text((x + 14, y + h + 10), title, fill="black", font=font(26))
        d.text((x + 14, y + h + 44), sub, fill="#555555", font=font(20))
    sheet.save(path, quality=86)
    print("wrote", os.path.relpath(path, ROOT))


def main():
    head = git("log", "-1", "--format=%h", "--", EPRO).decode().strip()
    latest = previews(head)
    load = lambda data: Image.open(io.BytesIO(data)).convert("RGB")

    cells = [(crop_dark(load(latest[i])), t, s) for i, t, s in FAMILY]
    grid(cells, 3, 640, 480, 84, "black", os.path.join(HERE, "board-generations.jpg"))

    cells = []
    for c, caption in EVOLUTION:
        h, date, _ = commit_info(c)
        cells.append((crop_dark(load(previews(c)[LOGIC_PCB])), f"{date}  ({h})", caption))
    live = os.path.join(HERE, "logic-pcb-live-2026-10-28.png")
    if os.path.exists(live):
        im = Image.open(live).convert("RGB")
        im = im.crop((18, 18, im.width, im.height))  # drop the editor rulers
        cells.append((crop_dark(im), "2026-10-28 19:20  (live)",
                      "charger parts placed, not routed yet"))
    grid(cells, 4, 480, 440, 84, "black", os.path.join(HERE, "logic-board-evolution.jpg"))

    cells = []
    for c in (EVOLUTION[0][0], EVOLUTION[-1][0]):
        h, date, subject = commit_info(c)
        cells.append((load(previews(c)[LOGIC_SCH]), f"{date}  ({h})", subject[:90]))
    grid(cells, 2, 1200, 850, 84, "white", os.path.join(HERE, "logic-schematic-before-after.jpg"))

    v1 = load(latest["c1e2d82be7579d22"])
    v1.save(os.path.join(HERE, "v1-schematic.jpg"), quality=88)
    print("wrote docs/design-history/v1-schematic.jpg")


if __name__ == "__main__":
    main()
