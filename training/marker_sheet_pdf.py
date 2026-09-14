#!/usr/bin/env python3
"""Lay the tray marker sheets out on one A4 page at exact physical scale.

WHY NOT PRINT THE PNGs DIRECTLY
-------------------------------
`assets/trays/*_markers.png` are 600 dpi but carry no resolution tag, so any
print dialog picks its own scale -- and the marker length is a term in
T_cam_marker, so a rescaled marker is a ground-truth scale error nothing
downstream can see. Worse, `bowl_markers.png` is 210.5 mm wide: wider than A4,
so even a correctly scaled print clips it.

This cuts the eight cards out of the strips and lays them on a single A4 page at
600 dpi, pasted pixel for pixel with no resampling, and writes a PDF whose page
IS A4. With `print-scaling=none` there is then nothing for the print path to
decide.

Cards get a 10 mm gutter rather than the strips' 20 px (0.85 mm). The recess is
52.4 mm for a 52 mm card, so a knife run down the middle of a 0.85 mm gap leaves
a 52.42 mm card that does not go in -- the gap has to be wide enough to cut each
edge on its own line.

Two further choices, both about the corners ArUco localises:

  * BILEVEL, CCITT G4. Pillow stores 8-bit greyscale in PDFs as JPEG, which
    rings at exactly the black/white edges the marker corners are. A 1-bit page
    is lossless and tiny. The markers are pure 0/255 already -- 945 px over
    7 modules is exactly 135 px per module -- so thresholding costs them nothing.
  * CUT LINES ON THE CARD EDGE, run long for scissors. The PNG's grey card
    outline does not survive thresholding, so each card edge gets a solid line
    that carries on across the gutters: every row and column becomes one straight
    cut. A line exactly on the edge sits 6 mm from the black square -- at the
    outer limit of the one-module (5.7 mm) quiet zone ArUco needs, not inside it
    -- and check() confirms every id still decodes with the lines drawn. Lines
    stop short of the scale bars, which share the page with the grid.

A 100 mm horizontal and a 100 mm vertical bar check both axes: a printer can
scale X and Y differently, and one bar would not catch that.

    python training/marker_sheet_pdf.py      # -> assets/print/aruco_markers_A4.pdf
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "assets" / "trays"
OUT = REPO / "assets" / "print" / "aruco_markers_A4.pdf"

DPI = 600
MM = DPI / 25.4
# A4 is 4960.6 x 7015.7 px at 600 dpi. Round DOWN so the page is never larger
# than A4 -- a page a hair over the media size is exactly what invites a
# driver to "fit" it.
PAGE_W, PAGE_H = 4960, 7015
CARD_PX = 1228            # author_marker_trays.marker_sheet: round(52 / 25.4 * 600)
GAP_PX = 20               # ... and the gap it leaves between cards
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def px(mm: float) -> int:
    return int(round(mm * MM))


def load(cls: str) -> np.ndarray:
    return np.array(Image.open(SRC / f"{cls}_markers.png").convert("L"))


def check(page: np.ndarray) -> None:
    """Refuse to write a sheet that would not print as drawn.

    PIL clips anything drawn off the canvas without complaint -- an earlier
    layout lost the whole vertical scale bar that way -- so the margins are
    measured from the rendered page rather than trusted from the layout maths.
    """
    import cv2
    ink = np.argwhere(page < 128)
    h, w = page.shape
    margins = dict(left=ink[:, 1].min(), right=w - 1 - ink[:, 1].max(),
                   top=ink[:, 0].min(), bottom=h - 1 - ink[:, 0].max())
    tight = {k: v / MM for k, v in margins.items() if v < px(8.0)}
    if tight:
        raise SystemExit(f"ink closer than 8 mm to the page edge: {tight}")
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250))
    _, ids, _ = det.detectMarkers(page)
    got = sorted(int(i) for i in ids.ravel()) if ids is not None else []
    if got != list(range(100, 108)):
        raise SystemExit(f"expected marker ids 100-107 on the page, decoded {got}")
    print("margins mm: " + ", ".join(f"{k} {v / MM:.1f}" for k, v in margins.items())
          + f"; decoded ids {got[0]}-{got[-1]}")


def cut_lines(draw: ImageDraw.ImageDraw, cards: list[tuple[int, int, int, int]]) -> None:
    """Solid lines along every card edge, extended across the gutters.

    Cards sit on a grid, so an edge line only ever runs along other cards' edges
    and through gutters, never across a card. Each line spans just the cards in
    its own row or column plus 5 mm, which keeps it clear of the scale bars in
    the grid's empty ninth cell and below it.
    """
    ext, half = px(5.0), 2                                # 4 px = 0.17 mm line
    rows: dict[int, list[int]] = {}
    cols: dict[int, list[int]] = {}
    for x, y, w, h in cards:
        rows.setdefault(y, []).append(x)
        cols.setdefault(x, []).append(y)
    for y, xs in rows.items():
        x0, x1 = min(xs) - ext, max(xs) + CARD_PX + ext
        for edge in (y, y + CARD_PX):
            draw.rectangle([x0, edge - half, x1, edge + half - 1], fill=0)
    for x, ys in cols.items():
        y0, y1 = min(ys) - ext, max(ys) + CARD_PX + ext
        for edge in (x, x + CARD_PX):
            draw.rectangle([edge - half, y0, edge + half - 1, y1], fill=0)


def main() -> int:
    page = np.full((PAGE_H, PAGE_W), 255, np.uint8)

    # Cut every card out of its strip, in marker-id order (100-107).
    cut = []
    for cls, n in (("red_cube", 2), ("mustard_bottle", 2), ("bowl", 4)):
        strip = load(cls)
        for i in range(n):
            x0 = i * (CARD_PX + GAP_PX)
            cut.append(strip[:CARD_PX, x0:x0 + CARD_PX])

    # Laser printers cannot mark the outer ~4-5 mm; keep ink >= 8 mm in.
    margin = px(14.0)
    pitch = CARD_PX + px(10.0)                            # 52 mm card + 10 mm gutter
    grid_top = px(50.0)
    cards = []                                            # (x, y, w, h) on the page
    for k, card in enumerate(cut):                        # 3 x 3 grid, 8 filled
        x, y = margin + (k % 3) * pitch, grid_top + (k // 3) * pitch
        page[y:y + CARD_PX, x:x + CARD_PX] = card
        cards.append((x, y, CARD_PX, CARD_PX))

    # Bilevel. The markers are exactly 0/255; only the grey outline and the
    # anti-aliased label change, and the outline is replaced by cut lines.
    img = Image.fromarray(np.where(page < 160, 0, 255).astype(np.uint8)).convert("1")
    draw = ImageDraw.Draw(img)
    cut_lines(draw, cards)

    f_big = ImageFont.truetype(FONT, px(4.2))
    f_small = ImageFont.truetype(FONT, px(3.0))
    draw.text((margin, px(12.0)), "The Eyes - ArUco tray markers (DICT_5X5_250, ids 100-107)",
              font=f_big, fill=0)
    lines = [
        "Print at 100% / actual size, print-scaling=none.",
        "Both bars must measure 100.0 mm; each black marker square 40.0 mm.",
        "Cut ON the lines (52 x 52 mm): the card's edges place the marker in its tray.",
        "Plain 80 gsm paper, glue stick - not card stock, not foam tape.",
    ]
    for k, line in enumerate(lines):
        draw.text((margin, px(19.5 + 5.0 * k)), line, font=f_small, fill=0)

    # 100 mm horizontal bar with 10 mm ticks, below the grid.
    below = grid_top + 3 * pitch                          # under the third row
    hx, hy, bar = margin, below + px(12.0), px(100.0)
    draw.rectangle([hx, hy, hx + bar, hy + 4], fill=0)
    for k in range(11):
        tx = hx + px(10.0 * k)
        draw.rectangle([tx, hy - px(3.0 if k % 5 == 0 else 1.5), tx + 3, hy + 4], fill=0)
    draw.text((hx, hy + px(3.0)), "100 mm (horizontal)", font=f_small, fill=0)

    # 100 mm vertical bar in the grid's empty ninth cell, running down past the
    # grid. It checks the paper-feed axis, which a printer can scale
    # independently of the other one.
    vx, vy = margin + 2 * pitch + CARD_PX // 2, grid_top + 2 * pitch
    draw.rectangle([vx, vy, vx + 4, vy + bar], fill=0)
    for k in range(11):
        ty = vy + px(10.0 * k)
        draw.rectangle([vx - px(3.0 if k % 5 == 0 else 1.5), ty, vx + 4, ty + 3], fill=0)
    draw.text((vx - px(40.0), vy + px(80.0)), "100 mm (vertical) ->", font=f_small, fill=0)

    check(np.array(img, dtype=np.uint8) * 255)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    img.save(OUT, "PDF", resolution=DPI)
    img.save(OUT.with_suffix(".preview.png"))
    print(f"wrote {OUT.relative_to(REPO)}  ({PAGE_W / MM:.2f} x {PAGE_H / MM:.2f} mm page, "
          f"{OUT.stat().st_size / 1024:.0f} KiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
