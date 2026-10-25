"""Builds the Final Presentation deck, in the style of the First Presentation.

    python3 ProjectReport/presentation/build_final_deck.py            # the deck
    python3 ProjectReport/presentation/build_final_deck.py --video D  # plus D/video_slides.pptx

The deck goes to ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-FinalPresentation.pptx.
--video also writes a second deck holding each slide's build stages (one block of
the pipeline lit at a time, the problem drawn in three steps) without page
numbers, for rendering into the video's frames. Every number on a slide is from
the final report or the run it names, and the slide's source line says which.
Run from the repository root.
"""
import argparse
import os

from PIL import Image
from pptx import Presentation
from pptx.chart.data import CategoryChartData
from pptx.dml.color import RGBColor
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION, XL_LEGEND_POSITION
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.ns import qn
from pptx.util import Emu, Inches, Pt

# The First Presentation's palette and type.
INK, SLATE, MUTED = "16222C", "3E4A54", "5E6973"
TEAL, MINT, TEAL_LIGHT = "0E736B", "6CCABD", "DCEBE8"
BG, CARD, LINE, RULE, DARK = "F4F2EC", "FBFAF6", "D3D6D2", "C3C1BC", "14202B"
RED, RED_LIGHT, YELLOW = "B5473A", "F3DDD8", "E8B33A"
HEAD, BODY, MONO = "Space Grotesk", "IBM Plex Sans", "IBM Plex Mono"

W, H = 13.333, 7.5
L, R = 0.89, 0.89
CW = W - L - R
IMG = "ProjectReport/presentation/deck_images"
PHOTO = "assets/photos"
TOTAL = 18


def rgb(h):
    return RGBColor.from_string(h)


class Deck:
    def __init__(self, video=False):
        self.prs = Presentation()
        self.prs.slide_width, self.prs.slide_height = Inches(W), Inches(H)
        self.blank = self.prs.slide_layouts[6]
        self.video = video
        self.n = 0

    # ---- primitives -------------------------------------------------------
    def slide(self, dark=False):
        s = self.prs.slides.add_slide(self.blank)
        fill = s.background.fill
        fill.solid()
        fill.fore_color.rgb = rgb(DARK if dark else BG)
        return s

    def text(self, s, x, y, w, h, runs, size=15, color=SLATE, font=BODY, bold=False,
             align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, spacing=None, name=None):
        """runs: a string, or a list of paragraphs, each a string or a list of
        (text, {overrides}) pairs."""
        tb = s.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
        if name:
            tb.name = name
        tf = tb.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
        tf.vertical_anchor = anchor
        paras = [runs] if isinstance(runs, str) else runs
        for i, para in enumerate(paras):
            p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
            p.alignment = align
            if spacing:
                p.space_after = Pt(spacing)
            parts = [(para, {})] if isinstance(para, str) else para
            for t, o in parts:
                r = p.add_run()
                r.text = t
                f = r.font
                f.name = o.get("font", font)
                f.size = Pt(o.get("size", size))
                f.bold = o.get("bold", bold)
                f.italic = o.get("italic", False)
                f.color.rgb = rgb(o.get("color", color))
        return tb

    def box(self, s, x, y, w, h, fill=CARD, line=LINE, shape=MSO_SHAPE.ROUNDED_RECTANGLE,
            dash=False, weight=0.75, radius=0.06):
        b = s.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
        if shape == MSO_SHAPE.ROUNDED_RECTANGLE:
            b.adjustments[0] = radius
        if fill is None:
            b.fill.background()
        else:
            b.fill.solid()
            b.fill.fore_color.rgb = rgb(fill)
        if line is None:
            b.line.fill.background()
        else:
            b.line.color.rgb = rgb(line)
            b.line.width = Pt(weight)
            if dash:
                b.line.dash_style = 4  # dash
        b.shadow.inherit = False
        b.text_frame.text = ""
        return b

    def line(self, s, x1, y1, x2, y2, color=SLATE, weight=1.0, dash=False, arrow=False):
        c = s.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y1), Inches(x2), Inches(y2))
        c.line.color.rgb = rgb(color)
        c.line.width = Pt(weight)
        if dash:
            c.line.dash_style = 4
        if arrow:
            ln = c.line._get_or_add_ln()
            tail = ln.makeelement(qn("a:tailEnd"), {"type": "triangle", "w": "med", "len": "med"})
            ln.append(tail)
        return c

    def image(self, s, path, x, y, w=None, h=None):
        """Places the picture inside the box, keeping its aspect; returns its extent."""
        iw, ih = Image.open(path).size
        if w and h:
            if iw / ih > w / h:
                h2 = w * ih / iw
                y, h = y + (h - h2) / 2, h2
            else:
                w2 = h * iw / ih
                x, w = x + (w - w2) / 2, w2
        elif w:
            h = w * ih / iw
        else:
            w = h * iw / ih
        s.shapes.add_picture(path, Inches(x), Inches(y), Inches(w), Inches(h))
        return x, y, w, h

    def image_fill(self, s, path, x, y, w, h, focus=(0.5, 0.5)):
        """Fills the box, cropping the picture."""
        iw, ih = Image.open(path).size
        pic = s.shapes.add_picture(path, Inches(x), Inches(y), Inches(w), Inches(h))
        if iw / ih > w / h:
            keep = (w / h) / (iw / ih)
            cut = 1 - keep
            pic.crop_left, pic.crop_right = cut * focus[0], cut * (1 - focus[0])
        else:
            keep = (iw / ih) / (w / h)
            cut = 1 - keep
            pic.crop_top, pic.crop_bottom = cut * focus[1], cut * (1 - focus[1])
        return pic

    def frame(self, s, kicker, title, source=None):
        """Kicker, title, source line, footer and page number of a content slide."""
        self.n += 1
        self.text(s, L, 0.62, CW, 0.26, kicker.upper(), size=12, color=TEAL, font=MONO)
        self.text(s, L, 0.98, CW, 0.62, title, size=32, color=INK, font=HEAD, bold=True)
        if source:
            self.text(s, L, 6.52, CW, 0.24, "Source: " + source, size=10, color=MUTED)
        self.text(s, L, 6.9, 6.0, 0.26, "The Eyes · Group 12 · Final Presentation", size=12, color=MUTED)
        if not self.video:
            self.text(s, W - R - 1.2, 6.9, 1.2, 0.26, f"{self.n} / {TOTAL}", size=12, color=MUTED,
                      font=MONO, align=PP_ALIGN.RIGHT)

    def card(self, s, x, y, w, h, head, body, fill=CARD, line=LINE, head_color=INK, body_size=14,
             head_size=18, num=None):
        self.box(s, x, y, w, h, fill=fill, line=line)
        hx = x + 0.2
        if num is not None:
            self.text(s, hx, y + 0.16, 0.4, 0.34, str(num), size=head_size, color=TEAL, font=HEAD, bold=True)
            hx += 0.34
        self.text(s, hx, y + 0.16, w - (hx - x) - 0.2, 0.34, head, size=head_size, color=head_color,
                  font=HEAD, bold=True)
        self.text(s, x + 0.2, y + 0.16 + head_size / 72 * 1.45, w - 0.4, h - 0.6, body, size=body_size,
                  color=SLATE)

    def stat(self, s, x, y, w, big, small, color=TEAL, big_size=40):
        self.text(s, x, y, w, 0.7, big, size=big_size, color=color, font=HEAD, bold=True)
        self.text(s, x, y + big_size / 72 * 1.15, w, 0.7, small, size=13, color=SLATE)

    def table(self, s, x, y, w, rows, col_w, head_fill=TEAL_LIGHT, size=13, row_h=0.36, bold_last=False,
              highlight=None):
        t = s.shapes.add_table(len(rows), len(rows[0]), Inches(x), Inches(y), Inches(w),
                               Inches(row_h * len(rows))).table
        tbl_pr = t._tbl.tblPr
        style = tbl_pr.find(qn("a:tableStyleId"))
        if style is not None:
            style.text = "{5940675A-B579-460E-94D1-54222C63F5DA}"  # no style, table grid
        for j, cw in enumerate(col_w):
            t.columns[j].width = Inches(cw)
        for i, row in enumerate(rows):
            t.rows[i].height = Inches(row_h)
            for j, val in enumerate(row):
                c = t.cell(i, j)
                c.margin_left = c.margin_right = Inches(0.08)
                c.margin_top = c.margin_bottom = Inches(0.03)
                c.vertical_anchor = MSO_ANCHOR.MIDDLE
                c.fill.solid()
                hl = highlight and (i, j) in highlight
                c.fill.fore_color.rgb = rgb(head_fill if i == 0 else (TEAL_LIGHT if hl else CARD))
                tf = c.text_frame
                tf.word_wrap = True
                p = tf.paragraphs[0]
                p.alignment = PP_ALIGN.LEFT if j == 0 else PP_ALIGN.RIGHT
                r = p.add_run()
                r.text = str(val)
                r.font.name = BODY
                r.font.size = Pt(size)
                r.font.bold = i == 0 or (bold_last and i == len(rows) - 1) or bool(hl)
                r.font.color.rgb = rgb(INK if i == 0 else SLATE)
        return t

    def bars(self, s, x, y, w, h, cats, series, colors, title=None, fmt='0.00', max_v=None, legend=True,
             size=12):
        cd = CategoryChartData()
        cd.categories = cats
        for name, vals in series:
            cd.add_series(name, vals)
        gf = s.shapes.add_chart(XL_CHART_TYPE.COLUMN_CLUSTERED, Inches(x), Inches(y), Inches(w), Inches(h), cd)
        ch = gf.chart
        ch.font.name = BODY
        ch.font.size = Pt(size)
        ch.font.color.rgb = rgb(SLATE)
        ch.has_title = bool(title)
        if title:
            ch.chart_title.text_frame.text = title
            tp = ch.chart_title.text_frame.paragraphs[0]
            tp.runs[0].font.size = Pt(13)
            tp.runs[0].font.bold = True
            tp.runs[0].font.name = BODY
            tp.runs[0].font.color.rgb = rgb(INK)
        ch.has_legend = legend and len(series) > 1
        if ch.has_legend:
            ch.legend.position = XL_LEGEND_POSITION.BOTTOM
            ch.legend.include_in_layout = False
            ch.legend.font.size = Pt(size)
        va = ch.value_axis
        va.has_major_gridlines = True
        va.major_gridlines.format.line.color.rgb = rgb(LINE)
        va.format.line.fill.background()
        va.tick_labels.font.size = Pt(size)
        va.tick_labels.font.color.rgb = rgb(MUTED)
        if max_v is not None:
            va.maximum_scale = max_v
            va.minimum_scale = 0
        ca = ch.category_axis
        ca.format.line.color.rgb = rgb(RULE)
        ca.tick_labels.font.size = Pt(size)
        ca.tick_labels.font.color.rgb = rgb(SLATE)
        plot = ch.plots[0]
        plot.gap_width = 60
        plot.overlap = -10
        plot.has_data_labels = True
        dl = plot.data_labels
        dl.number_format = fmt
        dl.number_format_is_linked = False
        dl.position = XL_LABEL_POSITION.OUTSIDE_END
        dl.font.size = Pt(size - 1)
        dl.font.color.rgb = rgb(SLATE)
        for ser, col in zip(plot.series, colors):
            ser.format.fill.solid()
            ser.format.fill.fore_color.rgb = rgb(col)
            ser.format.line.fill.background()
        return ch

    def badge(self, s, x, y, n, d=0.38):
        b = self.box(s, x, y, d, d, fill=TEAL, line="FFFFFF", shape=MSO_SHAPE.OVAL, weight=1.5)
        tf = b.text_frame
        tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
        p = tf.paragraphs[0]
        p.alignment = PP_ALIGN.CENTER
        r = p.add_run()
        r.text = str(n)
        r.font.name, r.font.size, r.font.bold = HEAD, Pt(15), True
        r.font.color.rgb = rgb("FFFFFF")
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE

    def save(self, path):
        self.prs.save(path)


# ---- the slides ----------------------------------------------------------
def s_title(d):
    s = d.slide(dark=True)
    d.n += 1
    d.image_fill(s, f"{PHOTO}/rig_overview_20260929.jpg", 7.0, 0, W - 7.0, H, focus=(0.5, 0.5))
    d.text(s, 7.25, 6.78, 5.8, 0.5, "The rig: two RealSense D435i on tripods, the printed objects in their marker trays",
           size=11, color="FFFFFF")
    d.text(s, L, 1.55, 5.8, 0.3, "PRS PRACTICE MODULE · FINAL PRESENTATION", size=12, color=MINT, font=MONO)
    d.text(s, L, 2.0, 6.0, 1.2, "The Eyes", size=66, color="FFFFFF", font=HEAD, bold=True)
    d.text(s, L, 3.25, 5.7, 1.2, "Multi-view perception and scene understanding for robot manipulation",
           size=24, color="C8D3D8", font=HEAD)
    d.text(s, L, 4.75, 5.8, 0.9, ["Group 12 · AIS08 · Chan Wai Jye (solo)", "31 October 2026"], size=16,
           color="C8D3D8", spacing=4)
    d.text(s, L, 6.2, 5.8, 0.5, "Code built with Claude Code under the author's design and review",
           size=12, color="8A97A0")


def s_problem(d, stage=3):
    s = d.slide()
    d.frame(s, "01 · The problem", "Why one camera is not enough")
    # A top view of the table: one camera, an occluder, an object behind it.
    tx, ty, tw, th = L, 1.9, 5.7, 4.4
    d.box(s, tx, ty, tw, th, fill="EEEBE2", line=RULE, radius=0.03)
    d.text(s, tx + 0.15, ty + 0.1, 3, 0.25, "the table, from above", size=11, color=MUTED, font=MONO)
    # camera 1 bottom left, the cube up and to the right of it, the box between them;
    # camera 2, bottom right, sees past the box's end.
    occ = (tx + 1.65, ty + 2.35, 1.2, 0.16)
    d.box(s, *occ, fill=INK, line=None, shape=MSO_SHAPE.RECTANGLE)
    d.text(s, occ[0] - 0.75, occ[1] - 0.05, 0.7, 0.25, "a box", size=11, color=MUTED, align=PP_ALIGN.RIGHT)
    cube = (tx + 2.0, ty + 1.2, 0.5, 0.5)
    seen = stage >= 2
    d.box(s, *cube, fill=RED if seen else None, line=RED, shape=MSO_SHAPE.RECTANGLE, dash=not seen, weight=1.5)
    d.text(s, cube[0] + 0.62, cube[1] + 0.12, 1.4, 0.3, "the cube", size=13, color=RED if seen else MUTED)
    c1 = (tx + 1.0, ty + th - 0.55)
    d.box(s, c1[0] - 0.22, c1[1], 0.44, 0.3, fill=TEAL, line=None, shape=MSO_SHAPE.RECTANGLE)
    d.text(s, c1[0] - 0.6, c1[1] + 0.35, 1.6, 0.25, "camera 1", size=12, color=TEAL, bold=True)
    for ex in (occ[0] + 0.05, occ[0] + occ[2] - 0.05):
        d.line(s, c1[0], c1[1], ex, occ[1] + 0.16, color=TEAL, weight=1, dash=True)
    if stage == 1:
        d.text(s, tx + 3.0, ty + 3.1, 2.5, 0.6, "camera 1 sees the box, and nothing behind it", size=13,
               color=SLATE)
    if stage >= 2:
        c2 = (tx + tw - 0.9, ty + th - 0.55)
        d.box(s, c2[0] - 0.22, c2[1], 0.44, 0.3, fill=TEAL, line=None, shape=MSO_SHAPE.RECTANGLE)
        d.text(s, c2[0] - 0.9, c2[1] + 0.35, 1.6, 0.25, "camera 2", size=12, color=TEAL, bold=True,
               align=PP_ALIGN.RIGHT)
        d.line(s, c2[0], c2[1], cube[0] + 0.5, cube[1] + 0.5, color=TEAL, weight=1.25)
        # an object both see: two estimates that do not quite meet
        bx, by = tx + 3.9, ty + 0.75
        d.box(s, bx, by, 0.55, 0.55, fill=YELLOW, line=None, shape=MSO_SHAPE.OVAL)
        for (ox, oy) in ((bx + 0.12, by + 0.05), (bx + 0.3, by + 0.22)):
            d.box(s, ox, oy, 0.16, 0.16, fill=None, line=INK, shape=MSO_SHAPE.OVAL, weight=1.5)
        d.text(s, bx - 0.45, by + 0.65, 1.95, 0.6, "seen by both: two estimates that never quite agree",
               size=12, color=SLATE)
    # right column
    rx, rw = L + 6.2, CW - 6.2
    d.text(s, rx, 1.9, rw, 1.2,
           [[("A planner works with statements, like ", {}), ("(on cube table)", {"font": MONO, "size": 17}),
             (". A real robot has to get them from cameras.", {})]],
           size=20, color=INK, font=HEAD)
    if stage >= 3:
        qs = [("Association", "Which estimates, one per camera, are the same object?"),
              ("Arbitration", "When two views disagree, which one is wrong?"),
              ("Symbolisation", "How do poses become facts reliable enough to plan with?")]
        for i, (h, b) in enumerate(qs):
            d.card(s, rx, 3.0 + i * 1.17, rw, 1.1, h, b, num=i + 1, body_size=14)


def s_rig(d):
    s = d.slide()
    d.frame(s, "02 · The system", "The rig", source="config/extrinsics.yaml; frames.skew_ms of every scored scene and take")
    d.image(s, f"{PHOTO}/rig_trays_charuco_20260929.jpg", L, 1.85, 6.6, 4.5)
    rx, rw = L + 6.95, CW - 6.95
    items = [("2 × Intel RealSense D435i", "Active IR stereo depth plus colour, on tripods, each selected by its serial number"),
             ("About 0.74 m apart, 80° between views", "Frames paired by timestamp, never more than 16.7 ms apart"),
             ("Calibrated to about 0.3 px", "Against the ChArUco board, and registered again at the start of every session"),
             ("Three printed objects", "Printed from the pose estimator's own meshes; the grey marker trays are for ground truth only")]
    for i, (h, b) in enumerate(items):
        d.card(s, rx, 1.85 + i * 1.14, rw, 1.02, h, b, head_size=16, body_size=13)


PIPE_VIEW = [("2 × D435i", "RGB-D frames, 30 a second"),
             ("RGB-D YOLOv8n", "4-channel input; trained here on 40,012 synthetic frames, fine-tuned on 360 real"),
             ("FoundationPose", "6-DoF pose from the CAD mesh; pretrained, frozen"),
             ("World frame", "Each pose moved into the shared table frame")]
PIPE_PAIR = [("Depth check ★", "Does the other camera see through this pose?"),
             ("Association ★", "The same object, matched across the views"),
             ("Fusion ★", "Gate at 50 mm / 30°; confidence weighting; an EKF per object"),
             ("Scene graph ★", "on, in, clear, occluded_from; the support check"),
             ("Outputs", "PDDL (:init …) facts and ObjectState")]


def s_pipeline(d, lit=None):
    s = d.slide()
    d.frame(s, "02 · The system", "One C++ process, two views")
    d.text(s, L, 1.85, CW, 0.26, "PER VIEW · once for each camera", size=12, color=MUTED, font=MONO)
    d.text(s, L + 0.55, 4.02, CW - 0.55, 0.26, "PER FRAME PAIR · the project's core, written from first principles in C++",
           size=12, color=TEAL, font=MONO)
    k = 0
    for row, items, y, base in ((0, PIPE_VIEW, 2.2, CARD), (1, PIPE_PAIR, 4.37, TEAL_LIGHT)):
        n = len(items)
        gap = 0.3
        bw = (CW - gap * (n - 1)) / n
        for i, (h, b) in enumerate(items):
            x = L + i * (bw + gap)
            on = lit == k
            fill = TEAL if on else base
            d.box(s, x, y, bw, 1.45, fill=fill, line=TEAL if (on or row == 1) else LINE)
            d.text(s, x + 0.16, y + 0.14, bw - 0.3, 0.35, h, size=16, color="FFFFFF" if on else INK, font=HEAD, bold=True)
            d.text(s, x + 0.16, y + 0.52, bw - 0.3, 0.9, b, size=12, color="FFFFFF" if on else SLATE)
            if i < n - 1:
                d.line(s, x + bw + 0.03, y + 0.72, x + bw + gap - 0.03, y + 0.72, color=MUTED, weight=1.25, arrow=True)
            k += 1
    # world frame -> depth check, routed between the rows, clear of the row label
    bw0 = (CW - 0.3 * 3) / 4
    xw, ym = L + 3 * (bw0 + 0.3) + bw0 / 2, 3.85
    d.line(s, xw, 2.2 + 1.45, xw, ym, color=MUTED, weight=1.25)
    d.line(s, xw, ym, L + 0.35, ym, color=MUTED, weight=1.25)
    d.line(s, L + 0.35, ym, L + 0.35, 4.37, color=MUTED, weight=1.25, arrow=True)
    d.text(s, L, 6.05, CW, 0.4,
           "★ No sensor-fusion, pose-averaging or scene-graph library: Eigen for linear algebra only. "
           "The code was written with Claude Code under the author's design and review.",
           size=12, color=MUTED)


def s_console(d):
    s = d.slide()
    d.frame(s, "02 · The system", "What the console shows",
            source="the live take demo_live_d, 25 Oct 2026, at 60 s: the cube is in the bowl")
    x, y, w, h = d.image(s, f"{IMG}/live_d_60s_console.jpg", L, 1.8, 7.9, 4.6)
    spots = [(0.02, 0.02), (0.02, 0.49), (0.51, 0.49), (0.76, 0.49)]
    for i, (fx, fy) in enumerate(spots):
        d.badge(s, x + fx * w, y + fy * h, i + 1)
    rx, rw = x + w + 0.35, W - R - (x + w + 0.35)
    items = [("Both camera views", "Boxes from the detector, axes from the pose estimator, per camera"),
             ("The table from above", "The fused objects in the shared frame, and the two cameras"),
             ("The scene graph", "Each object and what it rests on or sits in"),
             ("The facts", "The (:init …) block a planner reads")]
    for i, (hd, b) in enumerate(items):
        yy = 1.8 + i * 1.17
        d.badge(s, rx, yy + 0.02, i + 1)
        d.text(s, rx + 0.5, yy, rw - 0.5, 0.32, hd, size=16, color=INK, font=HEAD, bold=True)
        d.text(s, rx + 0.5, yy + 0.36, rw - 0.5, 0.75, b, size=13, color=SLATE)


def s_live(d):
    s = d.slide()
    d.frame(s, "03 · Live", "Live on the rig",
            source="demo_live_d, 25 Oct 2026, its metrics.sqlite; final report §9.8")
    x, y, w, h = d.image(s, f"{IMG}/live_d_84s_console.jpg", L, 1.8, 7.9, 4.2)
    d.text(s, x, y + h + 0.06, w, 0.45,
           "LIVE, one take, not replayed. At 84 s the card hides the cube from cam_over: cam_front 3, cam_over 2, fused 3.",
           size=11, color=MUTED)
    rx, rw = x + w + 0.4, W - R - (x + w + 0.4)
    d.stat(s, rx, 1.75, rw, "13.3", "frame pairs a second over the 195 s take")
    d.stat(s, rx, 2.95, rw, "10", "a second with three objects tracked in each view")
    d.stat(s, rx, 4.15, rw, "79%", "of a three-object pair is the pose estimator")
    d.stat(s, rx, 5.35, rw, "1.1 s", "per new object per view, while the pose estimator registers it", big_size=32)


def s_ground_truth(d):
    s = d.slide()
    d.frame(s, "04 · Evaluation", "Ground truth from printed marker trays",
            source="final report §7–8; ground truth across cameras 0.3–1.4 mm (docs/OCTOBER_PLAN.md); config/meshes.yaml")
    iw = (CW - 0.3) / 2
    for i, (f, cap) in enumerate(((f"{IMG}/t1_cube_gt_still.jpg", "A ground-truth still: the cube in its tray, with the tray's markers"),
                                  (f"{IMG}/t1_cube_gt_overlay.jpg", "The pose the markers give, drawn back onto the frame (green)"))):
        xx = L + i * (iw + 0.3)
        _, yy, _, hh = d.image(s, f, xx, 1.8, iw, 3.2)
        d.text(s, xx, yy + hh + 0.06, iw, 0.3, cap, size=12, color=MUTED)
    cw3 = (CW - 0.6) / 3
    cards = [("Within 1.5 mm", "The two cameras agree on each tray's true pose"),
             ("Recorded once", "300 frame pairs a scene, replayed for every method"),
             ("Under 10% of the diameter", "ADD-S, the surface-to-surface error, counts as right: 8.7 mm for the cube")]
    for i, (h, b) in enumerate(cards):
        d.card(s, L + i * (cw3 + 0.3), 5.4, cw3, 1.0, h, b, head_size=16, body_size=13)


def s_scenes(d):
    s = d.slide()
    d.frame(s, "04 · Evaluation", "Sixteen scenes, four levels of occlusion",
            source="final report §8, §9.4; t2_in_a and t2_in_b, 10 Oct")
    tiers = [("T1", "Alone", "5 scenes", "A cube or the bottle, nothing else"),
             ("T2", "In clutter", "3 scenes", "Three objects among distractors"),
             ("T3", "Half hidden", "4 scenes", "From one camera; one scene's level not certified"),
             ("T4", "Fully hidden", "4 scenes", "From one camera, which sees 0–6% of it")]
    cw4 = (CW - 0.9) / 4
    for i, (t, h, n, b) in enumerate(tiers):
        x = L + i * (cw4 + 0.3)
        d.box(s, x, 1.85, cw4, 2.15, fill=TEAL_LIGHT if i == 3 else CARD, line=TEAL if i == 3 else LINE)
        d.text(s, x + 0.2, 1.98, cw4 - 0.4, 0.5, t, size=30, color=TEAL, font=HEAD, bold=True)
        d.text(s, x + 0.2, 2.6, cw4 - 0.4, 0.35, h, size=18, color=INK, font=HEAD, bold=True)
        d.text(s, x + 0.2, 2.97, cw4 - 0.4, 0.3, n, size=13, color=TEAL, font=MONO)
        d.text(s, x + 0.2, 3.3, cw4 - 0.4, 0.65, b, size=13, color=SLATE)
    _, yy, ww, hh = d.image(s, f"{IMG}/t2_in_a_cam_over.jpg", L, 4.3, 3.6, 2.1)
    tx = L + ww + 0.35
    d.text(s, tx, 4.35, CW - ww - 0.35, 0.4, "Two more with the cube inside the bowl", size=18, color=INK,
           font=HEAD, bold=True)
    d.text(s, tx, 4.8, CW - ww - 0.35, 1.5,
           [[("(in red_cube bowl)", {"font": MONO}), (" is reported on 300 of 300 frame pairs in both.", {})],
            "Every method runs on the same recorded frames. Intervals are 95% bootstrap over scenes; a difference "
            "counts only if it is beyond the 2–4 mm sensor floor."],
           size=14, color=SLATE, spacing=8)


def s_hidden(d):
    s = d.slide()
    d.frame(s, "05 · Results", "A hidden object is still found",
            source="final report §9.4; the share each blocked camera sees from runs/scored/*/visibility.csv")
    x, y, w, h = d.image(s, f"{IMG}/t4_cube_over_views.jpg", L, 1.8, CW, 2.35)
    d.text(s, x, y + h + 0.06, w, 0.3, "t4_cube_over: cam_front (left) sees the cube; from cam_over (right) the card hides it",
           size=12, color=MUTED)
    rows = [["Scene", "Blocked camera sees", "It finds it on", "Fused finds it on", "Fused ADD-S"],
            ["t4_cube_over", "0%", "0 / 300", "300 / 300", "3.08 mm"],
            ["t4_cube_front", "0%", "0 / 300", "300 / 300", "1.65 mm"],
            ["t4_bottle_over", "6%", "0 / 300", "300 / 300", "3.91 mm"],
            ["t4_bottle_over_b", "2%", "0 / 300", "300 / 300", "2.85 mm"]]
    d.table(s, L, 4.55, 6.9, rows, [1.75, 1.3, 1.1, 1.35, 1.4], size=12, row_h=0.37,
            highlight={(i, 3) for i in range(1, 5)})
    rx, rw = L + 7.25, CW - 7.25
    d.stat(s, rx, 4.45, rw, "+1.00", "recall, fused against the blocked camera, 95% interval [1.00, 1.00]")
    d.text(s, rx, 5.65, rw, 0.8,
           "A single camera does as well only when it is the open one. No single camera covers all four scenes.",
           size=13, color=SLATE)


def s_combine(d):
    s = d.slide()
    d.frame(s, "05 · Results", "How the views are combined matters",
            source="runs/scored/20261005/t1_cube*/ablation.log (all arms with the EKF); final report §9.4, docs/results/ablation_ci.json")
    d.bars(s, L, 1.75, 6.9, 4.7, ["t1_cube", "t1_cube_b"],
           [("cam_front", (2.62, 3.34)), ("cam_over", (2.82, 2.28)), ("plain average", (26.19, 7.46)),
            ("fused", (1.54, 1.72))],
           [RULE, "A9B4B9", RED, TEAL], title="The cube alone: ADD-S, mm (lower is better)", fmt="0.00")
    rx, rw = L + 7.2, CW - 7.2
    d.card(s, rx, 1.85, rw, 1.45, "Averaging fails",
           "The cube looks the same from several directions; the cameras pick different ones, and their average is a rotation neither saw.",
           head_size=16, body_size=13, line=RED)
    d.card(s, rx, 3.45, rw, 1.3, "Gated vs plain average",
           "All 16 scenes: −9.81 mm [−16.96, −3.75], beyond the sensor floor.", head_size=16, body_size=13)
    d.card(s, rx, 4.9, rw, 1.5, "Fused vs the better camera",
           "All 16: 2.76 against 3.19 mm, a difference of −0.36 [−0.83, 0.08]. Not resolvable on this rig.",
           head_size=16, body_size=13)


def s_wrong_camera(d):
    s = d.slide()
    d.frame(s, "05 · Results", "When fusion believed the wrong camera",
            source="final report §9.4–9.5; t3_bottle_over_b and t3_bottle_over; refusals over all 22 scenes, docs/OCTOBER_PLAN.md, 17 Oct")
    tx, ty, tw, th = L, 1.85, 6.2, 4.5
    d.box(s, tx, ty, tw, th, fill="EEEBE2", line=RULE, radius=0.03)
    d.text(s, tx + 0.15, ty + 0.1, 3, 0.25, "the table, from above", size=11, color=MUTED, font=MONO)
    bottle = (tx + 2.0, ty + 0.85)          # the true bottle, top-left of centre
    cf = (tx + 0.75, ty + th - 0.5)         # cam_front, bottom left
    co = (tx + tw - 0.75, ty + th - 0.5)    # cam_over, bottom right
    # the cover, between cam_over and the bottle
    cov = (tx + 2.55, ty + 1.75, 1.1, 0.14)
    d.box(s, *cov, fill=INK, line=None, shape=MSO_SHAPE.RECTANGLE)
    d.text(s, cov[0] + 1.2, cov[1] - 0.08, 1.2, 0.3, "the cover", size=11, color=MUTED)
    d.box(s, bottle[0] - 0.3, bottle[1] - 0.3, 0.6, 0.6, fill=YELLOW, line=None, shape=MSO_SHAPE.OVAL)
    d.text(s, bottle[0] + 0.4, bottle[1] - 0.12, 1.4, 0.3, "the bottle", size=12, color=INK, bold=True)
    # where the half-hidden view put it: on cam_over's ray, at the cover
    claim = (tx + 3.05, ty + 2.15)
    d.box(s, claim[0] - 0.3, claim[1] - 0.3, 0.6, 0.6, fill=None, line=RED, shape=MSO_SHAPE.OVAL, dash=True, weight=2)
    d.text(s, tx + 0.3, claim[1] - 0.5, 2.15, 0.6, "where cam_over put it: score 0.99, 70–86 mm off",
           size=12, color=RED, align=PP_ALIGN.RIGHT)
    for (c, name, al) in ((cf, "cam_front", PP_ALIGN.LEFT), (co, "cam_over", PP_ALIGN.RIGHT)):
        d.box(s, c[0] - 0.22, c[1] - 0.1, 0.44, 0.3, fill=TEAL, line=None, shape=MSO_SHAPE.RECTANGLE)
        d.text(s, c[0] - (0.25 if al == PP_ALIGN.LEFT else 1.35), c[1] + 0.25, 1.6, 0.25, name, size=12, color=TEAL,
               bold=True, align=al)
    d.line(s, co[0], co[1] - 0.1, cov[0] + 0.6, cov[1] + 0.14, color=TEAL, weight=1, dash=True)
    # cam_front's line of sight through the claimed spot, on to what it measures behind it
    far = (claim[0] + (claim[0] - cf[0]) * 0.5, claim[1] + (claim[1] - cf[1]) * 0.5)
    d.line(s, cf[0], cf[1] - 0.1, far[0], far[1], color=TEAL, weight=2)
    d.box(s, far[0] - 0.07, far[1] - 0.07, 0.14, 0.14, fill=TEAL, line=None, shape=MSO_SHAPE.OVAL)
    d.text(s, far[0] + 0.15, far[1] - 0.2, tx + tw - far[0] - 0.3, 0.6, "cam_front measures a surface here",
           size=12, color=TEAL)
    d.text(s, tx + 1.4, ty + th - 0.75, 3.4, 0.6,
           "cam_front sees past the claimed spot, so nothing is there: the estimate is vetoed",
           size=12, color=SLATE, align=PP_ALIGN.CENTER)
    rx, rw = L + 6.55, CW - 6.55
    d.text(s, rx, 1.85, rw, 0.35, "Keep the higher score", size=18, color=INK, font=HEAD, bold=True)
    d.text(s, rx, 2.25, rw, 0.6, [[("70.75", {"color": RED, "bold": True, "font": HEAD, "size": 30}),
                                   ("  and  ", {}), ("83.72 mm", {"color": RED, "bold": True, "font": HEAD, "size": 30})]],
           size=16, color=SLATE)
    d.text(s, rx, 2.9, rw, 0.4, "the good camera alone: 3.06 and 3.14 mm", size=13, color=MUTED)
    d.text(s, rx, 3.5, rw, 0.35, "Veto by the other camera's depth", size=18, color=INK, font=HEAD, bold=True)
    d.text(s, rx, 3.9, rw, 0.6, [[("3.03", {"color": TEAL, "bold": True, "font": HEAD, "size": 30}),
                                  ("  and  ", {}), ("3.12 mm", {"color": TEAL, "bold": True, "font": HEAD, "size": 30})]],
           size=16, color=SLATE)
    d.card(s, rx, 4.85, rw, 1.5, "1,035 estimates vetoed",
           "Over all 22 recordings. One of them was within 30 mm of the truth, of 17,121 that were.",
           head_size=16, body_size=13)


def s_not_there(d):
    s = d.slide()
    d.frame(s, "05 · Results", "Objects that are not there",
            source="final report §9.2, §9.7; replays of 20261005/t3_cube_front with each detector")
    iw = (CW - 0.3) / 2
    for i, (f, cap) in enumerate(((f"{IMG}/synthetic_t3_cube_front_5s_cam_front.jpg", "Synthetic-only detector, cam_front: a cube boxed on the card's red print"),
                                  (f"{IMG}/replay_t3_cube_front_5s_cam_front.jpg", "Fine-tuned detector, the same frame: no false object"))):
        xx = L + i * (iw + 0.3)
        d.image_fill(s, f, xx, 1.8, iw, 2.65, focus=(0.5, 0.3))
        d.text(s, xx, 4.5, iw, 0.3, cap, size=12, color=MUTED)
    d.card(s, L, 4.95, iw, 1.45, "The support check: 2,771 → 1",
           "An object is reported only if something supports it. Seven scenes: false object-pairs 2,771 → 1, all 2,100 real ones kept.",
           head_size=16, body_size=13)
    d.card(s, L + iw + 0.3, 4.95, iw, 1.45, "Fine-tuning: 731 → 0",
           "On 360 real frames of ten unscored layouts. mAP@0.5:0.95 0.670 → 0.934; false objects over 22 recordings 731 → 0.",
           head_size=16, body_size=13, fill=TEAL_LIGHT, line=TEAL)


def s_predicates(d):
    s = d.slide()
    d.frame(s, "05 · Results", "Facts checked against hand labels",
            source="final report §9.7; scripts/predicate_accuracy.py, docs/predicate_labels.yaml")
    d.bars(s, L, 1.75, 6.9, 4.7, ["on", "clear", "in", "occluded_from"],
           [("precision", (0.994, 0.995, 1.000, 0.557)), ("recall", (1.0, 1.0, 1.0, 1.0))],
           [TEAL, MINT], title="22 recordings, 6,600 frame pairs; target 0.95", fmt="0.000", max_v=1.1)
    rx, rw = L + 7.2, CW - 7.2
    d.card(s, rx, 1.85, rw, 1.55, "on, in, clear: met",
           "Precision 0.994 to 1.000, recall 1.000. The few false on and clear facts are one duplicate bottle in one dim scene.",
           head_size=16, body_size=13)
    d.card(s, rx, 3.55, rw, 2.85, "occluded_from: 0.557",
           "A half-hidden object (44–68% visible) reads as hidden from the camera whose estimate was refused. "
           "The definition fixed in advance is under 25% visible. And the fact cannot tell a blocked camera from a "
           "detector that missed.",
           head_size=16, body_size=13, line=RED)


def s_limits(d):
    s = d.slide()
    d.frame(s, "06 · Limits", "What it still gets wrong",
            source="final report §9.4, §9.7, §9.8, §11; the live takes of 25 Oct")
    d.image_fill(s, f"{IMG}/replay_t2_clutter_c_5s_cam_over.jpg", L, 1.8, 4.4, 2.75, focus=(0.5, 0.3))
    d.text(s, L, 4.62, 4.4, 0.7, "t2_clutter_c from cam_over: the cube is small and far away. Fused, it is inside the threshold on 75.7% of frames",
           size=12, color=MUTED)
    rx, rw = L + 4.75, CW - 4.75
    items = [("occluded_from", "A half-hidden object reads as hidden; precision 0.557"),
             ("A hand in view", "A pose can register on the hand; a passing hand can start a duplicate track for one frame"),
             ("Registration stalls", "About 1.1 s per new object per view, with nothing processed meanwhile"),
             ("One rig, one room", "Still scenes for every accuracy figure; four distractors, not eight")]
    for i, (h, b) in enumerate(items):
        y = 1.85 + i * 1.15
        d.card(s, rx, y, rw, 1.02, h, b, head_size=16, body_size=13)


def s_depth(d):
    s = d.slide()
    d.frame(s, "06 · Findings", "On real frames, depth is what finds the cube",
            source="final report §9.2, §9.9; 9,988 synthetic validation frames, 960 real frames of the 16 scored scenes")
    d.bars(s, L, 1.75, 6.9, 4.7, ["synthetic validation", "real, 16 scenes"],
           [("RGB only", (0.957, 0.501)), ("RGB-D", (0.957, 0.670)), ("RGB-D, fine-tuned", (0.945, 0.934))],
           [RULE, MINT, TEAL], title="Detector mAP@0.5:0.95", fmt="0.000", max_v=1.1)
    rx, rw = L + 7.2, CW - 7.2
    d.stat(s, rx, 1.8, rw, "+0.00076", "what depth added on synthetic frames: nothing", big_size=32)
    d.stat(s, rx, 3.05, rw, "+0.169", "what it adds on real frames; the cube's AP@0.5 goes 0.099 → 0.905", big_size=32)
    d.stat(s, rx, 4.3, rw, "0.934", "after fine-tuning on 360 real frames: most of the sim-to-real gap closed", big_size=32)


def s_techniques(d):
    s = d.slide()
    d.frame(s, "06 · Techniques", "Four kinds of technique in one system",
            source="final report §4–7; CLAUDE.md, technique map")
    items = [("Supervised deep learning", "An RGB-D YOLOv8n trained on 40,012 synthetic frames from Isaac Sim, then fine-tuned on 360 labelled real ones"),
             ("A multimodal network", "A 4-channel early-fusion stem, its depth channel made by inflating the COCO weights; FoundationPose, a pretrained transformer; ONNX Runtime on CUDA"),
             ("Hybrid estimation", "Two per-view estimators combined by confidence weighting, a disagreement gate and depth evidence; a learned front end with a classical EKF behind it"),
             ("Intelligent sensing", "Two calibrated RGB-D cameras; each camera's depth used as evidence against the other's poses; poses turned into relational facts")]
    cw2 = (CW - 0.3) / 2
    for i, (h, b) in enumerate(items):
        x = L + (i % 2) * (cw2 + 0.3)
        y = 1.85 + (i // 2) * 2.3
        d.card(s, x, y, cw2, 2.1, h, b, num=i + 1, head_size=18, body_size=14)


def s_output(d):
    s = d.slide()
    d.frame(s, "07 · Output", "What a planner receives",
            source="demo_live_d at 60 s; INTERFACES.md §5")
    cw2 = (CW - 0.4) / 2
    d.box(s, L, 1.85, cw2, 4.5, fill=INK, line=None)
    d.text(s, L + 0.3, 2.0, cw2 - 0.6, 0.3, "(:init …) as emitted, live, at 60 s", size=12, color=MINT, font=MONO)
    init = ["(:init", "  (clear bowl_02)", "  (clear mustard_bottle_01)", "  (clear red_cube_04)",
            "  (in red_cube_04 bowl_02)", "  (on bowl_02 table_01)", "  (on mustard_bottle_01 table_01)", ")"]
    d.text(s, L + 0.3, 2.45, cw2 - 0.6, 3.6, init, size=17, color="E6ECEA", font=MONO, spacing=2)
    rx = L + cw2 + 0.4
    d.text(s, rx, 1.85, cw2, 0.35, "ObjectState: a fixed contract", size=18, color=INK, font=HEAD, bold=True)
    fields = [("id, cls", "instance and class"), ("pose_w", "fused 6-DoF pose, table frame"),
              ("confidence", "fused score"), ("relations", "filled by the scene graph"),
              ("visible_views, observed_views", "fused from, seen by"),
              ("extent", "half-extents, axis-aligned"), ("position_sigma_m, orientation_sigma_rad", "uncertainty"),
              ("velocity_w, stamp_ns", "motion and time")]
    for i, (f, m) in enumerate(fields):
        y = 2.35 + i * 0.42
        d.text(s, rx, y, cw2, 0.4, [[(f, {"font": MONO, "color": TEAL, "size": 13}), ("  " + m, {"size": 13})]],
               size=13, color=SLATE)
    d.text(s, rx, 5.8, cw2, 0.6, "Fields may be added, never removed or re-typed: the capstone, MANIP-WM, builds on it.",
           size=14, color=INK, bold=True)


def s_close(d):
    s = d.slide(dark=True)
    d.n += 1
    d.text(s, L, 0.9, CW, 0.3, "THE EYES · GROUP 12", size=12, color=MINT, font=MONO)
    d.text(s, L, 1.35, CW, 1.0, "Two cameras do see more than one", size=44, color="FFFFFF", font=HEAD, bold=True)
    d.text(s, L, 2.35, CW - 1.5, 1.0,
           "But the second camera matters most as a witness: it is what tells you when the first one is wrong.",
           size=22, color="C8D3D8", font=HEAD)
    res = [("2.76 mm", "fused ADD-S over the 16 scored scenes"),
           ("every frame", "an object hidden from one camera, still found"),
           ("9.8 mm", "the gate's margin over a plain average"),
           ("≥ 0.994", "precision of on, in and clear")]
    cw4 = (CW - 0.9) / 4
    for i, (b, t) in enumerate(res):
        x = L + i * (cw4 + 0.3)
        d.text(s, x, 3.95, cw4, 0.6, b, size=30, color=MINT, font=HEAD, bold=True)
        d.text(s, x, 4.6, cw4, 0.7, t, size=14, color="C8D3D8")
    d.text(s, L, 6.1, CW, 0.6,
           ["Code built with Claude Code under the author's design and review.",
            "Chan Wai Jye · Group 12 · AIS08 · 31 October 2026"],
           size=13, color="8A97A0", spacing=3)


ORDER = [s_title, s_problem, s_rig, s_pipeline, s_console, s_live, s_ground_truth, s_scenes, s_hidden,
         s_combine, s_wrong_camera, s_not_there, s_predicates, s_limits, s_depth, s_techniques, s_output, s_close]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", help="also write DIR/video_slides.pptx with the build stages")
    a = ap.parse_args()
    assert len(ORDER) == TOTAL
    d = Deck()
    for f in ORDER:
        f(d)
    out = "ProjectReport/PRS-PM-2026-AIS08-GRP-12-TheEyes-FinalPresentation.pptx"
    d.save(out)
    print("wrote", out, len(d.prs.slides._sldIdLst), "slides")
    if a.video:
        v = Deck(video=True)
        names = []
        for f in ORDER:
            if f is s_problem:
                for st in (1, 2, 3):
                    f(v, stage=st); names.append(f"problem_{st}")
            elif f is s_pipeline:
                f(v); names.append("pipeline")
                for k in range(len(PIPE_VIEW) + len(PIPE_PAIR)):
                    f(v, lit=k); names.append(f"pipeline_lit{k}")
            else:
                f(v); names.append(f.__name__[2:])
        os.makedirs(a.video, exist_ok=True)
        v.save(os.path.join(a.video, "video_slides.pptx"))
        with open(os.path.join(a.video, "video_slides.txt"), "w") as fh:
            fh.write("\n".join(f"{i + 1}\t{n}" for i, n in enumerate(names)) + "\n")
        print("wrote", a.video, len(names), "video slides")


if __name__ == "__main__":
    main()
