#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
"""Regenerate the #225 plane-term figure (docs/research/figures/plane-term).

Dependency-free by design: emits SVG directly, so it runs in the dev shell
without matplotlib. The numbers below are the measured rows from
`docs/research/registration-improvements.md` §9 — edit them there and here
together.

Usage: python3 scripts/plot-plane-term-figure.py [out.svg]
"""
import math
import sys

OUT = sys.argv[1] if len(sys.argv) > 1 else (
    "docs/research/figures/plane-term/plane-term-225.svg")

# Palette: the repo's existing research-figure colours (see
# scripts/plot-odometry-trust-figure.py), extended by one hue for the honka
# guard series. Validated (light surface #fcfcfb, categorical): lightness band,
# chroma floor, CVD separation, normal-vision floor and contrast all PASS.
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e2e1dd"
SERIES = ["#2a78d6", "#eb6834", "#8a5cd6", "#1d8f6b"]
REF = "#9a9894"

W, H = 940, 900

# --- Panel A: plane-term weight sweep, GT F@50mm RELATIVE to no pose stage --
# scale -> F@50mm. scale is --plane-sigma-scale: the plane term's weight in the
# objective goes as 1/scale^2, so LEFT = stronger plane term, RIGHT = weaker.
# "off" (--no-plane-factors) is the scale -> infinity limit, drawn at the right.
SCANS = [
    ("41069048", 0.8917, [(0.1, 0.4592), (0.316, 0.7138), (1, 0.8512),
                          (3.162, 0.8597), (10, 0.8802), (100, 0.8917)],
     0.8917),
    ("41069050", 0.8934, [(0.1, 0.5172), (0.316, 0.7025), (1, 0.8015),
                          (3.162, 0.8347), (10, 0.8845), (100, 0.8934)],
     0.8934),
    ("41069051", 0.8893, [(0.1, 0.7174), (0.316, 0.8452), (1, 0.8437),
                          (3.162, 0.8560), (10, 0.8831), (100, 0.8908)],
     0.8893),
    ("honka", 0.7572, [(0.1, 0.7229), (0.316, 0.7474), (1, 0.7595),
                       (3.162, 0.7517), (10, 0.7499), (100, 0.7571)],
     0.7572),
]

# --- Panel B: office flatness_rms (GT-free), lower is better ---------------
OFFICE = [
    ("0.1", 14.06, False), ("0.32", 12.80, False), ("1.0", 11.72, True),
    ("3.2", 12.02, False), ("10", 12.74, False), ("100", 12.86, False),
    ("off", 12.72, False),
]
OFFICE_NONE = 12.50

# --- Panel C: XFeat loop edges on the GT scans, F@50mm relative to none -----
# (scan, F with the shipped 0.50 m gate, F with a 0.05 m gate), plane term off.
XFEAT = [
    ("41069048", 0.8917, 0.8917, 0.7316),
    ("41069050", 0.8934, 0.4468, 0.7594),
    ("41069051", 0.8893, 0.2911, 0.7505),
]

out = []
add = out.append
# REUSE-IgnoreStart
add('<!--')
# REUSE-IgnoreStart
# These two lines are the SVG's own REUSE header, emitted into the output
# file -- they are data, not this script's licence.
add('SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen')
add('SPDX-License-Identifier: GPL-3.0-or-later')
# REUSE-IgnoreEnd
add('-->')
# REUSE-IgnoreEnd
add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
    f'viewBox="0 0 {W} {H}" font-family="Helvetica,Arial,sans-serif">')
add(f'<rect width="{W}" height="{H}" fill="{SURFACE}"/>')


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def text(x, y, s, fill=MUTED, size=12, weight=None, anchor=None):
    a = f' text-anchor="{anchor}"' if anchor else ""
    w = f' font-weight="{weight}"' if weight else ""
    add(f'<text x="{x:.1f}" y="{y:.1f}" fill="{fill}" font-size="{size}"{w}{a}>'
        f'{esc(s)}</text>')


# ============================ Panel A ======================================
AX, AY, AW, AH = 96, 116, 610, 250
OFF_GAP = 66              # x-space between the "100" tick and the "off" tick
YLO, YHI = -0.46, 0.05


def ax_x(scale):
    # log10 axis from 0.1 to 100
    return AX + (math.log10(scale) + 1) / 3.0 * (AW - OFF_GAP)


AX_OFF = AX + AW


def ax_y(d):
    return AY + AH - (d - YLO) / (YHI - YLO) * AH


text(AX - 4, 40, "Is there a GT-optimal plane-term weight?", INK, 17, "600")
text(AX - 4, 62, "Sweeping the plane term's authority against absolute ground "
                 "truth. Zero = running no pose stage at all.", MUTED, 12.5)
text(AX - 4, 88, "A. Change in GT F-score @ 50 mm vs no pose stage "
                 "(higher is better; 0 = no pose stage)", INK, 14, "600")

# zero reference = "no pose stage"
zy = ax_y(0.0)
add(f'<line x1="{AX:.1f}" y1="{zy:.1f}" x2="{AX + AW:.1f}" y2="{zy:.1f}" '
    f'stroke="{REF}" stroke-width="1.5" stroke-dasharray="5 4"/>')
text(AX + 4, zy - 8, "no pose stage", MUTED, 11.5)

# y grid + labels
for v in (-0.4, -0.3, -0.2, -0.1, 0.0):
    y = ax_y(v)
    if v != 0.0:
        add(f'<line x1="{AX:.1f}" y1="{y:.1f}" x2="{AX + AW:.1f}" y2="{y:.1f}" '
            f'stroke="{GRID}" stroke-width="1"/>')
    text(AX - 10, y + 4, f"{v:+.1f}" if v else "0", MUTED, 11, anchor="end")

# x ticks
for s, lab in [(0.1, "0.1"), (0.316, "0.32"), (1, "1.0"), (3.162, "3.2"),
               (10, "10"), (100, "100")]:
    x = ax_x(s)
    add(f'<line x1="{x:.1f}" y1="{AY:.1f}" x2="{x:.1f}" y2="{AY + AH:.1f}" '
        f'stroke="{GRID}" stroke-width="1"/>')
    text(x, AY + AH + 20, lab, MUTED, 11.5, anchor="middle")
add(f'<line x1="{AX_OFF:.1f}" y1="{AY:.1f}" x2="{AX_OFF:.1f}" '
    f'y2="{AY + AH:.1f}" stroke="{GRID}" stroke-width="1"/>')
text(AX_OFF, AY + AH + 20, "off", MUTED, 11.5, "600", anchor="middle")
text(ax_x(1), AY + AH + 42, "--plane-sigma-scale   (left = stronger plane "
     "term, right = weaker; 1.0 = shipped default)", MUTED, 11.5,
     anchor="middle")

# series
for i, (name, base, pts, off_f) in enumerate(SCANS):
    c = SERIES[i]
    xy = [(ax_x(s), ax_y(f - base)) for s, f in pts]
    xy.append((AX_OFF, ax_y(off_f - base)))
    d = " ".join(("M" if k == 0 else "L") + f"{x:.1f},{y:.1f}"
                 for k, (x, y) in enumerate(xy))
    add(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="2" '
        f'stroke-linejoin="round" stroke-linecap="round"/>')
    for x, y in xy:
        add(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4.5" fill="{c}" '
            f'stroke="{SURFACE}" stroke-width="2"/>')

# honka's peak is +0.0023 — real, but invisible on an axis that has to span
# -0.45. Label it directly rather than truncate the axis or split the scale.
hx, hy = ax_x(1), ax_y(0.7595 - 0.7572)
add(f'<line x1="{hx:.1f}" y1="{hy - 6:.1f}" x2="{hx:.1f}" y2="{hy - 22:.1f}" '
    f'stroke="{SERIES[3]}" stroke-width="1.2"/>')
text(hx, hy - 27, "honka peak +0.0023", SERIES[3], 11, "600", anchor="middle")

# legend (>= 2 series: always present)
lx = AX + AW + 26
text(lx, AY + 4, "scan", MUTED, 11.5, "600")
for i, (name, _, _, _) in enumerate(SCANS):
    y = AY + 22 + i * 18
    add(f'<rect x="{lx}" y="{y - 8}" width="10" height="10" '
        f'fill="{SERIES[i]}" rx="2"/>')
    text(lx + 15, y + 1, name, MUTED, 11.5)
text(lx, AY + 116, "The three drifting", MUTED, 11)
text(lx, AY + 130, "scans rise to zero", MUTED, 11)
text(lx, AY + 144, "and stop: their best", MUTED, 11)
text(lx, AY + 158, "plane weight is OFF.", MUTED, 11)
text(lx, AY + 180, "honka (no drift) is", MUTED, 11)
text(lx, AY + 194, "the only scan with a", MUTED, 11)
text(lx, AY + 208, "peak above zero —", MUTED, 11)
text(lx, AY + 222, "at the shipped 1.0.", MUTED, 11)

# ============================ Panel B ======================================
BX, BY, BW, BH = 96, 486, 610, 130
text(BX - 4, 434, "B. Office scan — GT-free flatness_rms, mm (lower is better)",
     INK, 14, "600")
text(BX - 4, 456, "The GT-free metric disagrees: on the scan the defaults were "
                  "tuned on, the optimum is the shipped 1.0, not off.", MUTED,
     12)

BMAX = 15.0
n = len(OFFICE)
slot = BW / n
bw = slot - 26


def by(v):
    return BY + BH - v / BMAX * BH


for v in (0, 5, 10):
    y = by(v)
    add(f'<line x1="{BX:.1f}" y1="{y:.1f}" x2="{BX + BW:.1f}" y2="{y:.1f}" '
        f'stroke="{GRID}" stroke-width="1"/>')
    text(BX - 10, y + 4, str(v), MUTED, 11, anchor="end")

ny = by(OFFICE_NONE)
add(f'<line x1="{BX:.1f}" y1="{ny:.1f}" x2="{BX + BW:.1f}" y2="{ny:.1f}" '
    f'stroke="{REF}" stroke-width="1.5" stroke-dasharray="5 4"/>')
text(BX + BW + 8, ny + 4, f"no pose stage {OFFICE_NONE:.2f}", MUTED, 11)

for i, (lab, v, best) in enumerate(OFFICE):
    x = BX + i * slot + 13
    y = by(v)
    c = SERIES[0] if best else REF
    add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bw:.1f}" '
        f'height="{BY + BH - y:.1f}" fill="{c}" rx="4"/>')
    text(x + bw / 2, y - 8, f"{v:.2f}", INK, 12, "600", anchor="middle")
    text(x + bw / 2, BY + BH + 18, lab, MUTED, 11.5, anchor="middle")
text(BX + BW / 2, BY + BH + 40, "--plane-sigma-scale", MUTED, 11.5,
     anchor="middle")

# ============================ Panel C ======================================
CX, CY, CW, CH = 96, 726, 610, 118
text(CX - 4, 674, "C. XFeat loop edges on the same GT scans — change in "
                  "F@50mm vs no pose stage", INK, 14, "600")
text(CX - 4, 696, "Applied with the plane term off. Every configuration that "
                  "moves the poses loses GT accuracy.", MUTED, 12)

CLO = -0.62


def cy(d):
    return CY + (d / CLO) * CH


for v in (0.0, -0.2, -0.4, -0.6):
    y = cy(v)
    add(f'<line x1="{CX:.1f}" y1="{y:.1f}" x2="{CX + CW:.1f}" y2="{y:.1f}" '
        f'stroke="{GRID if v else REF}" stroke-width="{1 if v else 1.5}"'
        f'{"" if v else chr(32) + "stroke-dasharray=" + chr(34) + "5 4" + chr(34)}/>')
    text(CX - 10, y + 4, f"{v:+.1f}" if v else "0", MUTED, 11, anchor="end")

GROUPS = [("shipped gate 0.50 m", 2), ("gate 0.05 m", 3)]
gslot = CW / len(GROUPS)
for gi, (glab, idx) in enumerate(GROUPS):
    for si, row in enumerate(XFEAT):
        name, base = row[0], row[1]
        d = row[idx] - base
        bwid = 42
        x = CX + gi * gslot + 60 + si * (bwid + 14)
        y0, y1 = cy(0.0), cy(d)
        add(f'<rect x="{x:.1f}" y="{min(y0, y1):.1f}" width="{bwid}" '
            f'height="{abs(y1 - y0):.1f}" fill="{SERIES[si]}" rx="4"/>')
        text(x + bwid / 2, y1 + 14, f"{d:+.3f}", INK, 11, "600",
             anchor="middle")
        if d == 0.0:
            text(x + bwid / 2, y1 + 30, "no edges", MUTED, 10,
                 anchor="middle")
            text(x + bwid / 2, y1 + 42, "survived PCM", MUTED, 10,
                 anchor="middle")
    text(CX + gi * gslot + 60 + (3 * 42 + 2 * 14) / 2, CY + CH + 26, glab,
         MUTED, 11.5, "600", anchor="middle")

text(CX + CW + 26, CY + 10, "Why: on these", MUTED, 11)
text(CX + CW + 26, CY + 24, "scans the edges", MUTED, 11)
text(CX + CW + 24 + 2, CY + 38, "disagree with the", MUTED, 11)
text(CX + CW + 26, CY + 52, "seed by 58–169 mm", MUTED, 11)
text(CX + CW + 26, CY + 66, "on a ~2 m trajectory,", MUTED, 11)
text(CX + CW + 26, CY + 80, "but the seed is", MUTED, 11)
text(CX + CW + 26, CY + 94, "already accurate to", MUTED, 11)
text(CX + CW + 26, CY + 108, "16 mm. The office", MUTED, 11)
text(CX + CW + 26, CY + 122, "scan: 14.5 m on 18 m.", MUTED, 11)

add('</svg>')
with open(OUT, "w") as fh:
    fh.write("\n".join(out) + "\n")
print(f"wrote {OUT}")
