#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
"""Regenerate the #225 odometry-trust figure (docs/research/figures/odometry).

Dependency-free by design: emits SVG directly, so it runs in the dev shell
without matplotlib. The numbers below are the measured rows from
`docs/research/registration-improvements.md` §8 — edit them there and here
together.

Usage: python3 scripts/plot-odometry-trust-figure.py [out.svg]
"""
import math
import sys

OUT = sys.argv[1] if len(sys.argv) > 1 else (
    "docs/research/figures/odometry/odometry-trust-225.svg")

# Palette: the repo's existing research-figure colours, extended by one hue.
# Validated (light surface #fcfcfb, categorical): lightness band, chroma floor,
# CVD separation, normal-vision floor and contrast all PASS.
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e2e1dd"
SERIES = ["#2a78d6", "#eb6834", "#8a5cd6"]
REF = "#9a9894"

W, H = 900, 700

# --- Panel A: ARKitScenes GT F@50mm vs odometry trust multiplier ------------
# multiplier -> F@50mm; multiplier 1.0 is the shipped default (0.005 / 0.01).
SCANS = [
    ("41069048", [(0.1, 0.8802), (1, 0.8512), (10, 0.4565), (100, 0.2346)]),
    ("41069050", [(0.1, 0.8845), (1, 0.8015), (10, 0.4948)]),
    ("41069051", [(0.1, 0.8831), (1, 0.8437), (10, 0.7708)]),
]
# "no pose stage" F for the same three scans — drawn as one band, since the
# three values (0.8893 / 0.8917 / 0.8934) are indistinguishable at this scale.
NONE_LO, NONE_HI = 0.8893, 0.8934
# Hand-tuned end-label offsets: 41069048 and 41069050 converge near 10x looser,
# so their labels have to go in opposite directions to stay legible.
LABEL_NUDGE = {
    "41069048": (-10, 18),
    "41069050": (-6, -14),
    "41069051": (-10, 18),
}

# --- Panel B: office flatness_rms (GT-free), lower is better ----------------
OFFICE = [
    ("no pose\nstage", 12.50, False),
    ("10x\ntighter", 12.74, False),
    ("shipped\ndefault", 11.72, True),
    ("--odometry-\nnoise motion", 12.96, False),
    ("10x\nlooser", 13.36, False),
]

out = []
add = out.append
# REUSE-IgnoreStart
# (the generated SVG gets its own SPDX header below; these are string
# literals, not this script's own SPDX tags — reuse lint mis-detects them
# as a second, malformed license expression for this .py file otherwise)
add('<!--')
add('SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen')
add('SPDX-License-Identifier: GPL-3.0-or-later')
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
AX, AY, AW, AH = 92, 108, 620, 250
YLO, YHI = 0.18, 0.95


def ax_x(mult):
    # log10 axis from 0.1x to 100x
    return AX + (math.log10(mult) + 1) / 3.0 * AW


def ax_y(f):
    return AY + AH - (f - YLO) / (YHI - YLO) * AH


text(AX - 2, 40, "The odometry-trust hypothesis, tested", INK, 17, "600")
text(AX - 2, 62, "Loosening odometry does not recover the ARKitScenes GT "
                 "regression — it amplifies it, monotonically, on every scan.",
     MUTED, 12.5)
text(AX - 2, 80, "A. Absolute GT F-score @ 50 mm vs odometry trust "
                 "(higher is better)", INK, 14, "600")

# "no pose stage" reference band
by0, by1 = ax_y(NONE_HI), ax_y(NONE_LO)
add(f'<rect x="{AX:.1f}" y="{by0:.1f}" width="{AW:.1f}" '
    f'height="{max(by1 - by0, 1.5):.1f}" fill="{REF}" opacity="0.30"/>')
add(f'<line x1="{AX:.1f}" y1="{by0:.1f}" x2="{AX + AW:.1f}" y2="{by0:.1f}" '
    f'stroke="{REF}" stroke-width="1.5" stroke-dasharray="5 4"/>')
text(AX + AW - 4, by0 - 8, "no pose stage  (0.889 – 0.893)", MUTED, 11.5,
     anchor="end")

# y grid + labels
yt = [0.2, 0.4, 0.6, 0.8]
for v in yt:
    y = ax_y(v)
    add(f'<line x1="{AX:.1f}" y1="{y:.1f}" x2="{AX + AW:.1f}" y2="{y:.1f}" '
        f'stroke="{GRID}" stroke-width="1"/>')
    text(AX - 10, y + 4, f"{v:.1f}", MUTED, 11, anchor="end")

# x ticks
XT = [(0.1, "10x tighter"), (1, "shipped default"), (10, "10x looser"),
      (100, "100x looser")]
for m, lab in XT:
    x = ax_x(m)
    add(f'<line x1="{x:.1f}" y1="{AY:.1f}" x2="{x:.1f}" y2="{AY + AH:.1f}" '
        f'stroke="{GRID}" stroke-width="1"/>')
    text(x, AY + AH + 20, lab, MUTED, 11.5, anchor="middle")
text(AX + AW / 2, AY + AH + 40, "odometry sigma multiplier "
     "(--odometry-sigma-rot / --odometry-sigma-trans)", MUTED, 11.5,
     anchor="middle")

# series
for i, (name, pts) in enumerate(SCANS):
    c = SERIES[i]
    d = " ".join(("M" if k == 0 else "L") + f"{ax_x(m):.1f},{ax_y(f):.1f}"
                 for k, (m, f) in enumerate(pts))
    add(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="2" '
        f'stroke-linejoin="round" stroke-linecap="round"/>')
    for m, f in pts:
        add(f'<circle cx="{ax_x(m):.1f}" cy="{ax_y(f):.1f}" r="4.5" '
            f'fill="{c}" stroke="{SURFACE}" stroke-width="2"/>')
    # Direct label at each line's END, where the three are well separated (at
    # the left they bunch within 0.005 F and would collide). Secondary
    # encoding, so identity never rests on colour alone.
    mE, fE = pts[-1]
    dx, dy = LABEL_NUDGE[name]
    text(ax_x(mE) + dx, ax_y(fE) + dy, name, c, 11.5, "600", anchor="middle")

# legend (>= 2 series: always present)
lx = AX + AW + 18
text(lx, AY + 4, "ARKitScenes", MUTED, 11.5, "600")
for i, (name, _) in enumerate(SCANS):
    y = AY + 22 + i * 18
    add(f'<rect x="{lx}" y="{y - 8}" width="10" height="10" '
        f'fill="{SERIES[i]}" rx="2"/>')
    text(lx + 15, y + 1, name, MUTED, 11.5)
text(lx, AY + 96, "Every line sits", MUTED, 11)
text(lx, AY + 110, "BELOW the dashed", MUTED, 11)
text(lx, AY + 124, "reference: the best", MUTED, 11)
text(lx, AY + 138, "pose stage on these", MUTED, 11)
text(lx, AY + 152, "scans is none at all.", MUTED, 11)

# ============================ Panel B ======================================
BX, BY, BW, BH = 92, 470, 620, 150
text(BX - 2, 424, "B. Office scan — GT-free flatness_rms "
                  "(lower is better)", INK, 14, "600")
text(BX - 2, 444, "The same knob on the scan the defaults were tuned on: both "
                  "directions are worse, so the default is a local optimum.",
     MUTED, 12)

BMAX = 14.5
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

for i, (lab, v, best) in enumerate(OFFICE):
    x = BX + i * slot + 13
    y = by(v)
    c = SERIES[0] if best else REF
    add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bw:.1f}" '
        f'height="{BY + BH - y:.1f}" fill="{c}" rx="4"/>')
    text(x + bw / 2, y - 8, f"{v:.2f}", INK, 12, "600", anchor="middle")
    for k, line in enumerate(lab.split("\n")):
        text(x + bw / 2, BY + BH + 18 + k * 13, line, MUTED, 11,
             anchor="middle")

text(BX - 10, BY - 6, "mm", MUTED, 11, anchor="end")

# ============================ Footer =======================================
text(BX - 2, H - 28,
     "--odometry-robust (letting GNC demote individual odometry factors) is "
     "omitted from both panels: it is bit-identical to the", MUTED, 11.5)
text(BX - 2, H - 14,
     "default on all five scans — smooth drift has no single wrong edge for a "
     "robust kernel to find.", MUTED, 11.5)

add('</svg>')

with open(OUT, "w") as fh:
    fh.write("\n".join(out) + "\n")
print(f"wrote {OUT}")
