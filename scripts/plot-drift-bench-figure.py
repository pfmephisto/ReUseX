#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
"""Regenerate the #338 drift-benchmark figure (docs/research/figures/drift).

Dependency-free by design: emits SVG directly, so it runs in the dev shell
without matplotlib. The numbers below are the measured rows from
`docs/research/registration-improvements.md` §10 — edit them there and here
together.

Usage: python3 scripts/plot-drift-bench-figure.py [out.svg]
"""
import sys

OUT = sys.argv[1] if len(sys.argv) > 1 else (
    "docs/research/figures/drift/drift-bench-338.svg")

# Palette: the repo's existing research-figure colours (see
# scripts/plot-plane-term-figure.py / plot-odometry-trust-figure.py).
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e2e1dd"
SERIES = ["#2a78d6", "#eb6834", "#8a5cd6", "#1d8f6b"]
REF = "#9a9894"

W, H = 940, 980

# --- Measured data ---------------------------------------------------------
# drift level -> (no pose stage F@50mm, optimize F@50mm).
# The d=0 column is the UNDRIFTED original, recorded in §9.3; d=0.25 and d=1.0
# are this section's drifted variants (seed 1). "d" is --drift-scale, i.e. the
# realised median distant-pair disagreement is d * 0.20 * trajectory extent.
SCANS = [
    ("41069048", [(0.0, 0.8917, 0.8512),
                  (0.25, 0.4640, 0.7927),
                  (1.0, 0.2288, 0.2750)]),
    ("41069050", [(0.0, 0.8934, 0.8015),
                  (0.25, 0.5393, 0.6378),
                  (1.0, 0.1970, 0.2980)]),
    ("41069051", [(0.0, 0.8893, 0.8437),
                  (0.25, 0.4182, 0.7607),
                  (1.0, 0.2169, 0.2305)]),
]

LEVEL_LABELS = {0.0: "none", 0.25: "mild", 1.0: "heavy"}

# --- Panel C: the four configurations at HEAVY drift (d = 1.0) -------------
# scan -> (no pose stage, optimize, plane off + XFeat edges, plane + XFeat).
# The interesting cell is the last one: once drift exceeds the plane term's
# association gate, an outside constraint is what puts the seed back inside it.
HEAVY = [
    ("41069048", 0.2288, 0.2750, 0.2287, 0.2750),  # weak edges (169 mm)
    ("41069050", 0.1970, 0.2980, 0.3478, 0.6042),
    ("41069051", 0.2169, 0.2305, 0.3361, 0.5406),
]
CONFIG_LABELS = ["none", "optimize", "+XFeat\n(plane off)", "plane\n+XFeat"]

esc = lambda s: (str(s).replace("&", "&amp;").replace("<", "&lt;")
                 .replace(">", "&gt;"))


def text(x, y, s, size=12, fill=INK, anchor="start", weight="normal"):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
            f'text-anchor="{anchor}" font-weight="{weight}" '
            f'font-family="Inter, Helvetica, Arial, sans-serif">'
            f'{esc(s)}</text>')


def line(x1, y1, x2, y2, stroke=GRID, width=1, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return (f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="{stroke}" stroke-width="{width}"{d}/>')


def rect(x, y, w, h, fill):
    return (f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" '
            f'height="{h:.1f}" fill="{fill}"/>')


# The generated SVG carries its own REUSE header. The tags are assembled from
# fragments and fenced below so that REUSE reads this file's real header at the
# top and does not try to parse these string literals as a licence expression.
# REUSE-IgnoreStart
_SPDX = "SPDX-"
out = ['<!--',
       _SPDX + 'FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen',
       _SPDX + 'License-Identifier: GPL-3.0-or-later',
       '-->',
       f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
       f'viewBox="0 0 {W} {H}">',
       rect(0, 0, W, H, SURFACE)]
# REUSE-IgnoreEnd

out.append(text(28, 34, "Synthetic drift makes the ARKitScenes GT scans able "
                        "to adjudicate a pose stage (#338)", 17, INK,
                weight="600"))
out.append(text(28, 54, "Absolute GT (3dod_mesh) F@50mm, higher is better. "
                        "Drift is injected into a copy of the seed poses; "
                        "the GT mesh is untouched.", 12, MUTED))

# ── Panel A: three small multiples, F@50mm by drift level ──────────────────
PA_TOP, PA_BOT = 118, 340
PANEL_W, PANEL_GAP, PANEL_X0 = 270, 40, 40
BAR_W = 26

out.append(text(28, 80, "A · F@50mm against absolute GT, by injected drift",
                13, INK, weight="600"))

for si, (scan, rows) in enumerate(SCANS):
    x0 = PANEL_X0 + si * (PANEL_W + PANEL_GAP)
    # axes
    out.append(line(x0, PA_BOT, x0 + PANEL_W, PA_BOT, MUTED, 1))
    for gv in (0.2, 0.4, 0.6, 0.8):
        y = PA_BOT - gv * (PA_BOT - PA_TOP)
        out.append(line(x0, y, x0 + PANEL_W, y, GRID, 1))
        if si == 0:
            out.append(text(x0 - 8, y + 4, f"{gv:.1f}", 10, MUTED, "end"))
    out.append(text(x0 + PANEL_W / 2, PA_TOP - 12, scan, 12, INK, "middle",
                    "600"))

    group_w = PANEL_W / len(rows)
    for gi, (lvl, f_none, f_opt) in enumerate(rows):
        cx = x0 + group_w * (gi + 0.5)
        for k, (val, col) in enumerate(((f_none, REF), (f_opt, SERIES[0]))):
            bx = cx - BAR_W - 2 + k * (BAR_W + 4)
            bh = val * (PA_BOT - PA_TOP)
            out.append(rect(bx, PA_BOT - bh, BAR_W, bh, col))
            out.append(text(bx + BAR_W / 2, PA_BOT - bh - 5, f"{val:.2f}", 9,
                            MUTED, "middle"))
        out.append(text(cx, PA_BOT + 16, LEVEL_LABELS[lvl], 10, MUTED,
                        "middle"))
        out.append(text(cx, PA_BOT + 29,
                        "d=0" if lvl == 0 else f"d={lvl:g}", 9, MUTED,
                        "middle"))

# legend
lx, ly = PANEL_X0, PA_BOT + 52
out.append(rect(lx, ly - 9, 14, 11, REF))
out.append(text(lx + 20, ly, "no pose stage", 11, MUTED))
out.append(rect(lx + 130, ly - 9, 14, 11, SERIES[0]))
out.append(text(lx + 150, ly, "rux optimize (shipped default)", 11, MUTED))

# ── Panel B: the sign flip ────────────────────────────────────────────────
PB_TOP, PB_BOT = 440, 610
PB_X0, PB_W = 40, W - 120

out.append(text(28, 412, "B · What the pose stage is worth: "
                         "F(optimize) − F(no pose stage)", 13, INK,
                weight="600"))
out.append(text(28, 430, "Below the line the stage should do nothing; above "
                         "it, the stage is recovering drift.", 11, MUTED))

vals = [f_opt - f_none for _, rows in SCANS for _, f_none, f_opt in rows]
lo, hi = min(vals + [0.0]) - 0.05, max(vals) + 0.06
span = hi - lo
ypx = lambda v: PB_BOT - (v - lo) / span * (PB_BOT - PB_TOP)

for gv in (-0.1, 0.0, 0.1, 0.2, 0.3):
    if not (lo <= gv <= hi):
        continue
    y = ypx(gv)
    out.append(line(PB_X0, y, PB_X0 + PB_W, y,
                    MUTED if gv == 0 else GRID, 1.5 if gv == 0 else 1))
    out.append(text(PB_X0 - 8, y + 4, f"{gv:+.1f}", 10, MUTED, "end"))

xs = [PB_X0 + PB_W * (i + 0.5) / 3 for i in range(3)]
for i, lvl in enumerate((0.0, 0.25, 1.0)):
    out.append(text(xs[i], PB_BOT + 20,
                    f"{LEVEL_LABELS[lvl]}  (d={lvl:g})", 11, MUTED, "middle"))

for si, (scan, rows) in enumerate(SCANS):
    pts = [(xs[i], ypx(f_opt - f_none))
           for i, (_, f_none, f_opt) in enumerate(rows)]
    d = " ".join(f"{'M' if i == 0 else 'L'}{x:.1f},{y:.1f}"
                 for i, (x, y) in enumerate(pts))
    out.append(f'<path d="{d}" fill="none" stroke="{SERIES[si]}" '
               f'stroke-width="2.5"/>')
    for x, y in pts:
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" '
                   f'fill="{SERIES[si]}"/>')
    out.append(text(pts[-1][0] + 12, pts[-1][1] + 4, scan, 11, SERIES[si]))

# ── Panel C: configurations at heavy drift ────────────────────────────────
PC_TOP, PC_BOT = 765, 890

out.append(text(28, 700, "C · At heavy drift (d=1), what an outside constraint "
                         "is for", 13, INK, weight="600"))
out.append(text(28, 716, "The plane term recovers drift by associating plane "
                         "detections across frames through a 0.10 m gate. Past "
                         "that gate it stops working —", 11, MUTED))
out.append(text(28, 731, "and a loop edge that pulls the seed back inside it "
                         "is then worth more than either part alone. Where the "
                         "edges carry 169 mm of their own error (41069048), "
                         "nothing changes.", 11, MUTED))

CBAR = 22
for si, (scan, *cfgs) in enumerate(HEAVY):
    x0 = PANEL_X0 + si * (PANEL_W + PANEL_GAP)
    out.append(line(x0, PC_BOT, x0 + PANEL_W, PC_BOT, MUTED, 1))
    for gv in (0.2, 0.4, 0.6):
        y = PC_BOT - gv / 0.7 * (PC_BOT - PC_TOP)
        out.append(line(x0, y, x0 + PANEL_W, y, GRID, 1))
        if si == 0:
            out.append(text(x0 - 8, y + 4, f"{gv:.1f}", 10, MUTED, "end"))
    out.append(text(x0 + PANEL_W / 2, PC_TOP - 12, scan, 12, INK, "middle",
                    "600"))
    cols = [REF, SERIES[0], SERIES[1], SERIES[3]]
    for ci, val in enumerate(cfgs):
        bx = x0 + 26 + ci * (PANEL_W - 52) / 4
        bh = val / 0.7 * (PC_BOT - PC_TOP)
        out.append(rect(bx, PC_BOT - bh, CBAR, bh, cols[ci]))
        out.append(text(bx + CBAR / 2, PC_BOT - bh - 5, f"{val:.2f}", 9, MUTED,
                        "middle"))
        for li, ln in enumerate(CONFIG_LABELS[ci].split("\n")):
            out.append(text(bx + CBAR / 2, PC_BOT + 16 + li * 11, ln, 9, MUTED,
                            "middle"))

out.append(text(28, H - 12,
                "Undrifted (d=0) values are the recorded §9.3 rows; drifted "
                "variants use seed 1. Drift target = d x 0.20 x trajectory "
                "extent.", 10, MUTED))
out.append("</svg>")

with open(OUT, "w") as fh:
    fh.write("\n".join(out))
print(f"wrote {OUT}")
