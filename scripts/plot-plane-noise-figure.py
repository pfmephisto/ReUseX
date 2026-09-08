# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Figure for issue #225: plane-factor measurement noise model, measured against
# absolute ground truth (ARKitScenes 3dod_mesh + MuSHRoom honka Faro laser).
#
# Two panels, one axis each (never a dual axis): GT F-score @50 mm (higher is
# better) and median accuracy error (lower is better). Same three pose configs
# per scan, so each group answers one question: does the pose stage help, and
# does the new noise model help relative to the shipped one?

import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRATCH = os.environ.get("REPORT_DIR", ".")

# Categorical slots 1-3 of the validated default palette (validator: all-pairs
# CVD dE 9.2, normal-vision 24.0, light surface). Every bar is direct-labeled,
# which is the required relief for the aqua slot's sub-3:1 surface contrast.
SERIES = [
    ("noop", "no pose stage", "#2a78d6"),
    ("inliers", "optimize — inlier-count noise (main)", "#eb6834"),
    ("fit", "optimize — fit-geometry noise (this PR)", "#1baf7a"),
]

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#dcdbd6"

SCANS = [
    ("41069048", "ARKitScenes\n41069048"),
    ("41069050", "ARKitScenes\n41069050"),
    ("41069051", "ARKitScenes\n41069051"),
    ("honka", "MuSHRoom honka\n(Faro laser GT)"),
]


def load(scan, tag):
    p = os.path.join(SCRATCH, f"{scan}-{tag}-accuracy.json")
    if not os.path.exists(p):
        return None
    return json.load(open(p))


scans = [(s, lbl) for s, lbl in SCANS if load(s, "fit") is not None]

fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.2))
fig.patch.set_facecolor(SURFACE)

panels = [
    ("fscore", "GT F-score @ 50 mm", "higher is better", 1.0, "{:.3f}"),
    ("accuracy_median", "Median accuracy error (mm)", "lower is better", 1000.0, "{:.1f}"),
]

for ax, (key, title, note, mul, fmt) in zip(axes, panels):
    ax.set_facecolor(SURFACE)
    n = len(SERIES)
    width = 0.24
    xs = range(len(scans))
    for si, (tag, label, color) in enumerate(SERIES):
        vals = []
        for scan, _ in scans:
            d = load(scan, tag)
            vals.append(d[key] * mul if d else 0.0)
        pos = [x + (si - (n - 1) / 2) * (width + 0.02) for x in xs]
        bars = ax.bar(
            pos, vals, width, label=label, color=color,
            edgecolor=SURFACE, linewidth=2.0,  # 2px surface gap between fills
        )
        for b, v in zip(bars, vals):
            ax.annotate(
                fmt.format(v),
                (b.get_x() + b.get_width() / 2, b.get_height()),
                textcoords="offset points", xytext=(0, 3),
                ha="center", va="bottom", fontsize=8, color=INK2,
            )

    ax.set_xticks(list(xs))
    ax.set_xticklabels([lbl for _, lbl in scans], fontsize=9, color=INK2)
    ax.set_title(f"{title}   ({note})", fontsize=11, color=INK, pad=10, loc="left")
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.xaxis.grid(False)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(axis="y", colors=INK2, labelsize=9, length=0)
    ax.tick_params(axis="x", length=0)

axes[0].set_ylim(0, 1.02)

handles, labels = axes[0].get_legend_handles_labels()
fig.legend(
    handles, labels, loc="lower center", ncol=3, frameon=False,
    fontsize=9.5, labelcolor=INK2, bbox_to_anchor=(0.5, -0.015),
)
fig.suptitle(
    "rux optimize: plane-factor measurement noise model vs absolute ground truth (#225)",
    fontsize=12.5, color=INK, x=0.012, ha="left", y=0.985,
)
fig.text(
    0.012, 0.925,
    "fit-geometry noise wins on all three DRIFTING scans and loses on the non-drifting one — "
    "the same drift-dependence the loop-closure front-end showed. Shipped opt-in, not as the default.",
    fontsize=9.5, color=INK2, ha="left",
)
fig.tight_layout(rect=(0, 0.07, 1, 0.90))
out = os.environ.get("OUT", "docs/research/images/plane-noise-model-gt.png")
fig.savefig(out, dpi=160, facecolor=SURFACE)
print("wrote", out)
for scan, _ in scans:
    for tag, label, _c in SERIES:
        d = load(scan, tag)
        if d:
            print(f"{scan:10s} {tag:8s} F={d['fscore']:.4f} "
                  f"chamfer={d['chamfer']*1000:6.2f}mm acc_med={d['accuracy_median']*1000:6.2f}mm")
