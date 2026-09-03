# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Assemble side-by-side benchmark montages: the RGB sensor frame next to the
segmentation overlay from one or more annotated project DBs, with a shared class
colour legend. Used to produce the visual comparisons posted to the PR.

The label images are CV_16U with a +1 offset (0 = background/unlabelled, class =
value - 1) — the convention written by osd::make_labled_image + the DB. Classes
are indices into the prompt list used for that run (pass --classes).

Example::

    python -m reusex_sam3.montage \
        --rgb-db scan.rux \
        --panel "SAM 3:sam3.rux" --panel "SAM 3.1:sam31.rux" \
        --classes wall,floor,ceiling,window,door \
        --frames 6 --out docs/benchmarks/sam3_vs_sam31.png
"""

from __future__ import annotations

import argparse
import sqlite3
from typing import Optional

import cv2
import numpy as np

# A fixed, high-contrast palette (BGR) so a class keeps its colour across panels.
_PALETTE = [
    (0, 0, 220), (0, 160, 0), (220, 0, 0), (0, 200, 220), (220, 0, 220),
    (220, 160, 0), (120, 0, 200), (0, 120, 255), (150, 150, 0), (0, 220, 120),
    (200, 120, 120), (120, 200, 0), (200, 0, 120), (0, 80, 160), (160, 80, 0),
    (80, 160, 200), (200, 200, 120), (120, 120, 200),
]


def _color(cid: int):
    return _PALETTE[cid % len(_PALETTE)]


def _rgb_frames(db: str):
    con = sqlite3.connect(db)
    out = {}
    for nid, blob in con.execute(
        "SELECT node_id, color FROM sensor_frames WHERE color IS NOT NULL ORDER BY node_id"
    ):
        out[nid] = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
    return out


def _labels(db: str):
    con = sqlite3.connect(db)
    out = {}
    for nid, blob in con.execute(
        "SELECT node_id, label_image FROM segmentation_images ORDER BY node_id"
    ):
        out[nid] = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_UNCHANGED)
    return out


def _overlay(rgb, label, alpha=0.55):
    """Alpha-blend class colours (label>0) over the RGB frame."""
    out = rgb.copy()
    if label is None:
        return out
    if label.shape[:2] != rgb.shape[:2]:
        label = cv2.resize(label, (rgb.shape[1], rgb.shape[0]),
                           interpolation=cv2.INTER_NEAREST)
    color = np.zeros_like(rgb)
    for v in np.unique(label):
        if v == 0:
            continue
        color[label == v] = _color(int(v) - 1)
    mask = (label > 0)[..., None]
    blended = cv2.addWeighted(rgb, 1 - alpha, color, alpha, 0)
    return np.where(mask, blended, out)


def _label_bar(width, text):
    bar = np.full((34, width, 3), 40, np.uint8)
    cv2.putText(bar, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2, cv2.LINE_AA)
    return bar


def _legend(width, classes, present):
    """A wrapped legend strip: colour swatch + class name for classes present."""
    items = [(c, classes[c]) for c in sorted(present) if c < len(classes)]
    if not items:
        items = [(i, n) for i, n in enumerate(classes)]
    cols = max(1, width // 220)
    rows = (len(items) + cols - 1) // cols
    strip = np.full((rows * 30 + 10, width, 3), 30, np.uint8)
    for k, (cid, name) in enumerate(items):
        r, c = divmod(k, cols)
        x = 12 + c * 220
        y = 24 + r * 30
        cv2.rectangle(strip, (x, y - 14), (x + 22, y + 4), _color(cid), -1)
        cv2.putText(strip, f"{name}", (x + 30, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (235, 235, 235), 1, cv2.LINE_AA)
    return strip


def build_montage(rgb_db, panels, classes, n_frames=6, thumb_w=380,
                  out="montage.png", stride=None):
    """panels: list of (title, label_db). Renders a grid: rows=frames,
    cols=[RGB] + one per panel; a title row on top and a class legend below."""
    rgb = _rgb_frames(rgb_db)
    panel_labels = [(t, _labels(db)) for t, db in panels]
    ids = sorted(rgb)
    if stride:
        pick = ids[::stride][:n_frames]
    else:
        pick = [ids[int(i)] for i in np.linspace(0, len(ids) - 1, n_frames)]

    thumb_h = None
    present = set()
    col_titles = ["RGB"] + [t for t, _ in panels]
    grid_rows = []
    for nid in pick:
        base = rgb[nid]
        if thumb_h is None:
            thumb_h = int(base.shape[0] * thumb_w / base.shape[1])
        cells = [cv2.resize(base, (thumb_w, thumb_h))]
        for _, labels in panel_labels:
            lab = labels.get(nid)
            if lab is not None:
                present.update(int(v) - 1 for v in np.unique(lab) if v > 0)
            cells.append(cv2.resize(_overlay(base, lab), (thumb_w, thumb_h)))
        grid_rows.append(np.hstack(cells))
    grid = np.vstack(grid_rows)

    W = grid.shape[1]
    titles = np.hstack([_label_bar(thumb_w, t) for t in col_titles])
    legend = _legend(W, classes, present)
    montage = np.vstack([titles, grid, legend])
    cv2.imwrite(out, montage)
    print(f"[montage] wrote {out}  ({montage.shape[1]}x{montage.shape[0]}, "
          f"{len(pick)} frames, {len(panels)} panels)")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rgb-db", required=True, help="Project DB with RGB frames.")
    ap.add_argument("--panel", action="append", default=[],
                    help='"Title:label_db.rux" — repeatable, one column each.')
    ap.add_argument("--classes", required=True,
                    help="Comma-separated class names (prompt order).")
    ap.add_argument("--frames", type=int, default=6)
    ap.add_argument("--stride", type=int, default=None,
                    help="Pick every Nth frame instead of evenly spaced.")
    ap.add_argument("--out", default="montage.png")
    args = ap.parse_args(argv)

    panels = []
    for p in args.panel:
        title, db = p.split(":", 1)
        panels.append((title, db))
    classes = [c.strip() for c in args.classes.split(",")]
    build_montage(args.rgb_db, panels, classes, n_frames=args.frames,
                  out=args.out, stride=args.stride)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
