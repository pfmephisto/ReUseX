#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Render the camera trajectory of one or more .rux projects top-down (X-Y), for
# before/after loop-closure comparison (issue #221/#225). A drifting scan shows
# a start<->end gap that a correct loop closure pulls shut; this makes that
# visible at a glance. Also marks the frames referenced by an edge JSON so the
# loop constraint is shown against the trajectory it corrects.
#
# Usage:
#   render_trajectory.py out.png LABEL1=proj1.rux [LABEL2=proj2.rux ...] \
#       [--edges edges.json] [--title "..."]

import argparse
import json
import sqlite3

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def centres(db_path):
    con = sqlite3.connect(db_path)
    rows = con.execute(
        "SELECT node_id, transform FROM sensor_frames "
        "WHERE transform IS NOT NULL ORDER BY node_id"
    ).fetchall()
    con.close()
    ids = np.array([r[0] for r in rows])
    C = np.array([np.frombuffer(r[1], np.float64).reshape(4, 4)[:3, 3] for r in rows])
    return ids, C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("projects", nargs="+", help="LABEL=path.rux")
    ap.add_argument("--edges", default=None)
    ap.add_argument("--title", default="Camera trajectory (top-down)")
    args = ap.parse_args()

    fig, ax = plt.subplots(figsize=(9, 8))
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]
    id_to_xy = {}
    for k, spec in enumerate(args.projects):
        label, path = spec.split("=", 1)
        ids, C = centres(path)
        c = colors[k % len(colors)]
        ax.plot(C[:, 0], C[:, 1], "-", color=c, lw=1.2, alpha=0.85, label=label)
        ax.plot(*C[0, :2], "o", color=c, ms=10, mfc="white", mew=2)  # start
        ax.plot(*C[-1, :2], "s", color=c, ms=10, mfc="white", mew=2)  # end
        gap = np.linalg.norm(C[0] - C[-1])
        ax.annotate(
            f"{label}: start↔end {gap:.2f} m",
            xy=(0.02, 0.98 - 0.04 * k),
            xycoords="axes fraction",
            color=c,
            fontsize=10,
            va="top",
        )
        if k == 0:
            id_to_xy = {int(i): C[n, :2] for n, i in enumerate(ids)}

    if args.edges:
        ed = json.load(open(args.edges))["edges"]
        drawn = 0
        for e in ed:
            a = id_to_xy.get(int(e["node_i"]))
            b = id_to_xy.get(int(e["node_j"]))
            if a is None or b is None:
                continue
            ax.plot([a[0], b[0]], [a[1], b[1]], "-", color="0.6", lw=0.5, alpha=0.5)
            drawn += 1
        ax.plot([], [], "-", color="0.6", lw=0.8, label=f"{drawn} loop edges")

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(args.title)
    ax.legend(loc="lower right", fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
