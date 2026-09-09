#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Side-by-side match visualiser for the loop-edge matcher backends (#264).
#
# Counting matches is not evidence: #236 showed a cross-camera ORB front-end
# producing 189 confident loop edges of which PCM rejected 186 as geometrically
# impossible. So this draws the correspondences a backend proposes for a given
# frame pair and colours each one by whether it survives the SAME RANSAC-Kabsch
# fit that `export_loop_edges.py` runs before emitting an edge:
#
#   green = inlier of the recovered rigid pose (a match the edge is built from)
#   red   = proposed but rejected
#
# A backend that draws a dense green bundle across a wide baseline is doing the
# job; one whose green lines all bunch into a single patch is producing a
# degenerate fit, which reads as "many inliers" in a table and as a wrong pose
# in the graph.

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from export_loop_edges import backproject, ransac_pose, read_frames  # noqa: E402
from matchers import load_matcher  # noqa: E402

GREEN = (80, 220, 90)
RED = (60, 60, 235)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def draw_pair(fi, fj, xy_i, xy_j, inl, title, max_lines=150):
    """One panel: the two frames side by side with their correspondences."""
    hi, wi = fi.color.shape[:2]
    hj, wj = fj.color.shape[:2]
    h = max(hi, hj)
    bar = 26
    canvas = np.zeros((h + bar, wi + wj, 3), np.uint8)
    canvas[bar : bar + hi, :wi] = fi.color
    canvas[bar : bar + hj, wi:] = fj.color

    n = len(xy_i)
    sel = np.arange(n)
    if n > max_lines:  # keep the drawing legible; sample, never reorder
        sel = np.linspace(0, n - 1, max_lines).astype(int)
    # draw rejects first so surviving matches stay readable on top
    for keep in (False, True):
        for k in sel:
            if bool(inl[k]) != keep:
                continue
            a = (int(round(xy_i[k, 0])), int(round(xy_i[k, 1])) + bar)
            b = (int(round(xy_j[k, 0])) + wi, int(round(xy_j[k, 1])) + bar)
            col = GREEN if keep else RED
            cv2.line(canvas, a, b, col, 1, cv2.LINE_AA)
            cv2.circle(canvas, a, 2, col, -1, cv2.LINE_AA)
            cv2.circle(canvas, b, 2, col, -1, cv2.LINE_AA)
    cv2.putText(canvas, title, (8, 18), FONT, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def evaluate(fi, fj, xy_i, xy_j, args, rng):
    """Run the exporter's own lift + RANSAC so the picture matches the pipeline."""
    p_i, vi = backproject(xy_i, fi.depth_m, fi.K, args.min_depth, args.max_depth)
    p_j, vj = backproject(xy_j, fj.depth_m, fj.K, args.min_depth, args.max_depth)
    both = vi & vj
    inl_full = np.zeros(len(xy_i), bool)
    if int(both.sum()) < 3:
        return inl_full, 0, None
    T, inl = ransac_pose(
        p_j[both], p_i[both], args.ransac_thresh, args.ransac_iters, rng
    )
    if T is None:
        return inl_full, 0, None
    inl_full[np.flatnonzero(both)[inl]] = True
    return inl_full, int(inl.sum()), T


def main():
    ap = argparse.ArgumentParser(
        description="Draw and compare loop-edge matcher correspondences (#264)."
    )
    ap.add_argument("project", help="path to the .rux project database")
    ap.add_argument("-o", "--output", required=True, help="output image (.jpg/.png)")
    ap.add_argument(
        "--pair",
        required=True,
        action="append",
        help="frame pair as NODE_I:NODE_J (repeatable)",
    )
    ap.add_argument(
        "--matchers",
        default="orb,mapanything",
        help="comma-separated backends to stack, in order",
    )
    ap.add_argument("--max-matches", type=int, default=4000)
    ap.add_argument("--max-lines", type=int, default=150)
    ap.add_argument("--ransac-thresh", type=float, default=0.10)
    ap.add_argument("--ransac-iters", type=int, default=500)
    ap.add_argument("--min-depth", type=float, default=0.3)
    ap.add_argument("--max-depth", type=float, default=5.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--weights", default=None)
    ap.add_argument("--variant", default=None)
    ap.add_argument("--allow-noncommercial", action="store_true")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    frames = read_frames(args.project)
    by_id = {f.node_id: f for f in frames}
    pairs = []
    for spec in args.pair:
        a, b = (int(x) for x in spec.replace(",", ":").split(":"))
        missing = [n for n in (a, b) if n not in by_id]
        if missing:
            sys.exit(f"node id(s) {missing} not in {args.project}")
        pairs.append((a, b))

    panels = []
    for name in args.matchers.split(","):
        name = name.strip()
        matcher = load_matcher(SimpleNamespace(**{**vars(args), "matcher": name}))
        for a, b in pairs:
            fi, fj = by_id[a], by_id[b]
            m = matcher.match(fi, fj, max_matches=args.max_matches)
            if m is None:
                title = f"{name}: {a} <-> {b}   NO MATCHES"
                panels.append(
                    draw_pair(fi, fj, np.zeros((0, 2)), np.zeros((0, 2)), [], title)
                )
                print(f"[{name}] {a}<->{b}: no matches")
                continue
            xy_i, xy_j = m
            inl, ninl, _ = evaluate(fi, fj, xy_i, xy_j, args, rng)
            spread = float(np.hypot(*xy_i[inl].std(0))) if ninl else 0.0
            title = (
                f"{name}: {a} <-> {b}   {len(xy_i)} matches, "
                f"{ninl} RANSAC inliers, inlier spread {spread:.0f} px"
            )
            panels.append(draw_pair(fi, fj, xy_i, xy_j, inl, title, args.max_lines))
            print(
                f"[{name}] {a}<->{b}: {len(xy_i)} matches, {ninl} inliers, "
                f"spread {spread:.0f}px"
            )
        del matcher

    w = max(p.shape[1] for p in panels)
    padded = [np.pad(p, ((0, 6), (0, w - p.shape[1]), (0, 0))) for p in panels]
    out = np.vstack(padded)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(args.output, out):
        sys.exit(f"failed to write {args.output}")
    print(f"[done] {out.shape[1]}x{out.shape[0]} -> {args.output}")


if __name__ == "__main__":
    main()
