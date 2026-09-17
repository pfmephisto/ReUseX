#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
GT Curator — Gradio-based tool for curating ground-truth anchor correspondences
for the ReUseX pose dataset (issue #221).

Launch:
  python tools/gt_curator/app.py -p /path/to/project.rux [--port 7860]

Dependencies (in ~/gt-curator-venv plus Nix paths):
  gradio, opencv-python (nix), numpy (nix), scipy

Learned matchers (MASt3R, XFeat) run out-of-process via match_worker.py
in their own venvs (~/loop-edges-work/{mast3r,xfeat}/.venv).

Solve (GTSAM) runs out-of-process via solve_gt_poses_cli.py in the mast3r
venv (which has gtsam + scipy + numpy; the gradio venv does NOT need gtsam).
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
from pathlib import Path
from typing import Optional

import cv2
import gradio as gr
import numpy as np

# ── local modules ───────────────────────────────────────────────────────────
_HERE = Path(__file__).resolve().parent
_LOOP_EDGES = _HERE.parent / "loop_edges"
sys.path.insert(0, str(_LOOP_EDGES))
sys.path.insert(0, str(_HERE))

from anchors import AnchorList  # noqa: E402
from db_reader import (  # noqa: E402
    propose_candidates,
    read_all_poses,
    read_color_thumbnail,
    read_depth_thumbnail,
    read_frame_count,
    read_seed_positions,
)
from icp_align import run_icp_on_frames, sliders_to_T  # noqa: E402
from opencv_features import clear_cache, match_pair  # noqa: E402

# ── paths ───────────────────────────────────────────────────────────────────
XFEAT_VENV = Path.home() / "loop-edges-work" / "xfeat" / ".venv" / "bin" / "python"
MAST3R_VENV = Path.home() / "loop-edges-work" / "mast3r" / ".venv" / "bin" / "python"
# The solve CLI uses gtsam which needs numpy<2; the curator venv has this set up correctly.
SOLVE_VENV = Path.home() / "gt-curator-venv" / "bin" / "python"
RUX_BIN = Path("/home/mephisto/repos/ReUseX/build/apps/rux/rux")
MATCH_WORKER = _HERE / "match_worker.py"
SOLVE_CLI = _HERE / "solve_gt_poses_cli.py"

# libstdc++ needed for gtsam + xfeat wheels
_GCC_LIB = "/nix/store/xm08aqdd7pxcdhm0ak6aqb1v7hw5q6ri-gcc-14.3.0-lib/lib"


_MAST3R_VENV_SITE = str(
    Path.home() / "loop-edges-work" / "mast3r" / ".venv" / "lib" / "python3.13" / "site-packages"
)
_XFEAT_VENV_SITE = str(
    Path.home() / "loop-edges-work" / "xfeat" / ".venv" / "lib" / "python3.13" / "site-packages"
)
# Solve venv: curator venv has gtsam + scipy 1.14 + numpy 1.26 (compatible)
_SOLVE_VENV_SITE = str(Path.home() / "gt-curator-venv" / "lib" / "python3.13" / "site-packages")
# libz needed by pip numpy wheels
_ZLIB = "/nix/store/l7xwm1f6f3zj2x8jwdbi8gdyfbx07sh7-zlib-1.3.1/lib"


def _subprocess_env(venv_site: Optional[str] = None):
    """Build subprocess environment.

    For the solve subprocess: we need gtsam (which requires numpy<2) so we put
    the curator venv site-packages as the ONLY PYTHONPATH to shadow Nix numpy 2.x.
    For xfeat/mast3r matchers: their own venv sites are used.
    """
    env = os.environ.copy()
    ld = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{_GCC_LIB}:{_ZLIB}:{ld}" if ld else f"{_GCC_LIB}:{_ZLIB}"
    if venv_site:
        # Override Nix PYTHONPATH so the venv's numpy wins over Nix numpy 2.x
        env["PYTHONPATH"] = venv_site
    return env


# ── global state ─────────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_db_path: Optional[str] = None
_positions: list = []          # [(node_id, xyz_or_None)]
_node_ids: list[int] = []      # sorted list
_anchors = AnchorList()

# Last match result (for Accept button)
_last_match: Optional[dict] = None


# ── helpers ──────────────────────────────────────────────────────────────────

def _require_db():
    if not _db_path:
        raise gr.Error("No project loaded. Pass -p /path/to/project.rux at startup.")


def _node_id_at_index(idx: int) -> int:
    if not _node_ids:
        return 0
    idx = max(0, min(idx, len(_node_ids) - 1))
    return _node_ids[idx]


def _scatter_plot_data():
    """Return a PIL image of the top-down frame scatter."""
    if not _positions:
        return None
    pts = [(nid, xyz) for nid, xyz in _positions if xyz is not None]
    if not pts:
        return None
    xs = np.array([p[1][0] for p in pts])
    ys = np.array([p[1][1] for p in pts])  # use X and Y as top-down

    # Render to image
    margin = 20
    h, w = 400, 400
    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()
    rng_x = max(x_max - x_min, 0.1)
    rng_y = max(y_max - y_min, 0.1)

    canvas = np.ones((h, w, 3), dtype=np.uint8) * 240

    def to_px(x, y):
        px = int((x - x_min) / rng_x * (w - 2 * margin)) + margin
        py = int((1 - (y - y_min) / rng_y) * (h - 2 * margin)) + margin
        return px, py

    for _, xyz in pts:
        px, py = to_px(xyz[0], xyz[1])
        cv2.circle(canvas, (px, py), 2, (100, 100, 200), -1)

    return canvas  # RGB


def _draw_scatter_with_highlight(node_a: Optional[int], node_b: Optional[int]):
    canvas = _scatter_plot_data()
    if canvas is None:
        return None

    if not _positions:
        return canvas

    pts = [(nid, xyz) for nid, xyz in _positions if xyz is not None]
    xs = np.array([p[1][0] for p in pts])
    ys = np.array([p[1][1] for p in pts])
    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()
    rng_x = max(x_max - x_min, 0.1)
    rng_y = max(y_max - y_min, 0.1)
    h, w = canvas.shape[:2]
    margin = 20

    def to_px(x, y):
        px = int((x - x_min) / rng_x * (w - 2 * margin)) + margin
        py = int((1 - (y - y_min) / rng_y) * (h - 2 * margin)) + margin
        return px, py

    pos_map = {nid: xyz for nid, xyz in _positions if xyz is not None}
    for nid, col, size in [(node_a, (220, 80, 80), 6), (node_b, (80, 220, 80), 6)]:
        if nid is not None and nid in pos_map:
            px, py = to_px(pos_map[nid][0], pos_map[nid][1])
            cv2.circle(canvas, (px, py), size, col, -1)

    return canvas


def _draw_correspondences(match: dict, db_path: str, node_i: int, node_j: int):
    """Return side-by-side correspondence image using draw_pair from visualize_matches."""
    from export_loop_edges import read_frames
    from visualize_matches import draw_pair

    frames = read_frames(db_path)
    by_id = {f.node_id: f for f in frames}
    fi = by_id.get(node_i)
    fj = by_id.get(node_j)
    if fi is None or fj is None:
        return None

    xy_i = np.array(match["xy_i"]) if match["xy_i"] else np.zeros((0, 2))
    xy_j = np.array(match["xy_j"]) if match["xy_j"] else np.zeros((0, 2))
    inl = np.array(match["inlier_mask"]) if match["inlier_mask"] else np.zeros(0, bool)

    if len(xy_i) == 0:
        return None

    n_inl = int(inl.sum()) if len(inl) else 0
    rms_str = f"{match['rms']*1000:.1f}mm" if match.get("rms") else "—"
    title = f"n={len(xy_i)} matches, {n_inl} inliers, RMS={rms_str}"
    img = draw_pair(fi, fj, xy_i, xy_j, inl, title)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


# ── subprocess helpers ────────────────────────────────────────────────────────

def _run_learned_matcher(backend: str, db_path: str, node_i: int, node_j: int) -> dict:
    """Run match_worker.py in the appropriate venv and return the result dict."""
    venv_python = str(XFEAT_VENV if backend == "xfeat" else MAST3R_VENV)
    if not Path(venv_python).exists():
        return {"error": f"venv not found: {venv_python}", "T_ij": None,
                "n_inliers": 0, "rms": None, "xy_i": [], "xy_j": [], "inlier_mask": []}

    venv_site = _XFEAT_VENV_SITE if backend == "xfeat" else _MAST3R_VENV_SITE
    cmd = [
        venv_python, str(MATCH_WORKER),
        db_path, str(node_i), str(node_j), backend,
        "--allow-noncommercial",
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=300,
            env=_subprocess_env(venv_site=venv_site)
        )
        if result.returncode != 0:
            err = result.stderr.strip().splitlines()[-1] if result.stderr else "unknown"
            return {"error": f"worker failed: {err}", "T_ij": None,
                    "n_inliers": 0, "rms": None, "xy_i": [], "xy_j": [], "inlier_mask": []}
        stdout = result.stdout.strip()
        # Last non-empty line should be the JSON
        for line in reversed(stdout.splitlines()):
            line = line.strip()
            if line.startswith("{"):
                return json.loads(line)
        return {"error": "no JSON in worker output", "T_ij": None,
                "n_inliers": 0, "rms": None, "xy_i": [], "xy_j": [], "inlier_mask": []}
    except subprocess.TimeoutExpired:
        return {"error": "worker timed out (300s)", "T_ij": None,
                "n_inliers": 0, "rms": None, "xy_i": [], "xy_j": [], "inlier_mask": []}
    except Exception as exc:
        return {"error": str(exc), "T_ij": None,
                "n_inliers": 0, "rms": None, "xy_i": [], "xy_j": [], "inlier_mask": []}


# ══════════════════════════════════════════════════════════════════════════════
# Callback functions (all Gradio event handlers)
# ══════════════════════════════════════════════════════════════════════════════

def cb_load_frame_a(slider_a: int):
    """Frame A scrubber changed."""
    _require_db()
    node_id = _node_id_at_index(int(slider_a))
    thumb = read_color_thumbnail(_db_path, node_id)
    scatter = _draw_scatter_with_highlight(node_id, None)
    return thumb, scatter, str(node_id)


def cb_load_frame_b(slider_b: int):
    """Frame B scrubber changed."""
    _require_db()
    node_id = _node_id_at_index(int(slider_b))
    thumb = read_color_thumbnail(_db_path, node_id)
    scatter = _draw_scatter_with_highlight(None, node_id)
    return thumb, str(node_id)


def cb_propose_candidates(slider_a: int):
    """Propose candidate B frames for the current A frame."""
    _require_db()
    node_a = _node_id_at_index(int(slider_a))
    candidates = propose_candidates(_positions, node_a, n_candidates=15)
    if not candidates:
        return "No candidates found (not enough frames or no pose data)."
    lines = [f"Candidates for frame {node_a} (by seed-pose proximity):"]
    for nid, dist in candidates[:10]:
        lines.append(f"  node_id={nid:6d}  dist={dist:.2f}m")
    return "\n".join(lines)


def cb_match(slider_a: int, slider_b: int, method: str):
    """Run a match on the selected frame pair."""
    global _last_match
    _require_db()
    node_i = _node_id_at_index(int(slider_a))
    node_j = _node_id_at_index(int(slider_b))

    if node_i == node_j:
        return None, "Frames A and B are the same — select different frames.", ""

    # Dispatch to appropriate backend
    if method in ("orb", "sift", "akaze"):
        match = match_pair(_db_path, node_i, node_j, method=method)
    elif method in ("xfeat", "mast3r"):
        match = _run_learned_matcher(method, _db_path, node_i, node_j)
    elif method == "icp":
        result = run_icp_on_frames(_db_path, node_i, node_j)
        match = {
            "error": result.get("error"),
            "T_ij": result.get("T_ij"),
            "n_inliers": 0,
            "rms": result.get("rms"),
            "xy_i": [], "xy_j": [], "inlier_mask": [],
        }
    else:
        return None, f"Unknown method: {method}", ""

    _last_match = match

    # Error path
    if match.get("error"):
        return None, f"Error: {match['error']}", ""

    # Draw correspondences (only for feature methods)
    img = None
    if match.get("xy_i"):
        try:
            img = _draw_correspondences(match, _db_path, node_i, node_j)
        except Exception as exc:
            pass  # correspondence drawing is non-critical

    rms_str = f"{match['rms']*1000:.1f}mm" if match.get("rms") else "—"
    T = match.get("T_ij")
    t_vec = np.array(T).reshape(4, 4)[:3, 3] if T else None
    status = (
        f"node_i={node_i}, node_j={node_j}, method={method}\n"
        f"n_inliers={match.get('n_inliers', 0)}, RMS={rms_str}\n"
    )
    if t_vec is not None:
        status += f"T_ij translation: [{t_vec[0]:.3f}, {t_vec[1]:.3f}, {t_vec[2]:.3f}]"

    T_display = json.dumps(T, indent=2) if T else "none"
    return img, status, T_display


def cb_run_icp(slider_a: int, slider_b: int, dx: float, dy: float, dz: float, dyaw: float):
    """Run ICP with a coarse slider-based initial guess."""
    global _last_match
    _require_db()
    node_i = _node_id_at_index(int(slider_a))
    node_j = _node_id_at_index(int(slider_b))
    T_init = sliders_to_T(dx, dy, dz, dyaw)
    result = run_icp_on_frames(_db_path, node_i, node_j, T_init=T_init)

    _last_match = {
        "error": result.get("error"),
        "T_ij": result.get("T_ij"),
        "n_inliers": 0,
        "rms": result.get("rms"),
        "xy_i": [], "xy_j": [], "inlier_mask": [],
        "method": "icp",
    }

    if result.get("error"):
        return f"ICP error: {result['error']}", ""

    rms_m = result.get("rms", 0.0)
    overlap = result.get("overlap", 0.0)
    T = result.get("T_ij")
    status = (
        f"ICP done: RMS={rms_m*1000:.1f}mm, overlap={overlap*100:.1f}%\n"
        f"node_i={node_i}, node_j={node_j}"
    )
    T_display = json.dumps(T, indent=2) if T else "none"
    return status, T_display


def cb_accept(slider_a: int, slider_b: int, method: str):
    """Accept the last match result and append to the anchor list."""
    global _last_match
    _require_db()
    if _last_match is None or _last_match.get("T_ij") is None:
        return "No valid match to accept. Run a match first.", _anchors.to_display_rows()

    node_i = _node_id_at_index(int(slider_a))
    node_j = _node_id_at_index(int(slider_b))
    _anchors.add(
        node_i=node_i,
        node_j=node_j,
        T_ij=_last_match["T_ij"],
        method=method,
        rms=_last_match.get("rms"),
        n_inliers=_last_match.get("n_inliers", 0),
    )
    _last_match = None
    return f"Accepted anchor {len(_anchors)-1}: {node_i} ↔ {node_j}", _anchors.to_display_rows()


def cb_remove_anchor(index_str: str):
    """Remove anchor at the given index."""
    try:
        idx = int(index_str)
    except ValueError:
        return "Invalid index.", _anchors.to_display_rows()
    _anchors.remove(idx)
    return f"Removed anchor at index {idx}.", _anchors.to_display_rows()


def cb_export_anchors(export_path: str):
    """Export the anchor list to JSON."""
    if not export_path.strip():
        return "Please enter a file path."
    try:
        _anchors.export_json(export_path.strip())
        return f"Exported {len(_anchors)} anchors to {export_path}"
    except Exception as exc:
        return f"Export failed: {exc}"


def cb_import_anchors(import_path: str):
    """Import anchors from JSON."""
    if not import_path.strip():
        return "Please enter a file path.", _anchors.to_display_rows()
    try:
        n = _anchors.import_json(import_path.strip())
        return f"Imported {n} anchors from {import_path}", _anchors.to_display_rows()
    except Exception as exc:
        return f"Import failed: {exc}", _anchors.to_display_rows()


def cb_preview_gt(seed_rux: str, export_path: str, anchor_max_idx_str: str):
    """Trigger live GT preview: solve → rux create clouds → rux render."""
    _require_db()
    if len(_anchors) == 0:
        return "No anchors to preview with. Accept at least one pair first.", None, None

    seed_path = Path(seed_rux.strip() if seed_rux.strip() else _db_path)
    if not seed_path.exists():
        return f"Seed .rux not found: {seed_path}", None, None

    # Write current anchors to a temp file
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
        edges_path = tf.name
    _anchors.export_json(edges_path)

    # Output .rux in a temp dir
    with tempfile.TemporaryDirectory() as tmpdir:
        out_rux = Path(tmpdir) / "gt_preview.rux"

        # Parse anchor_max_idx
        anchor_max_idx_args = []
        try:
            amax = int(anchor_max_idx_str.strip())
            anchor_max_idx_args = ["--anchor-max-idx", str(amax)]
        except (ValueError, AttributeError):
            pass

        # 1) Solve
        solve_cmd = [
            str(SOLVE_VENV),
            str(SOLVE_CLI),
            "--seed", str(seed_path),
            "--edges", edges_path,
            "--out", str(out_rux),
        ] + anchor_max_idx_args

        try:
            r = subprocess.run(
                solve_cmd, capture_output=True, text=True, timeout=600,
                env=_subprocess_env(venv_site=_SOLVE_VENV_SITE)
            )
        except subprocess.TimeoutExpired:
            return "Solve timed out (600s).", None, None
        finally:
            Path(edges_path).unlink(missing_ok=True)

        if r.returncode != 0:
            err = r.stderr.strip().splitlines()[-1] if r.stderr else "unknown"
            return f"Solve failed: {err}\n\nSTDERR:\n{r.stderr[-2000:]}", None, None

        if not out_rux.exists():
            return "Solve produced no output .rux.", None, None

        # Extract summary
        summary_line = ""
        for line in r.stdout.splitlines():
            if line.startswith("SOLVE_SUMMARY:"):
                try:
                    summary = json.loads(line[len("SOLVE_SUMMARY:"):])
                    summary_line = (
                        f"Solve OK: {summary['n_frames']} frames, "
                        f"{summary['n_loop']} loop edges, "
                        f"max_shift={summary['max_shift_m']*1000:.0f}mm, "
                        f"elapsed={summary['elapsed_s']:.0f}s"
                    )
                except Exception:
                    pass

        # 2) Create clouds (on GT .rux)
        rux_cmd_base = [str(RUX_BIN), "-p", str(out_rux)]
        try:
            subprocess.run(
                rux_cmd_base + ["create", "clouds", "-g", "0.05"],
                capture_output=True, timeout=600, env=_subprocess_env(venv_site=None)
            )
        except subprocess.TimeoutExpired:
            pass  # clouds optional for preview

        # 3) Render: seed before, GT after
        before_png = Path(tmpdir) / "before.png"
        after_png = Path(tmpdir) / "after.png"

        for rux_file, png_file in [(str(seed_path), str(before_png)),
                                   (str(out_rux), str(after_png))]:
            try:
                subprocess.run(
                    [str(RUX_BIN), "-p", rux_file,
                     "render", "-o", png_file, "--view", "top", "--layers", "cloud"],
                    capture_output=True, timeout=120, env=_subprocess_env(venv_site=None)
                )
            except subprocess.TimeoutExpired:
                pass

        before_img = cv2.imread(str(before_png))
        after_img = cv2.imread(str(after_png))

        if before_img is not None:
            before_img = cv2.cvtColor(before_img, cv2.COLOR_BGR2RGB)
        if after_img is not None:
            after_img = cv2.cvtColor(after_img, cv2.COLOR_BGR2RGB)

        status = summary_line or "Solve completed (no summary parsed)."
        return status, before_img, after_img


# ══════════════════════════════════════════════════════════════════════════════
# Gradio UI definition
# ══════════════════════════════════════════════════════════════════════════════

def build_ui(db_path_arg: str) -> gr.Blocks:
    global _db_path, _positions, _node_ids

    _db_path = db_path_arg
    _positions = read_seed_positions(_db_path)
    _node_ids = [nid for nid, _ in _positions]
    n_frames = len(_node_ids)

    initial_scatter = _scatter_plot_data()
    first_thumb_a = read_color_thumbnail(_db_path, _node_ids[0]) if _node_ids else None
    first_thumb_b = read_color_thumbnail(_db_path, _node_ids[min(200, n_frames - 1)]) if _node_ids else None

    with gr.Blocks(title="GT Curator — ReUseX #221") as demo:
        gr.Markdown(f"# GT Curator — {Path(db_path_arg).name}  ({n_frames} frames)")

        # ── Row: scatter + frame panels ─────────────────────────────────────
        with gr.Row():
            with gr.Column(scale=1):
                gr.Markdown("### Top-down map")
                scatter_img = gr.Image(
                    value=initial_scatter, label="Frame positions (seed poses)",
                    height=380, show_label=True
                )

            with gr.Column(scale=1):
                gr.Markdown("### Frame A")
                slider_a = gr.Slider(0, n_frames - 1, value=0, step=1, label="Frame A index")
                thumb_a = gr.Image(value=first_thumb_a, label="A color", height=180)
                node_id_a = gr.Textbox(value=str(_node_ids[0] if _node_ids else 0),
                                       label="node_id A", interactive=False)

            with gr.Column(scale=1):
                gr.Markdown("### Frame B")
                slider_b = gr.Slider(0, n_frames - 1, value=min(200, n_frames - 1),
                                     step=1, label="Frame B index")
                thumb_b = gr.Image(value=first_thumb_b, label="B color", height=180)
                node_id_b = gr.Textbox(
                    value=str(_node_ids[min(200, n_frames - 1)] if _node_ids else 0),
                    label="node_id B", interactive=False
                )

        # Candidate proposal
        with gr.Row():
            propose_btn = gr.Button("Propose candidates for A")
            candidates_out = gr.Textbox(label="Candidates", lines=6)

        # ── Verification panel ───────────────────────────────────────────────
        gr.Markdown("---\n## Verification")
        with gr.Row():
            method_dd = gr.Dropdown(
                choices=["orb", "sift", "akaze", "xfeat", "mast3r", "icp"],
                value="orb",
                label="Method",
            )
            match_btn = gr.Button("Run match", variant="primary")

        corr_img = gr.Image(label="Correspondences (green=inlier, red=rejected)", height=360)
        match_status = gr.Textbox(label="Match status", lines=4)
        T_display = gr.Textbox(label="T_ij (4×4, flat)", lines=4)

        # ── ICP panel ───────────────────────────────────────────────────────
        with gr.Accordion("Manual + ICP (coarse nudge then refine)", open=False):
            gr.Markdown(
                "Adjust the coarse pose of frame B relative to A (gravity-locked: "
                "rotation = yaw only), then click Run ICP."
            )
            with gr.Row():
                sl_dx = gr.Slider(-5, 5, value=0, step=0.05, label="Δx (m)")
                sl_dy = gr.Slider(-5, 5, value=0, step=0.05, label="Δy (m)")
                sl_dz = gr.Slider(-3, 3, value=0, step=0.05, label="Δz (m)")
                sl_yaw = gr.Slider(-180, 180, value=0, step=1, label="Δyaw (°)")
            icp_btn = gr.Button("Run ICP")
            icp_status = gr.Textbox(label="ICP status", lines=2)
            icp_T = gr.Textbox(label="ICP T_ij", lines=4)

        # ── Accept / Anchor list ─────────────────────────────────────────────
        gr.Markdown("---\n## Anchor list")
        with gr.Row():
            accept_btn = gr.Button("Accept last match", variant="primary")
            accept_status = gr.Textbox(label="", lines=1)

        anchor_table = gr.Dataframe(
            headers=["#", "node_i", "node_j", "method", "inliers", "RMS (mm)"],
            value=_anchors.to_display_rows(),
            label="Accepted anchors",
        )

        with gr.Row():
            remove_idx = gr.Textbox(label="Remove anchor at index", value="0")
            remove_btn = gr.Button("Remove")
            remove_status = gr.Textbox(label="", lines=1)

        with gr.Row():
            export_path = gr.Textbox(
                label="Export JSON path",
                value=str(Path(_db_path).parent / "gt_anchors.json"),
            )
            export_btn = gr.Button("Export anchors")
            export_status = gr.Textbox(label="", lines=1)

        with gr.Row():
            import_path = gr.Textbox(label="Import JSON path", value="")
            import_btn = gr.Button("Import anchors")
            import_status = gr.Textbox(label="", lines=1)

        # ── Live GT preview ───────────────────────────────────────────────────
        gr.Markdown("---\n## Live GT preview (on-demand)")
        gr.Markdown(
            "Runs solve_gt_poses_cli.py on a copy of the seed, then `rux create clouds` "
            "and `rux render --view top`. This takes a couple of minutes on a full scan."
        )
        with gr.Row():
            seed_rux_path = gr.Textbox(
                label="Seed .rux path",
                value=str(Path(_db_path).parent / "pseudo-gt" / "newoffice_pgt_before_seed.rux"),
            )
            anchor_max_idx_input = gr.Textbox(
                label="Anchor zone max sequential idx (optional, e.g. 567)",
                value="567",
            )
        preview_btn = gr.Button("Run GT preview (slow — ~2 min)", variant="secondary")
        preview_status = gr.Textbox(label="Preview status", lines=3)
        with gr.Row():
            before_img = gr.Image(label="Before (seed)", height=360)
            after_img = gr.Image(label="After (GT)", height=360)

        # ── Wiring ───────────────────────────────────────────────────────────
        slider_a.change(cb_load_frame_a, [slider_a], [thumb_a, scatter_img, node_id_a])
        slider_b.change(cb_load_frame_b, [slider_b], [thumb_b, node_id_b])
        propose_btn.click(cb_propose_candidates, [slider_a], [candidates_out])
        match_btn.click(cb_match, [slider_a, slider_b, method_dd],
                        [corr_img, match_status, T_display])
        icp_btn.click(cb_run_icp, [slider_a, slider_b, sl_dx, sl_dy, sl_dz, sl_yaw],
                      [icp_status, icp_T])
        accept_btn.click(cb_accept, [slider_a, slider_b, method_dd],
                         [accept_status, anchor_table])
        remove_btn.click(cb_remove_anchor, [remove_idx], [remove_status, anchor_table])
        export_btn.click(cb_export_anchors, [export_path], [export_status])
        import_btn.click(cb_import_anchors, [import_path], [import_status, anchor_table])
        preview_btn.click(
            cb_preview_gt, [seed_rux_path, export_path, anchor_max_idx_input],
            [preview_status, before_img, after_img]
        )

    return demo


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="GT Curator — Gradio tool for curating anchors")
    ap.add_argument("-p", "--project", required=True, help="path to .rux database")
    ap.add_argument("--port", type=int, default=7860)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    db = Path(args.project)
    if not db.exists():
        sys.exit(f"Error: project not found: {db}")

    print(f"[gt_curator] Loading {db} …")
    n = read_frame_count(str(db))
    print(f"[gt_curator] {n} frames found")

    demo = build_ui(str(db))
    demo.launch(
        server_name=args.host,
        server_port=args.port,
        inbrowser=not args.no_browser,
        share=False,
    )


if __name__ == "__main__":
    main()
