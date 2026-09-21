# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Plotly-based interactive top-down graph view for the GT Curator.

Replaces the static PIL scatter with a Plotly Figure that supports
native zoom/pan.  Click-to-select is NOT wired at the Python level
because gr.Plot in Gradio 6.27 only exposes a .change event (no .select).
The sliders remain the primary A/B selection mechanism; this module
provides the visual overlay.

build_figure(positions, node_a, node_b, anchors, loaded_edges)
  → plotly.graph_objects.Figure
"""

from typing import Optional

import numpy as np

try:
    import plotly.graph_objects as go
    _PLOTLY_OK = True
except ImportError:
    _PLOTLY_OK = False


def build_figure(
    positions: list[tuple[int, object]],
    node_a: Optional[int] = None,
    node_b: Optional[int] = None,
    anchor_edges: Optional[list[dict]] = None,
    loaded_edges: Optional[list[dict]] = None,
) -> object:
    """Return a Plotly Figure for the top-down frame scatter.

    Parameters
    ----------
    positions:     [(node_id, xyz_or_None)] — from read_seed_positions()
    node_a:        currently selected Frame A node_id (highlighted red)
    node_b:        currently selected Frame B node_id (highlighted green)
    anchor_edges:  list of accepted anchor dicts (node_i, node_j) for overlay
    loaded_edges:  list of loaded prior-edge dicts (node_i, node_j) for overlay

    Returns
    -------
    plotly.graph_objects.Figure or None if plotly is unavailable or no positions.
    """
    if not _PLOTLY_OK:
        return None

    pts = [(nid, xyz) for nid, xyz in positions if xyz is not None]
    if not pts:
        return None

    pos_map: dict[int, np.ndarray] = {nid: xyz for nid, xyz in pts}

    node_ids = [nid for nid, _ in pts]
    xs = [float(xyz[0]) for _, xyz in pts]
    ys = [float(xyz[1]) for _, xyz in pts]

    fig = go.Figure()

    # ── base scatter: all frames ──────────────────────────────────────────────
    fig.add_trace(go.Scattergl(
        x=xs,
        y=ys,
        mode="markers",
        marker=dict(size=4, color="rgb(100,120,200)", opacity=0.6),
        text=[f"node_id={nid}" for nid in node_ids],
        hovertemplate="%{text}<br>x=%{x:.2f} y=%{y:.2f}<extra></extra>",
        name="frames",
        showlegend=False,
    ))

    # ── loaded prior-edge lines (faint) ───────────────────────────────────────
    if loaded_edges:
        edge_xs: list[Optional[float]] = []
        edge_ys: list[Optional[float]] = []
        for e in loaded_edges:
            pi = pos_map.get(e["node_i"])
            pj = pos_map.get(e["node_j"])
            if pi is not None and pj is not None:
                edge_xs += [float(pi[0]), float(pj[0]), None]
                edge_ys += [float(pi[1]), float(pj[1]), None]
        if edge_xs:
            fig.add_trace(go.Scattergl(
                x=edge_xs,
                y=edge_ys,
                mode="lines",
                line=dict(color="rgba(160,160,220,0.3)", width=1),
                hoverinfo="skip",
                name="loaded edges",
                showlegend=len(loaded_edges) > 0,
            ))

    # ── accepted anchor lines (solid) ────────────────────────────────────────
    if anchor_edges:
        anc_xs: list[Optional[float]] = []
        anc_ys: list[Optional[float]] = []
        for e in anchor_edges:
            pi = pos_map.get(e.get("node_i", -1))
            pj = pos_map.get(e.get("node_j", -1))
            if pi is not None and pj is not None:
                anc_xs += [float(pi[0]), float(pj[0]), None]
                anc_ys += [float(pi[1]), float(pj[1]), None]
        if anc_xs:
            fig.add_trace(go.Scattergl(
                x=anc_xs,
                y=anc_ys,
                mode="lines",
                line=dict(color="rgba(80,200,80,0.85)", width=2),
                hoverinfo="skip",
                name="accepted anchors",
                showlegend=True,
            ))

    # ── A/B highlight markers ────────────────────────────────────────────────
    for nid, colour, label in [
        (node_a, "rgb(220,60,60)", "A"),
        (node_b, "rgb(60,200,60)", "B"),
    ]:
        if nid is not None and nid in pos_map:
            xyz = pos_map[nid]
            fig.add_trace(go.Scattergl(
                x=[float(xyz[0])],
                y=[float(xyz[1])],
                mode="markers+text",
                marker=dict(size=12, color=colour, symbol="circle"),
                text=[label],
                textposition="top center",
                textfont=dict(color=colour, size=11),
                hovertemplate=f"Frame {label}: node_id={nid}<extra></extra>",
                name=f"Frame {label}",
                showlegend=True,
            ))

    fig.update_layout(
        margin=dict(l=20, r=20, t=30, b=20),
        paper_bgcolor="rgb(245,245,248)",
        plot_bgcolor="rgb(245,245,248)",
        xaxis=dict(showgrid=True, gridcolor="rgb(220,220,220)", title="X (m)"),
        yaxis=dict(
            showgrid=True, gridcolor="rgb(220,220,220)", title="Y (m)",
            scaleanchor="x", scaleratio=1,  # keep aspect ratio
        ),
        legend=dict(x=0.01, y=0.99, bgcolor="rgba(255,255,255,0.7)"),
        height=400,
    )

    return fig
