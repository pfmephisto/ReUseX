# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Anchor list management and JSON export for the GT curator.

The exported JSON schema matches what solve_gt_poses_cli.py consumes:
  {
    "schema": "reusex.gt_anchors.v1",
    "convention": "T_ij = pose(i)^-1 * pose(j), ...",
    "edges": [
      {
        "node_i": int,
        "node_j": int,
        "T_ij": [16 floats, row-major 4x4],
        "method": str,
        "rms": float or null,
        "n_inliers": int,
        "sigma_trans": float,
        "sigma_rot": float
      },
      ...
    ]
  }

This schema is deliberately a superset of the loop_edges.v1 schema so that
solve_gt_poses_cli.py can consume it directly via its --edges flag.
"""

import json
import math
from pathlib import Path

import numpy as np

SCHEMA = "reusex.gt_anchors.v1"
CONVENTION = (
    "T_ij = pose(i)^-1 * pose(j), optical->world; node ids are sensor_frames.node_id"
)

# Conservative noise model for manually-curated high-quality anchors
DEFAULT_SIGMA_TRANS = 0.05  # m
DEFAULT_SIGMA_ROT = 0.02    # rad


def _compute_sigmas(n_inliers: int | None, rms: float | None) -> tuple[float, float]:
    """Scale sigma by inlier count; fall back to default."""
    if n_inliers and n_inliers >= 3:
        scale = max(40 / max(n_inliers, 1), 0.5) ** 0.5
        sig_t = max(DEFAULT_SIGMA_TRANS * scale, 0.02)
        sig_r = max(DEFAULT_SIGMA_ROT * scale, 0.01)
        return sig_t, sig_r
    return DEFAULT_SIGMA_TRANS, DEFAULT_SIGMA_ROT


class AnchorList:
    """In-memory list of accepted anchor edges."""

    def __init__(self):
        self._edges: list[dict] = []

    def add(
        self,
        node_i: int,
        node_j: int,
        T_ij: np.ndarray | list,
        method: str,
        rms: float | None,
        n_inliers: int = 0,
    ) -> None:
        """Append an anchor. Duplicate (i, j) pairs are allowed (curators may
        re-do a pair with a different method and keep both for analysis)."""
        if isinstance(T_ij, np.ndarray):
            T_flat = T_ij.flatten().tolist()
        else:
            # Accept flat list, nested 4x4, or any nested iterable
            import itertools
            def _flatten(lst):
                for item in lst:
                    try:
                        yield from _flatten(item)
                    except TypeError:
                        yield item
            T_flat = [float(x) for x in _flatten(T_ij)]
        if len(T_flat) != 16:
            raise ValueError(f"T_ij must be 16 elements, got {len(T_flat)}")

        sig_t, sig_r = _compute_sigmas(n_inliers, rms)
        self._edges.append({
            "node_i": int(node_i),
            "node_j": int(node_j),
            "T_ij": T_flat,
            "method": str(method),
            "rms": float(rms) if rms is not None else None,
            "n_inliers": int(n_inliers),
            "sigma_trans": sig_t,
            "sigma_rot": sig_r,
        })

    def remove(self, index: int) -> None:
        if 0 <= index < len(self._edges):
            self._edges.pop(index)

    def clear(self) -> None:
        self._edges.clear()

    def __len__(self) -> int:
        return len(self._edges)

    def as_list(self) -> list[dict]:
        return list(self._edges)

    def to_display_rows(self) -> list[list]:
        """Return rows for a Gradio dataframe: [#, i, j, method, inliers, rms_mm]."""
        rows = []
        for k, e in enumerate(self._edges):
            rms_mm = f"{e['rms']*1000:.1f}" if e["rms"] is not None else "—"
            rows.append([k, e["node_i"], e["node_j"], e["method"], e["n_inliers"], rms_mm])
        return rows

    def export_json(self, path: str) -> None:
        """Write the anchor list to JSON in the solve_gt_poses_cli schema."""
        doc = {
            "schema": SCHEMA,
            "convention": CONVENTION,
            "edges": self._edges,
        }
        Path(path).write_text(json.dumps(doc, indent=2))

    def import_json(self, path: str) -> int:
        """Load anchors from a previously exported JSON; returns number loaded."""
        doc = json.loads(Path(path).read_text())
        edges = doc.get("edges", [])
        for e in edges:
            self._edges.append(e)
        return len(edges)
