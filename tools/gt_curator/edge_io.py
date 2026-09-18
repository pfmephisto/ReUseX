# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Load prior-computed edge JSON files (from xfeat, mast3r, spatial-filter runs)
into a browsable list for the GT Curator.

Supported schema flavours (all share `node_i`, `node_j`, `T_ij`):
  - reusex.loop_edges.v1  — `inliers` key (int)
  - reusex.gt_anchors.v1  — `n_inliers` key, also has `method`, `rms`
  - Any dict with `node_i`/`node_j` at the top level or nested under `edges`

A loaded edge carries: node_i, node_j, n_inliers/score, T_ij (may be None),
source file, and any extra notes (e.g. revisit_hint, sigma_trans).
"""

import json
from pathlib import Path
from typing import Optional


# ── public interface ──────────────────────────────────────────────────────────

class LoadedEdges:
    """In-memory list of edges loaded from prior-computed JSON files."""

    def __init__(self):
        self._edges: list[dict] = []  # raw dicts with normalised keys

    def clear(self) -> None:
        self._edges.clear()

    def load(self, path_str: str) -> tuple[int, list[str]]:
        """Load edges from a file or directory.  Returns (n_added, warnings)."""
        p = Path(path_str.strip())
        warnings: list[str] = []
        added = 0

        if p.is_dir():
            jsons = sorted(p.glob("*.json"))
            if not jsons:
                return 0, [f"No JSON files found in {p}"]
            for f in jsons:
                n, w = self._load_file(f)
                added += n
                warnings.extend(w)
        elif p.is_file():
            n, w = self._load_file(p)
            added += n
            warnings.extend(w)
        else:
            return 0, [f"Path not found: {p}"]

        return added, warnings

    def to_display_rows(self) -> list[list]:
        """Rows for gr.Dataframe: [#, node_i, node_j, n_inliers, source]."""
        rows = []
        for k, e in enumerate(self._edges):
            rows.append([
                k,
                e["node_i"],
                e["node_j"],
                e.get("n_inliers", "—"),
                e.get("source", ""),
            ])
        return rows

    def get(self, index: int) -> Optional[dict]:
        if 0 <= index < len(self._edges):
            return self._edges[index]
        return None

    def __len__(self) -> int:
        return len(self._edges)

    # ── private ──────────────────────────────────────────────────────────────

    def _load_file(self, path: Path) -> tuple[int, list[str]]:
        warnings: list[str] = []
        try:
            doc = json.loads(path.read_text())
        except Exception as exc:
            return 0, [f"{path.name}: parse error — {exc}"]

        # Extract the edge list
        if isinstance(doc, list):
            raw_edges = doc
        elif isinstance(doc, dict):
            raw_edges = doc.get("edges", [])
            if not raw_edges:
                # Maybe the file *is* a single edge dict?
                if "node_i" in doc and "node_j" in doc:
                    raw_edges = [doc]
        else:
            return 0, [f"{path.name}: unrecognised top-level type"]

        n_before = len(self._edges)
        for e in raw_edges:
            norm = _normalise_edge(e, source=path.name)
            if norm is not None:
                self._edges.append(norm)
            else:
                warnings.append(f"{path.name}: skipped edge missing node_i/node_j")

        return len(self._edges) - n_before, warnings


def _normalise_edge(e: dict, source: str) -> Optional[dict]:
    """Normalise any supported edge schema to a common dict."""
    if not isinstance(e, dict):
        return None
    node_i = e.get("node_i")
    node_j = e.get("node_j")
    if node_i is None or node_j is None:
        return None

    # n_inliers: various spellings
    n_inliers = e.get("n_inliers") or e.get("inliers") or e.get("num_inliers") or 0
    if isinstance(n_inliers, float):
        n_inliers = int(n_inliers)

    T_ij = e.get("T_ij")  # may be None — loaded edges are pre-computed candidates
    rms = e.get("rms")
    method = e.get("method", "")
    sigma_trans = e.get("sigma_trans")

    return {
        "node_i": int(node_i),
        "node_j": int(node_j),
        "n_inliers": n_inliers,
        "T_ij": T_ij,
        "rms": rms,
        "method": method,
        "sigma_trans": sigma_trans,
        "source": source,
        "revisit_hint": e.get("revisit_hint"),
    }
