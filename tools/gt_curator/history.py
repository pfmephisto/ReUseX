# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Session match history for the GT Curator.

Tracks every match invocation (one row per cb_match call), separate from the
accepted-anchor list.  Rows are appended on each match and can be re-opened
by selecting them in the UI (which sets A/B back to that pair).
"""

from typing import Optional


class MatchHistory:
    """In-memory rolling list of match attempts this session."""

    def __init__(self):
        self._rows: list[dict] = []

    def append(
        self,
        node_i: int,
        node_j: int,
        method: str,
        n_inliers: int,
        rms: Optional[float],
        accepted: bool = False,
    ) -> None:
        """Append one match attempt."""
        self._rows.append({
            "node_i": int(node_i),
            "node_j": int(node_j),
            "method": str(method),
            "n_inliers": int(n_inliers),
            "rms": float(rms) if rms is not None else None,
            "accepted": accepted,
        })

    def mark_last_accepted(self) -> None:
        """Mark the most-recent row as accepted (called after cb_accept)."""
        if self._rows:
            self._rows[-1]["accepted"] = True

    def to_display_rows(self) -> list[list]:
        """Rows for gr.Dataframe: [#, node_i, node_j, method, inliers, rms_mm, accepted]."""
        rows = []
        for k, r in enumerate(self._rows):
            rms_mm = f"{r['rms']*1000:.1f}" if r["rms"] is not None else "—"
            rows.append([
                k,
                r["node_i"],
                r["node_j"],
                r["method"],
                r["n_inliers"],
                rms_mm,
                "yes" if r["accepted"] else "no",
            ])
        return rows

    def get(self, index: int) -> Optional[dict]:
        if 0 <= index < len(self._rows):
            return self._rows[index]
        return None

    def __len__(self) -> int:
        return len(self._rows)
