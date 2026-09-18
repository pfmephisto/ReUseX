# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Unit tests for the new GT Curator features added in the interactive-graph
enhancement (issue #221):

  - edge_io.LoadedEdges: parse real pseudo-gt JSON files, normalise schemas
  - history.MatchHistory: append / select / mark-accepted
  - graph_view.build_figure: returns a valid Plotly Figure given positions + overlays

Tests are PURE logic where possible.  The three tests that touch real JSON
files require /home/mephisto/repos/NewOffice/pseudo-gt to exist; they are
skipped gracefully if the path is absent.
"""

import json
import sys
import tempfile
from pathlib import Path
from typing import Optional

import numpy as np
import pytest

# ── path setup ────────────────────────────────────────────────────────────────
_HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_HERE))

from anchors import AnchorList  # noqa: E402
from edge_io import LoadedEdges, LoadedEdges as _LE, _normalise_edge  # noqa: E402
from history import MatchHistory  # noqa: E402

_PSEUDO_GT = Path("/home/mephisto/repos/NewOffice/pseudo-gt")
_EDGES_DIR = _PSEUDO_GT / "edges"
_TARGETED_DIR = _PSEUDO_GT / "targeted-run"

_HAS_PSEUDO_GT = _PSEUDO_GT.is_dir()


# ══════════════════════════════════════════════════════════════════════════════
# edge_io tests
# ══════════════════════════════════════════════════════════════════════════════

class TestNormaliseEdge:
    def test_basic_edge(self):
        e = {"node_i": 10, "node_j": 200, "inliers": 42, "T_ij": list(range(16))}
        norm = _normalise_edge(e, source="test.json")
        assert norm is not None
        assert norm["node_i"] == 10
        assert norm["node_j"] == 200
        assert norm["n_inliers"] == 42
        assert norm["source"] == "test.json"

    def test_n_inliers_key(self):
        e = {"node_i": 1, "node_j": 2, "n_inliers": 77}
        norm = _normalise_edge(e, source="x.json")
        assert norm["n_inliers"] == 77

    def test_missing_nodes_returns_none(self):
        e = {"inliers": 5}
        assert _normalise_edge(e, source="x.json") is None

    def test_node_id_coerced_to_int(self):
        e = {"node_i": 3.0, "node_j": 7.0}
        norm = _normalise_edge(e, source="x.json")
        assert isinstance(norm["node_i"], int)
        assert isinstance(norm["node_j"], int)

    def test_none_T_ij_preserved(self):
        e = {"node_i": 1, "node_j": 2}
        norm = _normalise_edge(e, source="x.json")
        assert norm["T_ij"] is None


class TestLoadedEdgesSynthetic:
    def _write_edge_file(self, edges: list[dict], tmpdir: Path) -> Path:
        doc = {
            "schema": "reusex.loop_edges.v1",
            "convention": "test",
            "edges": edges,
        }
        p = tmpdir / "test_edges.json"
        p.write_text(json.dumps(doc))
        return p

    def test_load_single_file(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            raw = [
                {"node_i": 10, "node_j": 200, "inliers": 55, "T_ij": list(range(16))},
                {"node_i": 20, "node_j": 400, "inliers": 33},
            ]
            p = self._write_edge_file(raw, td)
            le = LoadedEdges()
            n, warnings = le.load(str(p))
            assert n == 2
            assert len(warnings) == 0
            assert len(le) == 2

    def test_display_rows_columns(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            raw = [{"node_i": 5, "node_j": 55, "inliers": 12}]
            p = self._write_edge_file(raw, td)
            le = LoadedEdges()
            le.load(str(p))
            rows = le.to_display_rows()
            assert len(rows) == 1
            assert rows[0][1] == 5    # node_i
            assert rows[0][2] == 55   # node_j
            assert rows[0][3] == 12   # n_inliers
            assert "test_edges.json" in rows[0][4]  # source

    def test_load_directory(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            for i, stem in enumerate(["aa", "bb"]):
                raw = [{"node_i": i, "node_j": i + 100, "inliers": i * 10}]
                p = self._write_edge_file(raw, td)
                p.rename(td / f"{stem}.json")
            le = LoadedEdges()
            n, warnings = le.load(str(td))
            assert n == 2

    def test_load_nonexistent_path(self):
        le = LoadedEdges()
        n, w = le.load("/nonexistent/path/edges.json")
        assert n == 0
        assert len(w) == 1

    def test_clear(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            p = self._write_edge_file([{"node_i": 1, "node_j": 2}], td)
            le = LoadedEdges()
            le.load(str(p))
            assert len(le) == 1
            le.clear()
            assert len(le) == 0

    def test_get(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            p = self._write_edge_file([
                {"node_i": 7, "node_j": 77, "inliers": 5},
                {"node_i": 8, "node_j": 88, "inliers": 6},
            ], td)
            le = LoadedEdges()
            le.load(str(p))
            e = le.get(1)
            assert e is not None
            assert e["node_i"] == 8

    def test_invalid_json_returns_warning(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            p = td / "bad.json"
            p.write_text("{ not valid json ]]]")
            le = LoadedEdges()
            n, w = le.load(str(p))
            assert n == 0
            assert len(w) == 1


@pytest.mark.skipif(not _HAS_PSEUDO_GT, reason="pseudo-gt dataset not present")
class TestLoadedEdgesRealFiles:
    def test_load_xfeat_edges(self):
        p = _EDGES_DIR / "xfeat_edges.json"
        le = LoadedEdges()
        n, warnings = le.load(str(p))
        assert n > 0, f"Expected edges from {p}"
        assert len(warnings) == 0
        e0 = le.get(0)
        assert e0 is not None
        assert isinstance(e0["node_i"], int)
        assert isinstance(e0["node_j"], int)

    def test_load_mast3r_edges(self):
        p = _EDGES_DIR / "mast3r_edges_NONCOMMERCIAL.json"
        le = LoadedEdges()
        n, warnings = le.load(str(p))
        assert n > 0
        e0 = le.get(0)
        assert e0["node_i"] is not None

    def test_load_targeted_run_files(self):
        for fname in ["spatial_filtered.json", "mast3r_targeted_revisits.json"]:
            p = _TARGETED_DIR / fname
            if not p.exists():
                continue
            le = LoadedEdges()
            n, warnings = le.load(str(p))
            assert n > 0, f"No edges loaded from {fname}"
            e0 = le.get(0)
            assert "node_i" in e0
            assert "node_j" in e0

    def test_load_entire_edges_directory(self):
        le = LoadedEdges()
        n, warnings = le.load(str(_EDGES_DIR))
        assert n > 0
        # All edges should have node_i/node_j
        for k in range(len(le)):
            e = le.get(k)
            assert e is not None
            assert isinstance(e["node_i"], int)
            assert isinstance(e["node_j"], int)


# ══════════════════════════════════════════════════════════════════════════════
# history tests
# ══════════════════════════════════════════════════════════════════════════════

class TestMatchHistory:
    def test_append_and_display(self):
        h = MatchHistory()
        h.append(node_i=10, node_j=200, method="orb", n_inliers=55, rms=0.012)
        rows = h.to_display_rows()
        assert len(rows) == 1
        assert rows[0][1] == 10    # node_i
        assert rows[0][2] == 200   # node_j
        assert rows[0][3] == "orb"
        assert rows[0][4] == 55    # n_inliers
        assert rows[0][5] == "12.0"  # rms_mm
        assert rows[0][6] == "no"  # not yet accepted

    def test_mark_last_accepted(self):
        h = MatchHistory()
        h.append(1, 2, "xfeat", 100, 0.01)
        h.append(3, 4, "orb", 50, 0.02)
        h.mark_last_accepted()
        rows = h.to_display_rows()
        assert rows[0][6] == "no"
        assert rows[1][6] == "yes"

    def test_multiple_appends(self):
        h = MatchHistory()
        for k in range(5):
            h.append(k, k + 100, "sift", k * 10, None)
        assert len(h) == 5
        rows = h.to_display_rows()
        assert rows[4][1] == 4

    def test_rms_none_displayed_as_dash(self):
        h = MatchHistory()
        h.append(0, 1, "icp", 0, None)
        rows = h.to_display_rows()
        assert rows[0][5] == "—"

    def test_get(self):
        h = MatchHistory()
        h.append(5, 55, "akaze", 30, 0.009)
        row = h.get(0)
        assert row is not None
        assert row["node_i"] == 5
        assert row["node_j"] == 55
        assert row["method"] == "akaze"

    def test_get_out_of_range(self):
        h = MatchHistory()
        assert h.get(0) is None
        assert h.get(-1) is None

    def test_empty_len(self):
        h = MatchHistory()
        assert len(h) == 0


# ══════════════════════════════════════════════════════════════════════════════
# graph_view tests
# ══════════════════════════════════════════════════════════════════════════════

class TestBuildFigure:
    def _make_positions(self, n: int = 20):
        rng = np.random.default_rng(42)
        return [(i, rng.uniform(0, 10, 3)) for i in range(n)]

    def test_returns_figure_with_positions(self):
        from graph_view import build_figure
        pos = self._make_positions(30)
        fig = build_figure(pos)
        assert fig is not None
        # Should have at least the base scatter trace
        import plotly.graph_objects as go
        assert isinstance(fig, go.Figure)
        assert len(fig.data) >= 1

    def test_highlights_node_a_and_b(self):
        from graph_view import build_figure
        pos = self._make_positions(30)
        fig = build_figure(pos, node_a=5, node_b=12)
        # Should have traces named "Frame A" and "Frame B"
        trace_names = [t.name for t in fig.data]
        assert "Frame A" in trace_names
        assert "Frame B" in trace_names

    def test_anchor_edge_overlay(self):
        from graph_view import build_figure
        pos = self._make_positions(30)
        anchors = [{"node_i": 0, "node_j": 10}, {"node_i": 5, "node_j": 15}]
        fig = build_figure(pos, anchor_edges=anchors)
        trace_names = [t.name for t in fig.data]
        assert "accepted anchors" in trace_names

    def test_loaded_edge_overlay(self):
        from graph_view import build_figure
        pos = self._make_positions(30)
        loaded = [{"node_i": 2, "node_j": 8}, {"node_i": 1, "node_j": 20}]
        fig = build_figure(pos, loaded_edges=loaded)
        trace_names = [t.name for t in fig.data]
        assert "loaded edges" in trace_names

    def test_empty_positions_returns_none(self):
        from graph_view import build_figure
        fig = build_figure([])
        assert fig is None

    def test_no_node_a_b_still_builds(self):
        from graph_view import build_figure
        pos = self._make_positions(10)
        fig = build_figure(pos, node_a=None, node_b=None)
        assert fig is not None

    def test_node_not_in_positions_silently_skipped(self):
        from graph_view import build_figure
        pos = self._make_positions(10)
        # node_id 999 is not in pos
        fig = build_figure(pos, node_a=999)
        trace_names = [t.name for t in fig.data]
        assert "Frame A" not in trace_names  # no A marker for missing node


# ══════════════════════════════════════════════════════════════════════════════
# Smoke tests: callback logic (no Gradio server, no DB)
# ══════════════════════════════════════════════════════════════════════════════

class TestCallbackLogic:
    """Test the pure-logic parts of callbacks without a running Gradio server."""

    def setup_method(self):
        """Inject synthetic global state into app module."""
        import app as appmod
        self._appmod = appmod
        # Save originals
        self._orig_positions = appmod._positions
        self._orig_node_ids = appmod._node_ids
        self._orig_db_path = appmod._db_path
        self._orig_anchors = appmod._anchors
        self._orig_loaded_edges = appmod._loaded_edges
        self._orig_match_history = appmod._match_history

        # Inject synthetic state
        rng = np.random.default_rng(0)
        appmod._positions = [(i, rng.uniform(0, 5, 3)) for i in range(50)]
        appmod._node_ids = [i for i in range(50)]
        appmod._db_path = "synthetic"  # non-None so _require_db passes
        appmod._anchors = AnchorList()
        appmod._loaded_edges = LoadedEdges()
        appmod._match_history = MatchHistory()

    def teardown_method(self):
        appmod = self._appmod
        appmod._positions = self._orig_positions
        appmod._node_ids = self._orig_node_ids
        appmod._db_path = self._orig_db_path
        appmod._anchors = self._orig_anchors
        appmod._loaded_edges = self._orig_loaded_edges
        appmod._match_history = self._orig_match_history

    def test_node_id_at_index(self):
        from app import _node_id_at_index
        # With our synthetic state: _node_ids = [0, 1, ..., 49]
        assert _node_id_at_index(0) == 0
        assert _node_id_at_index(10) == 10
        assert _node_id_at_index(49) == 49
        assert _node_id_at_index(999) == 49  # clamped to last

    def test_index_of_node_id(self):
        from app import _index_of_node_id
        assert _index_of_node_id(0) == 0
        assert _index_of_node_id(10) == 10
        assert _index_of_node_id(999) == 0  # not found → 0

    def test_build_graph_returns_figure(self):
        from app import _build_graph
        fig = _build_graph(node_a=3, node_b=12)
        import plotly.graph_objects as go
        assert isinstance(fig, go.Figure)

    def test_history_append_on_match_stub(self):
        """Simulate the part of cb_match that appends to history."""
        import app as appmod
        h = appmod._match_history
        h.append(node_i=5, node_j=15, method="orb", n_inliers=42, rms=0.011)
        rows = h.to_display_rows()
        assert len(rows) == 1
        assert rows[0][1] == 5
        assert rows[0][2] == 15

    def test_load_edges_populates_table(self):
        """Simulate cb_load_edges with a real JSON file (if available)."""
        import app as appmod
        xfeat_path = "/home/mephisto/repos/NewOffice/pseudo-gt/edges/xfeat_edges.json"
        if not Path(xfeat_path).exists():
            pytest.skip("xfeat_edges.json not present")
        n, warnings = appmod._loaded_edges.load(xfeat_path)
        assert n > 0
        rows = appmod._loaded_edges.to_display_rows()
        assert len(rows) == n
        # All rows must have node_i and node_j populated
        for row in rows:
            assert isinstance(row[1], int)  # node_i
            assert isinstance(row[2], int)  # node_j


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
