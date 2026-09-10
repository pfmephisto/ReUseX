# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""One end-to-end pass against a real ``rux`` and a real project.

Skipped unless both are available.  Point it at a specific pair with::

    RUX_BIN=build/apps/rux/rux RUX_PROJECT=scan.rux pytest -m integration

Everything here is read-only — ``info``, ``get``, ``log``, ``validate``,
``export csv`` and ``render`` all open the database read-only.
"""

from __future__ import annotations

import json
import os
import shutil
from pathlib import Path

import pytest
from mcp.client.client import Client

from rux_mcp.runner import RuxRunner
from rux_mcp.server import build_server

pytestmark = pytest.mark.integration


def find_real_rux() -> str | None:
    """Locate a usable ``rux``, or return ``None`` so the test skips."""
    candidate = os.environ.get("RUX_BIN")
    if candidate and Path(candidate).is_file() and os.access(candidate, os.X_OK):
        return candidate
    return shutil.which("rux")


def find_real_project() -> str | None:
    """Locate a ``.rux`` project, or return ``None`` so the test skips."""
    candidate = os.environ.get("RUX_PROJECT")
    if candidate and Path(candidate).is_file():
        return candidate
    for guess in (Path.cwd() / "project.rux", *Path.cwd().parents):
        path = guess if guess.suffix == ".rux" else guess / "project.rux"
        if path.is_file():
            return str(path)
    return None


RUX = find_real_rux()
PROJECT = find_real_project()

requires_rux = pytest.mark.skipif(
    RUX is None or PROJECT is None,
    reason="needs a built rux (RUX_BIN/PATH) and a .rux project (RUX_PROJECT)",
)


@pytest.fixture
def live_server(tmp_path: Path):
    runner = RuxRunner(project=Path(PROJECT), binary=RUX, timeout=180.0)
    return build_server(runner, render_dir=tmp_path / "renders")


def text_of(result) -> str:
    return "\n".join(block.text for block in result.content if block.type == "text")


@requires_rux
async def test_summary_query_and_render_against_a_real_project(live_server):
    async with Client(live_server) as client:
        info = await client.call_tool("project_info", {})
        assert not info.is_error, text_of(info)
        summary = json.loads(text_of(info))
        assert "schema_version" in summary

        report = await client.call_tool("validate_project", {})
        assert not report.is_error, text_of(report)
        assert "ok" in json.loads(text_of(report))

        clouds = json.loads(text_of(await client.call_tool("list_clouds", {})))

    # A freshly-created project has nothing to query or draw; the summary path
    # above is all such a project can prove.  Skipping has to happen outside
    # the client's task group, which would otherwise swallow the reason.
    if clouds["count"] == 0:
        pytest.skip(f"{PROJECT} holds no point clouds")
    first = clouds["clouds"][0]["name"]

    async with Client(live_server) as client:
        metadata = await client.call_tool(
            "query_db", {"path": f"clouds.{first}.metadata"}
        )
        assert not metadata.is_error, text_of(metadata)
        assert json.loads(text_of(metadata))["result"]["name"] == first

        # The binary sibling of that same path must stay refused.
        refused = await client.call_tool("query_db", {"path": f"clouds.{first}"})
        assert refused.is_error

        rendered = await client.call_tool(
            "render_view",
            {"view": "top", "layers": "cloud", "cloud": first, "size": "640x480"},
        )
        assert not rendered.is_error, text_of(rendered)
        produced = json.loads(text_of(rendered))["paths"]
        assert produced and Path(produced[0]).stat().st_size > 0
        images = [block for block in rendered.content if block.type == "image"]
        assert images and images[0].mime_type == "image/png"
