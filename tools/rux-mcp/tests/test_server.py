# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tool surface, driven through a real MCP client against a fake ``rux``."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from mcp.client.client import Client

from rux_mcp.runner import RuxRunner
from rux_mcp.server import build_server

#: Every tool the gateway promises.  A rename here is a breaking change for
#: whatever agent playbook is pointed at the server, so it is pinned.
EXPECTED_TOOLS = {
    "analyze_accuracy",
    "analyze_quality",
    "get_component",
    "get_frame",
    "get_passport",
    "label_definitions",
    "list_clouds",
    "list_components",
    "list_frames",
    "list_meshes",
    "list_passports",
    "pipeline_log",
    "project_info",
    "query_db",
    "render_view",
    "validate_project",
}

EXPECTED_RESOURCES = {
    "rux://project/summary",
    "rux://project/validation",
    "rux://project/components",
    "rux://project/log",
    "rux://guide/query-paths",
}

INFO_JSON = json.dumps(
    {
        "schema_version": 11,
        "project_path": "/tmp/scan.rux",
        "sensor_frames": {"count": 238},
        "point_clouds": [
            {"name": "cloud", "type": "PointXYZRGB", "point_count": 255744},
            {
                "name": "labels",
                "type": "Label",
                "point_count": 255744,
                "labels": {"0": "ceiling", "1": "floor", "2": "wall"},
            },
        ],
        "meshes": [{"name": "mesh", "vertex_count": 440, "face_count": 840}],
        "material_passports": [{"guid": "abc", "property_count": 59}],
    }
)


@pytest.fixture
def server(runner: RuxRunner, tmp_path: Path):
    return build_server(runner, render_dir=tmp_path / "renders")


def text_of(result) -> str:
    return "\n".join(block.text for block in result.content if block.type == "text")


async def test_tool_and_resource_surface_is_stable(server):
    async with Client(server) as client:
        tools = {tool.name for tool in (await client.list_tools()).tools}
        resources = {
            str(resource.uri) for resource in (await client.list_resources()).resources
        }
    assert tools == EXPECTED_TOOLS
    assert resources == EXPECTED_RESOURCES


async def test_no_tool_can_mutate_the_project(server):
    """Read-only is a promise, so no tool may describe a mutating command."""
    forbidden = ("import", "create", "edit", "optimize", "register", "set", "del")
    async with Client(server) as client:
        tools = (await client.list_tools()).tools
    names = {tool.name for tool in tools}
    assert not any(name.startswith(verb + "_") for verb in forbidden for name in names)


async def test_project_info_returns_the_summary(server, fake_rux):
    fake_rux.reply(INFO_JSON)
    async with Client(server) as client:
        result = await client.call_tool("project_info", {})
    assert not result.is_error
    assert json.loads(text_of(result))["schema_version"] == 11


async def test_list_clouds_wraps_the_list(server, fake_rux):
    fake_rux.reply(INFO_JSON)
    async with Client(server) as client:
        result = await client.call_tool("list_clouds", {})
    payload = json.loads(text_of(result))
    assert payload["count"] == 2
    assert [entry["name"] for entry in payload["clouds"]] == ["cloud", "labels"]


async def test_label_definitions_reads_the_named_cloud(server, fake_rux):
    fake_rux.reply(INFO_JSON)
    async with Client(server) as client:
        result = await client.call_tool("label_definitions", {"cloud": "labels"})
    assert json.loads(text_of(result))["2"] == "wall"


async def test_label_definitions_names_the_mistake(server, fake_rux):
    fake_rux.reply(INFO_JSON)
    async with Client(server) as client:
        result = await client.call_tool("label_definitions", {"cloud": "nope"})
    assert result.is_error
    assert "list_clouds()" in text_of(result)


async def test_list_frames_paginates(server, fake_rux):
    fake_rux.reply(json.dumps(list(range(1, 101))))
    async with Client(server) as client:
        result = await client.call_tool("list_frames", {"limit": 3, "offset": 10})
    payload = json.loads(text_of(result))
    assert payload == {
        "total": 100,
        "offset": 10,
        "limit": 3,
        "node_ids": [11, 12, 13],
    }


async def test_query_db_forwards_an_allowed_path(server, fake_rux):
    fake_rux.reply('{"name": "cloud", "point_count": 255744}')
    async with Client(server) as client:
        result = await client.call_tool("query_db", {"path": "clouds.cloud.metadata"})
    payload = json.loads(text_of(result))
    assert payload["path"] == "clouds.cloud.metadata"
    assert payload["result"]["point_count"] == 255744
    assert fake_rux.calls[-1].endswith("get clouds.cloud.metadata")


async def test_query_db_refuses_binary_without_calling_rux(server, fake_rux):
    fake_rux.reply("should never be read")
    async with Client(server) as client:
        result = await client.call_tool("query_db", {"path": "clouds.cloud"})
    assert result.is_error
    assert "render_view" in text_of(result)
    assert fake_rux.calls == []


async def test_rux_failure_reaches_the_model_with_its_stderr(server, fake_rux):
    fake_rux.reply("", stderr="Path error: Invalid collection name: x", status=1)
    async with Client(server) as client:
        result = await client.call_tool("project_info", {})
    assert result.is_error
    message = text_of(result)
    assert "Invalid collection name" in message
    assert "exit status: 1" in message


async def test_missing_project_is_reported_by_every_tool(tmp_path, fake_rux):
    runner = RuxRunner(
        project=tmp_path / "gone.rux", binary=str(fake_rux.path), timeout=5
    )
    server = build_server(runner, render_dir=tmp_path / "renders")
    fake_rux.reply(INFO_JSON)
    async with Client(server) as client:
        for name in ("project_info", "list_clouds", "pipeline_log", "list_components"):
            result = await client.call_tool(name, {})
            assert result.is_error, name
            assert "gone.rux" in text_of(result), name


async def test_timeout_is_reported_as_such(project, tmp_path):
    slow = tmp_path / "slow-rux"
    slow.write_text("#!/bin/sh\nsleep 5\n")
    slow.chmod(0o755)
    runner = RuxRunner(project=project, binary=str(slow), timeout=0.3)
    server = build_server(runner, render_dir=tmp_path / "renders")
    async with Client(server) as client:
        result = await client.call_tool("project_info", {})
    assert result.is_error
    assert "timed out" in text_of(result)


async def test_validate_reports_a_failing_project_instead_of_crashing(server, fake_rux):
    fake_rux.reply(
        '{"ok": false, "error_count": 1, "issues": []}',
        stderr="1 error found",
        status=1,
    )
    async with Client(server) as client:
        result = await client.call_tool("validate_project", {})
    assert not result.is_error
    assert json.loads(text_of(result))["error_count"] == 1


async def test_validate_stage_name_is_checked(server, fake_rux):
    fake_rux.reply("{}")
    async with Client(server) as client:
        result = await client.call_tool("validate_project", {"stage": "mesh; rm -rf /"})
    assert result.is_error
    assert fake_rux.calls == []


@pytest.mark.parametrize(
    "arguments,needle",
    [
        ({"view": "sideways"}, "unknown view"),
        ({"size": "huge"}, "1024x768"),
        ({"layers": "everything"}, "unknown render layer"),
        ({"layers": ""}, "at least one layer"),
    ],
)
async def test_render_view_rejects_bad_arguments(server, fake_rux, arguments, needle):
    fake_rux.reply("")
    async with Client(server) as client:
        result = await client.call_tool("render_view", arguments)
    assert result.is_error
    assert needle in text_of(result)
    assert fake_rux.calls == []


async def test_render_view_returns_the_png_it_produced(
    runner, fake_rux, tmp_path, monkeypatch
):
    # The fake rux cannot render, so have it create the file the CLI would.
    script = tmp_path / "bin" / "rux"
    script.write_text(
        "#!/bin/sh\n"
        'printf "%s\\n" "$*" >> "$(dirname "$0")/calls.txt"\n'
        'while [ "$1" != "-o" ]; do shift; done\n'
        'printf "\\211PNG\\r\\n" > "$2"\n'
    )
    script.chmod(0o755)
    server = build_server(runner, render_dir=tmp_path / "renders")
    async with Client(server) as client:
        result = await client.call_tool(
            "render_view", {"view": "top", "layers": "cloud,planes", "size": "640x480"}
        )
    assert not result.is_error
    summary = json.loads(text_of(result))
    assert summary["layers"] == ["cloud", "planes"]
    assert summary["image_count"] == 1
    assert Path(summary["paths"][0]).is_file()
    images = [block for block in result.content if block.type == "image"]
    assert len(images) == 1
    assert images[0].mime_type == "image/png"


async def test_render_view_can_withhold_the_image_bytes(runner, fake_rux, tmp_path):
    script = tmp_path / "bin" / "rux"
    script.write_text(
        "#!/bin/sh\n"
        'while [ "$1" != "-o" ]; do shift; done\n'
        'printf "\\211PNG\\r\\n" > "$2"\n'
    )
    script.chmod(0o755)
    server = build_server(runner, render_dir=tmp_path / "renders")
    async with Client(server) as client:
        result = await client.call_tool("render_view", {"inline": False})
    assert not result.is_error
    assert not [block for block in result.content if block.type == "image"]


async def test_components_come_from_the_element_csv(runner, fake_rux, tmp_path):
    script = tmp_path / "bin" / "rux"
    script.write_text(
        "#!/bin/sh\n"
        'while [ "$1" != "-o" ]; do shift; done\n'
        'cat > "$2" <<EOF\n'
        "kind,id,component_name,component_type,confidence,notes,"
        "Owner / contact email\n"
        "component,7b46,window_1,window,0.9,,\n"
        "passport,bc65,,,,,someone@example.com\n"
        "EOF\n"
    )
    script.chmod(0o755)
    server = build_server(runner, render_dir=tmp_path / "renders")
    async with Client(server) as client:
        listed = await client.call_tool("list_components", {})
        found = await client.call_tool("get_component", {"component_id": "window_1"})
        missing = await client.call_tool("get_component", {"component_id": "door_9"})

    payload = json.loads(text_of(listed))
    assert payload["count"] == 1
    component = payload["components"][0]
    assert component["component_type"] == "window"
    # Passport template columns and blank fields are dropped.
    assert "Owner / contact email" not in component
    assert "notes" not in component

    assert json.loads(text_of(found))["id"] == "7b46"
    assert missing.is_error
    assert "window_1" in text_of(missing)


async def test_analyze_accuracy_checks_the_ground_truth_exists(server, fake_rux):
    fake_rux.reply("{}")
    async with Client(server) as client:
        result = await client.call_tool(
            "analyze_accuracy", {"ground_truth_path": "/no/such/gt.ply"}
        )
    assert result.is_error
    assert "not found" in text_of(result)
    assert fake_rux.calls == []


async def test_query_path_guide_is_readable_as_a_resource(server):
    async with Client(server) as client:
        result = await client.read_resource("rux://guide/query-paths")
    body = result.contents[0].text
    assert "denied (binary)" in body
    assert "clouds.<item>.<property>" in body
    assert "list_components()" in body
