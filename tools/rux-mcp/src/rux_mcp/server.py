# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""The MCP tool and resource surface of the rux gateway.

Read-only by construction: no tool here runs ``rux import``, ``rux create``,
``rux edit``, ``rux optimize``, ``rux register``, ``rux set`` or ``rux del``.
The gateway can look at a project and describe it; changing one stays a
deliberate act at the command line.

Design follows the progressive-disclosure rule from #267: cheap summaries are
resources, everything else is a tool, geometry is only ever seen through
``render_view`` and raw point data never enters the context.
"""

from __future__ import annotations

import base64
import functools
import json
import re
import tempfile
import uuid
from pathlib import Path
from typing import Any, Callable, TypeVar

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ResourceError, ToolError
from mcp.types import ImageContent, TextContent

from rux_mcp import components as component_inventory
from rux_mcp.paths import PathNotAllowed, check_query_path, describe_allowlist
from rux_mcp.runner import RuxError, RuxRunner, strip_log_lines

#: Layers ``rux render`` knows about (``apps/rux/src/render.cpp``).
RENDER_LAYERS = (
    "cloud",
    "labels",
    "planes",
    "rooms",
    "instances",
    "mesh",
    "components",
)

_VIEW = re.compile(r"^(top|front|orbit(:\d+)?|frame:\d+)$")
_SIZE = re.compile(r"^(\d{2,5})x(\d{2,5})$")

#: Rendering a large cloud is slower than a metadata query; give it its own
#: budget so a modest default ``--timeout`` does not make the agent blind.
RENDER_TIMEOUT_FACTOR = 4.0

_F = TypeVar("_F", bound=Callable[..., Any])


def _friendly(fn: _F) -> _F:
    """Convert the gateway's own exceptions into agent-readable tool errors.

    The MCP SDK deliberately withholds the text of *unexpected* exceptions from
    the model — a tool that raises ``RuntimeError`` yields only "Error executing
    tool X".  Anything the agent could act on has to be raised as ``ToolError``.
    """

    @functools.wraps(fn)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        try:
            return fn(*args, **kwargs)
        except (
            RuxError,
            PathNotAllowed,
            ValueError,
            KeyError,
            FileNotFoundError,
        ) as exc:
            raise ToolError(str(exc)) from exc

    return wrapper  # type: ignore[return-value]


def _validate_size(size: str) -> str:
    match = _SIZE.fullmatch(size.strip().lower())
    if not match:
        raise ValueError(f"size must look like '1024x768', got {size!r}")
    width, height = int(match.group(1)), int(match.group(2))
    if width * height > 16_000_000:
        raise ValueError("size is too large; keep width*height under 16 megapixels")
    return f"{width}x{height}"


def _validate_view(view: str) -> str:
    cleaned = view.strip()
    if not _VIEW.fullmatch(cleaned):
        raise ValueError(
            f"unknown view {view!r}; use 'top', 'front', 'orbit' or 'orbit:N', "
            "or 'frame:<node_id>'"
        )
    return cleaned


def _validate_layers(layers: str) -> str:
    wanted = [part.strip() for part in layers.split(",") if part.strip()]
    if not wanted:
        raise ValueError("layers must name at least one layer")
    unknown = [name for name in wanted if name not in RENDER_LAYERS]
    if unknown:
        raise ValueError(
            f"unknown render layer(s): {', '.join(unknown)}. Available: "
            + ", ".join(RENDER_LAYERS)
        )
    return ",".join(wanted)


def _clouds_from_info(info: dict) -> list[dict]:
    return list(info.get("point_clouds") or [])


def build_server(
    runner: RuxRunner,
    *,
    render_dir: Path | None = None,
    name: str = "rux",
) -> MCPServer:
    """Assemble the MCP server for one ``.rux`` project.

    Args:
        runner: bound to the project this gateway serves.
        render_dir: where ``render_view`` writes PNGs.  A private temp
            directory is created when omitted.
        name: MCP server name reported during initialization.
    """
    renders = Path(
        render_dir
        if render_dir is not None
        else tempfile.mkdtemp(prefix="rux-mcp-renders-")
    )
    renders.mkdir(parents=True, exist_ok=True)

    server = MCPServer(
        name=name,
        version="0.1.0",
        instructions=(
            "Read-only access to one ReUseX project database "
            f"({runner.project}).\n"
            "Start with the rux://project/summary resource, then use "
            "query_db() for specific records and render_view() to actually "
            "look at the geometry — the gateway never puts raw point data in "
            "your context, so renders are how you see the scan.\n"
            "Nothing here modifies the project; run pipeline stages yourself "
            "with the rux CLI."
        ),
    )

    render_timeout = runner.timeout * RENDER_TIMEOUT_FACTOR

    # ---------------------------------------------------------------- tools

    @server.tool(
        description=(
            "Project summary: schema version, sensor-frame counts, point "
            "clouds, meshes, material passports. The cheapest orientation "
            "call — start here."
        )
    )
    @_friendly
    def project_info() -> dict:
        return runner.run_json(["info", "--json"])

    @server.tool(
        description=(
            "Point clouds stored in the project, with type, point count and "
            "label definitions where present."
        )
    )
    @_friendly
    def list_clouds() -> dict:
        clouds = _clouds_from_info(runner.run_json(["info", "--json"]))
        return {"count": len(clouds), "clouds": clouds}

    @server.tool(description="Meshes stored in the project, with vertex/face counts.")
    @_friendly
    def list_meshes() -> dict:
        meshes = list(runner.run_json(["info", "--json"]).get("meshes") or [])
        return {"count": len(meshes), "meshes": meshes}

    @server.tool(
        description=(
            "Label definitions of a label cloud (id -> class name), e.g. the "
            "semantic classes behind the 'labels' cloud or the plane ids "
            "behind 'planes'."
        )
    )
    @_friendly
    def label_definitions(cloud: str = "labels") -> dict:
        for entry in _clouds_from_info(runner.run_json(["info", "--json"])):
            if entry.get("name") == cloud:
                return dict(entry.get("labels") or {})
        raise ValueError(
            f"no cloud named {cloud!r}; call list_clouds() for what exists"
        )

    @server.tool(
        description=(
            "Sensor frame node ids, paginated. Frame images are deliberately "
            "not exposed; use render_view(view='frame:<id>') to see one."
        )
    )
    @_friendly
    def list_frames(limit: int = 50, offset: int = 0) -> dict:
        if limit < 1 or limit > 1000:
            raise ValueError("limit must be between 1 and 1000")
        if offset < 0:
            raise ValueError("offset must be >= 0")
        ids = runner.run_json(["get", "frames"])
        if not isinstance(ids, list):
            ids = []
        return {
            "total": len(ids),
            "offset": offset,
            "limit": limit,
            "node_ids": ids[offset : offset + limit],
        }

    @server.tool(
        description=(
            "Metadata for one sensor frame: image dimensions, intrinsics, the "
            "4x4 world pose (row-major) and which channels exist."
        )
    )
    @_friendly
    def get_frame(node_id: int) -> dict:
        return runner.run_json(["get", f"frames.{int(node_id)}"])

    @server.tool(
        description=(
            "Building-component inventory (windows, doors, walls...) with "
            "type, confidence and type-specific properties."
        )
    )
    @_friendly
    def list_components() -> dict:
        items = component_inventory.list_components(runner)
        return {"count": len(items), "components": items}

    @server.tool(
        description="One building component, addressed by GUID or by component_name."
    )
    @_friendly
    def get_component(component_id: str) -> dict:
        return component_inventory.get_component(runner, component_id)

    @server.tool(description="GUIDs of the material passports stored in the project.")
    @_friendly
    def list_passports() -> dict:
        items = list(
            runner.run_json(["info", "--json"]).get("material_passports") or []
        )
        return {"count": len(items), "passports": items}

    @server.tool(
        description=(
            "One material passport by document GUID, including its stored "
            "property values."
        )
    )
    @_friendly
    def get_passport(guid: str) -> dict:
        check_query_path(f"materials.{guid}")
        return runner.run_json(["get", f"materials.{guid}"])

    @server.tool(
        description=(
            "Path-addressed query over the project database — the JSON-only "
            "subset of `rux get`. An empty path lists the collections "
            "(clouds, frames, labels, log, materials, meshes, panoramas, "
            "projects). Paths that would return raw point clouds, images or "
            "mesh binaries are refused; read the rux://guide/query-paths "
            "resource for the full allowlist."
        )
    )
    @_friendly
    def query_db(path: str = "") -> dict:
        parts = check_query_path(path)
        joined = ".".join(parts)
        result = runner.run_json(["get", joined] if parts else ["get"])
        # Wrapped in a dict on purpose: a bare list return is exploded by the
        # SDK into one content block per element, which reads terribly for a
        # 16-element pose matrix.
        return {"path": joined, "result": result}

    @server.tool(
        description=(
            "Render the project off-screen and return the image(s). This is "
            "how you look at the scan. view: 'top' (orthographic floor plan), "
            "'front' (elevation), 'orbit[:N]' (N perspective views on a ring) "
            "or 'frame:<node_id>' (reproduce a captured viewpoint). layers is "
            "a comma-separated subset of "
            + ", ".join(RENDER_LAYERS)
            + ", drawn back to front."
        )
    )
    @_friendly
    def render_view(
        view: str = "top",
        layers: str = "cloud",
        size: str = "1024x768",
        cloud: str | None = None,
        mesh: str | None = None,
        point_size: float | None = None,
        elevation: float | None = None,
        inline: bool = True,
        max_inline_images: int = 4,
    ) -> list[TextContent | ImageContent]:
        view = _validate_view(view)
        layers = _validate_layers(layers)
        size = _validate_size(size)
        if max_inline_images < 0:
            raise ValueError("max_inline_images must be >= 0")

        target_dir = renders / uuid.uuid4().hex[:12]
        target_dir.mkdir(parents=True, exist_ok=True)
        output = target_dir / "view.png"

        args = [
            "render",
            "-o",
            str(output),
            "--view",
            view,
            "--layers",
            layers,
            "--size",
            size,
        ]
        if cloud:
            args += ["--cloud", cloud]
        if mesh:
            args += ["--mesh", mesh]
        if point_size is not None:
            args += ["--point-size", repr(float(point_size))]
        if elevation is not None:
            args += ["--elevation", repr(float(elevation))]

        runner.run(args, timeout=render_timeout)

        # 'orbit:N' writes numbered files next to the requested name, so take
        # whatever landed in this call's private directory.
        produced = sorted(target_dir.glob("*.png"))
        if not produced:
            raise RuxError(
                "rux render reported success but wrote no PNG; check the "
                "requested layers exist in this project"
            )

        summary = {
            "view": view,
            "layers": layers.split(","),
            "size": size,
            "image_count": len(produced),
            "paths": [str(path) for path in produced],
        }
        content: list[TextContent | ImageContent] = [
            TextContent(type="text", text=json.dumps(summary, indent=2))
        ]
        if inline:
            for path in produced[:max_inline_images]:
                content.append(
                    ImageContent(
                        type="image",
                        data=base64.b64encode(path.read_bytes()).decode("ascii"),
                        mime_type="image/png",
                    )
                )
        return content

    @server.tool(
        description=(
            "Ground-truth-free reconstruction quality: point-to-plane "
            "flatness RMS and 90th-percentile surface thickness, in metres. "
            "Lower is better; use it as the objective signal in a "
            "modify-and-check loop."
        )
    )
    @_friendly
    def analyze_quality(
        cloud: str = "cloud", planes: str = "planes", min_points: int = 100
    ) -> dict:
        if min_points < 3:
            raise ValueError("min_points must be >= 3")
        return runner.run_json(
            [
                "analyze",
                "quality",
                "--cloud",
                cloud,
                "--planes",
                planes,
                "--min-points",
                str(int(min_points)),
            ]
        )

    @server.tool(
        description=(
            "Score the reconstruction against an external ground-truth PLY: "
            "accuracy, completeness, chamfer, precision/recall/F-score. Both "
            "clouds must share a coordinate frame."
        )
    )
    @_friendly
    def analyze_accuracy(
        ground_truth_path: str,
        cloud: str = "cloud",
        threshold: float = 0.05,
        gt_voxel: float = 0.01,
    ) -> dict:
        gt = Path(ground_truth_path).expanduser()
        if not gt.is_file():
            raise FileNotFoundError(f"ground-truth cloud not found: {gt}")
        if threshold <= 0:
            raise ValueError("threshold must be positive")
        return runner.run_json(
            [
                "analyze",
                "accuracy",
                "--gt",
                str(gt),
                "--cloud",
                cloud,
                "--threshold",
                repr(float(threshold)),
                "--gt-voxel",
                repr(float(gt_voxel)),
            ]
        )

    @server.tool(
        description=(
            "Pipeline execution history: which stages ran, with what "
            "parameters, when, and whether they succeeded. Newest first."
        )
    )
    @_friendly
    def pipeline_log(limit: int = 20) -> dict:
        if limit < 0:
            raise ValueError("limit must be >= 0 (0 means all entries)")
        entries = runner.run_json(["log", "--json", "-n", str(int(limit))])
        if not isinstance(entries, list):
            entries = [entries]
        return {"count": len(entries), "entries": entries}

    @server.tool(
        description=(
            "Referential-integrity report for the whole project, or — with "
            "stage set — the input contract of one pipeline stage "
            "(import, optimize, register, clouds, annotate, project, planes, "
            "rooms, instances, mesh, texture, windows, gsplat)."
        )
    )
    @_friendly
    def validate_project(stage: str | None = None) -> dict:
        args = ["validate", "--json"]
        if stage:
            if not re.fullmatch(r"[a-z_]+", stage):
                raise ValueError(f"invalid stage name {stage!r}")
            args += ["--stage", stage]
        try:
            return runner.run_json(args)
        except RuxError as exc:
            # `rux validate` exits non-zero when it finds errors but still
            # prints the report; a failing project is an answer, not a crash.
            payload = strip_log_lines(exc.stdout)
            if payload:
                try:
                    return json.loads(payload)
                except json.JSONDecodeError:
                    pass
            raise

    # ------------------------------------------------------------ resources

    def _resource_json(args: list[str]) -> str:
        try:
            return json.dumps(runner.run_json(args), indent=2)
        except RuxError as exc:
            raise ResourceError(str(exc)) from exc

    @server.resource(
        "rux://project/summary",
        name="Project summary",
        description="Schema version, frame counts, clouds, meshes, passports.",
        mime_type="application/json",
    )
    def resource_summary() -> str:
        return _resource_json(["info", "--json"])

    @server.resource(
        "rux://project/validation",
        name="Project validation report",
        description="Referential-integrity issues found in the project database.",
        mime_type="application/json",
    )
    def resource_validation() -> str:
        try:
            return json.dumps(validate_project(), indent=2)
        except ToolError as exc:
            raise ResourceError(str(exc)) from exc

    @server.resource(
        "rux://project/components",
        name="Building component inventory",
        description="Every building component with its type and properties.",
        mime_type="application/json",
    )
    def resource_components() -> str:
        try:
            return json.dumps(component_inventory.list_components(runner), indent=2)
        except RuxError as exc:
            raise ResourceError(str(exc)) from exc

    @server.resource(
        "rux://project/log",
        name="Pipeline log",
        description="Full pipeline execution history for the project.",
        mime_type="application/json",
    )
    def resource_log() -> str:
        return _resource_json(["log", "--json"])

    @server.resource(
        "rux://guide/query-paths",
        name="query_db path allowlist",
        description=(
            "Which `rux get` paths the gateway exposes, and why the rest " "are not."
        ),
        mime_type="text/plain",
    )
    def resource_query_paths() -> str:
        return describe_allowlist()

    return server
