# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""The MCP tool and resource surface of the rux gateway.

Read-only *by default*: at :attr:`WriteMode.none` no tool here runs
``rux import``, ``rux create``, ``rux edit``, ``rux optimize``,
``rux register``, ``rux set`` or ``rux del`` — and the tools that could are
not registered at all, so the model never sees them advertised. Raising the
write mode is an operator decision made on the command line, never something
an agent can ask for.

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

from rux_mcp import commands as command_catalogue
from rux_mcp import components as component_inventory
from rux_mcp.commands import CommandNotAllowed, WriteMode
from rux_mcp.jobs import JobBusy, JobNotFound, JobRunner
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
        except (JobNotFound, KeyError) as exc:
            # KeyError's str() wraps the message in quotes; use it verbatim.
            message = (
                exc.args[0]
                if exc.args and isinstance(exc.args[0], str)
                else str(exc)
            )
            raise ToolError(message) from exc
        except (
            RuxError,
            PathNotAllowed,
            CommandNotAllowed,
            JobBusy,
            ValueError,
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


def _instructions(project: Path, write_mode: WriteMode) -> str:
    """The server's own briefing, which depends on what it will allow."""
    shared = (
        f"Access to one ReUseX project database ({project}).\n"
        "Start with the rux://project/summary resource, then use query_db() "
        "for specific records and render_view() to actually look at the "
        "geometry — the gateway never puts raw point data in your context, so "
        "renders are how you see the scan.\n"
    )
    if write_mode is WriteMode.none:
        return shared + (
            "This gateway is READ-ONLY. Nothing here modifies the project; "
            "run pipeline stages yourself with the rux CLI, or ask the "
            "operator to restart the gateway with --write-mode stages."
        )
    return shared + (
        f"This gateway can also RUN commands (write mode: {write_mode.value}). "
        "list_commands() says which, command_help() reads the real flag list "
        "out of the binary, and run_command() submits one. Commands run as "
        "background jobs — one at a time, because the project database is "
        "sqlite — so poll job_status() for anything slower than a few "
        "seconds.\n"
        "Before running a stage, validate_project(stage=...) tells you "
        "whether its inputs are in place; afterwards, analyze_quality() and "
        "render_view() tell you whether it helped."
    )


def build_server(
    runner: RuxRunner,
    *,
    render_dir: Path | None = None,
    job_log_dir: Path | None = None,
    write_mode: WriteMode = WriteMode.none,
    name: str = "rux",
) -> MCPServer:
    """Assemble the MCP server for one ``.rux`` project.

    Args:
        runner: bound to the project this gateway serves.
        render_dir: where ``render_view`` writes PNGs.  A private temp
            directory is created when omitted.
        job_log_dir: where the full output of each background job is written.
            Defaults to a ``jobs`` directory beside the renders.
        write_mode: how much of the `rux` command catalogue to expose.
            :attr:`WriteMode.none` — the default — registers no mutating tool
            at all, so an agent cannot even see them.
        name: MCP server name reported during initialization.
    """
    write_mode = WriteMode(write_mode)
    renders = Path(
        render_dir
        if render_dir is not None
        else tempfile.mkdtemp(prefix="rux-mcp-renders-")
    )
    renders.mkdir(parents=True, exist_ok=True)
    jobs = JobRunner(
        runner,
        log_dir=Path(job_log_dir) if job_log_dir else renders.parent / "rux-mcp-jobs",
    )

    server = MCPServer(
        name=name,
        version="0.2.0",
        instructions=_instructions(runner.project, write_mode),
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

    # ---------------------------------------------------- command catalogue

    @server.tool(
        description=(
            "Which `rux` commands this gateway will run, and which are "
            "blocked by its write mode. Read this before run_command()."
        )
    )
    @_friendly
    def list_commands() -> dict:
        entries = [
            {
                "command": command.key,
                "mode": command.mode.value,
                "runnable": write_mode.allows(command.mode),
                "mutates_project": command.mutates_project,
                "summary": command.summary,
                "positional": command.positional_hint or None,
                "default_timeout_seconds": command.default_timeout,
                "needs_confirmation_option": command_catalogue.NEEDS_CONFIRMATION.get(
                    command.key
                ),
            }
            for command in command_catalogue.CATALOGUE.values()
        ]
        runnable = [entry for entry in entries if entry["runnable"]]
        return {
            "write_mode": write_mode.value,
            "runnable_count": len(runnable),
            "commands": entries,
            "note": (
                "Read-only gateway: no command that changes the project is "
                "exposed. Ask the operator to restart with "
                "--write-mode stages|full."
                if write_mode is WriteMode.none
                else "run_command() submits a background job; poll "
                "job_status() unless it finished inside wait_seconds."
            ),
        }

    @server.tool(
        description=(
            "The `--help` text of one catalogued command, straight from the "
            "binary. Use it to get the real flag names before run_command() "
            "— the gateway does not carry its own copy of the flag list."
        )
    )
    @_friendly
    def command_help(command: str) -> dict:
        # Help is harmless for anything in the catalogue, so it is readable
        # regardless of write mode; the response says whether it is runnable.
        entry = command_catalogue.resolve(command, WriteMode.full)
        text = runner.run([*entry.argv, "--help"], check_project=False)
        return {
            "command": entry.key,
            "runnable": write_mode.allows(entry.mode),
            "required_write_mode": entry.mode.value,
            "help": strip_log_lines(text),
        }

    if write_mode is not WriteMode.none:

        @server.tool(
            description=(
                "Run one catalogued `rux` command against the project. "
                "'options' are long flags without dashes — {'grid-size': "
                "0.05, 'cuda': true} becomes --grid-size=0.05 --cuda. "
                "'arguments' are positionals (an input path, a `set` path and "
                "value). Returns a job: it waits up to wait_seconds for the "
                "command to finish, and otherwise hands back a job_id to poll "
                "with job_status(). One command runs at a time."
            )
        )
        @_friendly
        def run_command(
            command: str,
            arguments: list[str] | None = None,
            options: dict[str, Any] | None = None,
            wait_seconds: float = 10.0,
            timeout_seconds: float | None = None,
        ) -> dict:
            invocation = command_catalogue.build(
                command,
                write_mode,
                arguments=arguments,
                options=options,
                timeout=timeout_seconds,
            )
            key = invocation.command.key
            confirm = command_catalogue.NEEDS_CONFIRMATION.get(key)
            if confirm and not (options or {}).get(confirm):
                raise CommandNotAllowed(
                    f"'{key}' destroys data and asks for confirmation on a "
                    "terminal, which a gateway does not have. Re-run it with "
                    f"options={{'{confirm}': true}} if that is really what you "
                    "want."
                )
            if wait_seconds < 0 or wait_seconds > 300:
                raise ValueError("wait_seconds must be between 0 and 300")

            job = jobs.submit(key, invocation.args, invocation.timeout)
            if wait_seconds > 0:
                job = jobs.wait(job.id, wait_seconds)

            result = job.summary()
            result["output_tail"] = job.output(3000)
            if not result["done"]:
                result["hint"] = (
                    f"still running after {wait_seconds:g}s — poll "
                    f"job_status(job_id='{job.id}'), or read job_output() for "
                    "progress. Its full log is at " + str(job.log_path)
                )
            elif result["state"] == "succeeded" and invocation.command.mutates_project:
                result["next"] = (
                    "the project changed: re-read rux://project/summary, and "
                    "check the result with validate_project(), "
                    "analyze_quality() or render_view()."
                )
            return result

        @server.tool(
            description=(
                "State of one background job. Set wait_seconds to block for "
                "it rather than polling in a tight loop."
            )
        )
        @_friendly
        def job_status(job_id: str, wait_seconds: float = 0.0) -> dict:
            if wait_seconds < 0 or wait_seconds > 300:
                raise ValueError("wait_seconds must be between 0 and 300")
            job = jobs.wait(job_id, wait_seconds) if wait_seconds else jobs.get(job_id)
            result = job.summary()
            result["output_tail"] = job.output(1500)
            return result

        @server.tool(
            description=(
                "The captured output of a job — `rux`'s own log lines, which "
                "say what a stage actually did. Tail by default; the full log "
                "is on disk at the job's log_path."
            )
        )
        @_friendly
        def job_output(job_id: str, tail_chars: int = 6000) -> dict:
            job = jobs.get(job_id)
            if tail_chars < 0:
                raise ValueError("tail_chars must be >= 0")
            return {
                "job_id": job.id,
                "command": job.label,
                "state": job.state,
                "log_path": str(job.log_path),
                "output": job.output(tail_chars),
            }

        @server.tool(description="Jobs this gateway has run, newest first.")
        @_friendly
        def list_jobs(limit: int = 20) -> dict:
            if limit < 1 or limit > 200:
                raise ValueError("limit must be between 1 and 200")
            entries = [job.summary() for job in jobs.list(limit)]
            active = jobs.active_job
            return {
                "count": len(entries),
                "running": active.id if active else None,
                "jobs": entries,
            }

        @server.tool(
            description=(
                "Terminate a running job. A stage killed part-way may leave "
                "the project without the output it was producing — check "
                "validate_project() afterwards."
            )
        )
        @_friendly
        def cancel_job(job_id: str) -> dict:
            return jobs.cancel(job_id).summary()

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
