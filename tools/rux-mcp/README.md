# rux-mcp — a read-only MCP gateway over the `rux` CLI

`rux-mcp` exposes one ReUseX project (`*.rux`) to an LLM agent over the
[Model Context Protocol](https://modelcontextprotocol.io). It is a thin Python
adapter: every tool shells out to the `rux` command line and returns what `rux`
already knows how to print. No pipeline logic lives here.

This is **Phase 2 of [#267](https://github.com/pfmephisto/ReUseX/issues/267)**
(agent-driven simplified modeling). Phase 1 — `rux render`, the agent's "eyes" —
already shipped, so the gateway can hand an agent a picture of the scan.
The Blender / blender-mcp half of #267 is a later phase and is not part of this
package.

## Read-only, on purpose

Nothing here runs `rux import`, `rux create`, `rux edit`, `rux optimize`,
`rux register`, `rux set` or `rux del`. The gateway can describe a project and
show you what it looks like; changing one stays a deliberate act at the command
line. Write-back (`save_components`) is Phase 4 of #267 and will arrive with a
provenance flag on the records it writes, not as a general mutation surface.

Two further rules follow from #267's progressive-disclosure design:

- **Raw point data never enters an agent's context.** `rux get clouds.cloud`
  streams a binary PCD; `query_db` refuses it and says so, naming
  `clouds.cloud.metadata` and `render_view()` instead.
- **The project path is server configuration, not a tool argument.** A gateway
  process is bound to one `.rux` file for its lifetime, so no tool schema
  carries a path an agent could point somewhere else.

## Install

Its own venv, prebuilt wheels, one runtime dependency. It is deliberately
**not** wired into CMake and **not** nix-packaged.

```bash
cd tools/rux-mcp
python3 -m venv .venv
.venv/bin/pip install -e '.[dev]'
```

> **Inside `nix develop`**, the dev shell exports a `PYTHONPATH` that shadows
> the venv's own packages. Run the server as `env -u PYTHONPATH .venv/bin/rux-mcp …`,
> or start it from a shell outside the dev environment.

### SDK choice

The official MCP Python SDK (`mcp>=2`), not the standalone `fastmcp` package.
Both offer the same decorator ergonomics — `fastmcp` v1 *was* the SDK's
`mcp.server.fastmcp` — but the official SDK tracks the specification directly
and is one dependency instead of two. Note that in `mcp` 2.x the class was
renamed: `FastMCP` → `MCPServer` (`mcp.server.mcpserver`). Pinned `>=2.0,<3`
because the 1.x → 2.x rename is a hard break.

## Run

```bash
rux-mcp --project scan.rux --rux-bin build/apps/rux/rux
```

| Flag | Env | Default | Meaning |
|---|---|---|---|
| `-p/--project` | `RUX_PROJECT` | `./project.rux` | the `.rux` file this gateway serves |
| `--rux-bin` | `RUX_BIN` | `rux` on `PATH` | the executable to shell out to |
| `--timeout` | `RUX_MCP_TIMEOUT` | `120` | per-call budget in seconds; `render_view` gets 4× |
| `--render-dir` | `RUX_MCP_RENDER_DIR` | a private temp dir | where `render_view` writes PNGs |
| `--transport` | `RUX_MCP_TRANSPORT` | `stdio` | `stdio`, `sse` or `streamable-http` |

A missing `rux` binary is fatal at startup. A missing *project* is only a
warning — the file may appear later, and every tool reports it precisely until
it does.

### Register with Claude Code

```bash
claude mcp add rux -- /abs/path/to/tools/rux-mcp/.venv/bin/rux-mcp \
  --project /abs/path/to/scan.rux --rux-bin /abs/path/to/build/apps/rux/rux
```

Or, in a `mcpServers` config block:

```json
{
  "mcpServers": {
    "rux": {
      "command": "/abs/path/to/tools/rux-mcp/.venv/bin/rux-mcp",
      "args": ["--project", "/abs/path/to/scan.rux"],
      "env": { "RUX_BIN": "/abs/path/to/build/apps/rux/rux" }
    }
  }
}
```

## Tool surface

| Tool | Backed by | Returns |
|---|---|---|
| `project_info()` | `rux info --json` | schema version, frame counts, clouds, meshes, passports |
| `list_clouds()` | `rux info --json` | every stored cloud with type, point count, label definitions |
| `list_meshes()` | `rux info --json` | meshes with vertex/face counts |
| `label_definitions(cloud)` | `rux info --json` | `{id: class name}` for one label cloud |
| `list_frames(limit, offset)` | `rux get frames` | paginated sensor-frame node ids |
| `get_frame(node_id)` | `rux get frames.ID` | dimensions, intrinsics, 4×4 world pose |
| `list_components()` | `rux export csv` | building-component inventory |
| `get_component(component_id)` | `rux export csv` | one component, by GUID or name |
| `list_passports()` | `rux info --json` | material-passport GUIDs |
| `get_passport(guid)` | `rux get materials.GUID` | one passport with its property values |
| `query_db(path)` | `rux get <path>` | allowlisted JSON query over the whole database |
| `render_view(view, layers, …)` | `rux render` | PNG image content **plus** the file paths |
| `analyze_quality(cloud, planes, …)` | `rux analyze quality` | flatness RMS / thickness p90 |
| `analyze_accuracy(ground_truth_path, …)` | `rux analyze accuracy` | accuracy / completeness / F-score |
| `pipeline_log(limit)` | `rux log --json` | stage history with parameters and status |
| `validate_project(stage)` | `rux validate --json` | integrity report, or one stage's input contract |

Resources (cheap summaries, fetched without a tool call):
`rux://project/summary`, `rux://project/validation`, `rux://project/components`,
`rux://project/log`, `rux://guide/query-paths`.

### `query_db` allowlist

`rux get` is path-addressable over the whole database and will happily write
megabytes of binary PCD, PNG or PLY to stdout. `query_db` allows a path only
when it is known to yield JSON:

| Collection | Reachable | Refused |
|---|---|---|
| `clouds` | `clouds`, `clouds.N.{metadata,type,point_count}` | `clouds.N` (binary PCD) |
| `frames` | `frames`, `frames.ID`, `.{metadata,pose,intrinsics,has_pose,timestamp}` | `.{color,depth,confidence,image}` |
| `labels` | `labels`, `labels.ID.metadata` | `labels.ID` (PNG raster) |
| `meshes` | `meshes`, `meshes.N`, `.{metadata,format,vertex_count,polygon_count}` | `.{data,texture,material}` |
| `materials` | anything (all JSON) | — |
| `log`, `projects` | anything (all JSON) | — |
| `panoramas` | `panoramas`, `panoramas.N.metadata` | `panoramas.N`, `.image` |

Path components are restricted to `[A-Za-z0-9_.*?-]` and may not start with
`-`, so a component cannot be read as a CLI flag; commands are built as an argv
list and never go through a shell.

### Known CLI/gateway gaps

- **Building components have no `rux get` route.** The router registry
  (`apps/rux/src/database/resource_router.cpp`) implements
  `clouds, frames, labels, log, materials, meshes, panoramas, projects` — the
  `components` and `passports` entries in `rux get --help` are stale, and
  passports actually live under `materials`. `list_components()`/
  `get_component()` therefore read `rux export csv` (which opens the database
  read-only) and parse the component rows. Swap `rux_mcp/components.py` for a
  `rux get components` call once gap 3 of #267 is closed.
- **Component *geometry* is not exposed at all** — only properties. Seeing
  where a component is means `render_view(layers='components')`.
- **`rux` logs to stdout, not stderr.** A schema warning ("Project schema is
  v11 but this build expects v12") is printed *ahead of* the JSON that
  `--json` produces, and stderr stays empty. `runner.strip_log_lines()`
  removes spdlog-formatted lines before parsing, and a failed call falls back
  to stdout for its diagnostics. Worth fixing in `rux` itself — until then,
  never `json.loads()` `rux` output directly.

## Demo

Against the office scan (`afb3234950`, schema v11, 238 frames, 9 components),
driven through an in-memory MCP client:

```
>>> tools/list
project_info, list_clouds, list_meshes, label_definitions, list_frames,
get_frame, list_components, get_component, list_passports, get_passport,
query_db, render_view, analyze_quality, analyze_accuracy, pipeline_log,
validate_project

>>> resources/list
rux://project/summary, rux://project/validation, rux://project/components,
rux://project/log, rux://guide/query-paths

>>> label_definitions(cloud='labels')
{
  "0": "ceiling", "1": "floor", "2": "wall", "3": "door frame",
  "4": "window", "5": "radiator", "6": "table", "7": "chair",
  "8": "shelf", "9": "bench", "10": "ceiling lamp", "11": "desk lamp",
  "12": "electrical outlet"
}

>>> list_components()
{
  "count": 9,
  "components": [
    {
      "kind": "component",
      "id": "7b463945-2535-463f-add5-074cede33e22",
      "component_name": "window_1",
      "component_type": "window",
      "window_pane_count": "0",
      "window_operable": "true"
    },
    ...
  ]
}

>>> query_db('frames.100.intrinsics')
{
  "path": "frames.100.intrinsics",
  "result": {
    "cx": 318.8351, "cy": 239.1002,
    "fx": 531.5749, "fy": 531.5749,
    "width": 640, "height": 480,
    "local_transform": [0,0,1,0, -1,0,0,0, 0,-1,0,0, 0,0,0,1]
  }
}

>>> query_db('clouds.cloud')            # refused, with the way forward
Error executing tool query_db: 'clouds.cloud' is not exposed: 'clouds.<name>'
streams the raw binary PCD. Use 'clouds.<name>.metadata' for the summary, or
render_view() to see the geometry.

>>> validate_project(stage='mesh')
{"error_count": 0, "issues": [], "ok": true, "warning_count": 0, "path": "..."}

>>> analyze_quality()
{
  "flatness_rms": 0.0223,
  "flatness_rms_max": 0.0372,
  "labeled_points": 201697,
  "plane_count": 38,
  "planes": [ ... ]
}

>>> render_view(view='top', layers='cloud', size='800x600')
{
  "view": "top",
  "layers": ["cloud"],
  "size": "800x600",
  "image_count": 1,
  "paths": ["/tmp/rux-mcp-renders-…/bfb4cfa997ce/view.png"]
}
[image] image/png, 499564 base64 chars
```

That last call is the whole point of the gateway — the agent gets this back as
image content, not as coordinates:

![Top-down render of the office scan returned by render_view()](docs/demo-top-view.png)

`view='orbit:8'` writes eight numbered files and returns the first
`max_inline_images` (default 4) inline; `inline=False` returns paths only.

## Tests

```bash
env -u PYTHONPATH .venv/bin/python -m pytest -q
```

The unit tests replace `rux` with a shell script that records its argv and
replays canned stdout/stderr/exit status, so the whole subprocess path is
exercised — including the three failure modes every tool must survive: a
missing project file, a non-zero exit (stderr is forwarded to the model), and a
timeout. Tools are driven through a real MCP client against an in-memory
server, so the assertions cover what an agent actually receives.

One integration test runs against a real binary and a real project. It skips
unless both are found:

```bash
RUX_BIN=../../build/apps/rux/rux RUX_PROJECT=/path/to/scan.rux \
  env -u PYTHONPATH .venv/bin/python -m pytest -q -m integration
```

## Implementation notes

- `runner.py` — the only place that spawns `rux`. Distinguishes
  `ProjectMissingError`, `RuxNotFoundError`, `RuxTimeoutError` and `RuxError`,
  each carrying the argv, the exit status and a de-coloured stderr tail.
- `paths.py` — the `query_db` allowlist, deny-by-default, with refusal messages
  that name the JSON-bearing alternative so an agent can correct itself.
- `components.py` — the `rux export csv` stopgap for the component inventory.
- `server.py` — tools and resources. Every tool is wrapped so the gateway's own
  exceptions become `ToolError`: the MCP SDK deliberately withholds the text of
  *unexpected* exceptions from the model (a raw `RuntimeError` reaches it as
  "Error executing tool X" and nothing else), so anything an agent could act on
  has to be raised as `ToolError`.
- List-returning tools wrap their result in a dict (`{"count": …, "clouds": …}`).
  A bare list return is exploded by the SDK into one content block per element,
  which reads terribly for a 16-element pose matrix.
