# rux-mcp — an MCP gateway over the `rux` CLI

`rux-mcp` exposes one ReUseX project (`*.rux`) to an LLM agent over the
[Model Context Protocol](https://modelcontextprotocol.io). It is a thin Python
adapter: every tool shells out to the `rux` command line and returns what `rux`
already knows how to print. No pipeline logic lives here.

This is **Phase 2 of [#267](https://github.com/pfmephisto/ReUseX/issues/267)**
(agent-driven simplified modeling). Phase 1 — `rux render`, the agent's "eyes" —
already shipped, so the gateway can hand an agent a picture of the scan.
The Blender / blender-mcp half of #267 is a later phase and is not part of this
package.

## Read-only unless you say otherwise

A default `rux-mcp` runs nothing that changes anything: no `rux import`,
`create`, `edit`, `optimize`, `register`, `set` or `del`. Starting it with
`--write-mode stages` or `--write-mode full` unlocks a deny-by-default
catalogue of `rux` commands the agent can then run as background jobs — see
[The write surface](#the-write-surface). The gate is **registration**, not
refusal: below its tier a mutating tool is not advertised at all, so a
read-only server cannot even be asked.

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
| `--write-mode` | `RUX_MCP_WRITE_MODE` | `none` | `none`, `stages` or `full` — see below |
| `--job-log-dir` | `RUX_MCP_JOB_LOG_DIR` | beside the render dir | full output of each background job |
| `--transport` | `RUX_MCP_TRANSPORT` | `stdio` | `stdio`, `sse` or `streamable-http` |

A missing `rux` binary is fatal at startup. A missing *project* is only a
warning — the file may appear later, and every tool reports it precisely until
it does. The write mode is printed on stderr at startup, on its own line, and
`full` additionally prints a warning.

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
| `list_commands()` | the catalogue | every `rux` command, its tier, and whether this server may run it |
| `command_help(command)` | `rux <command> --help` | the real flag list, out of the binary |

`list_commands` and `command_help` are registered at every write mode,
including `none` — knowing what is blocked is how an agent learns to ask the
operator for it. The five tools that actually run something
(`run_command`, `job_status`, `job_output`, `list_jobs`, `cancel_job`) exist
only above `--write-mode none`.

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

## The write surface

Off by default. `--write-mode` picks a tier, and the tiers are drawn by **what
is at risk**, not by how the CLI happens to be grouped:

| Mode | Adds | Why here |
|---|---|---|
| `none` *(default)* | — | nothing the gateway runs changes a project or writes a file |
| `stages` | `create *`, `optimize`, `register`, `align 360`, `edit downsample`, `export *` | adds derived data; recoverable by re-running the stage |
| `full` | `import *`, `set`, `del`, `edit perturb-poses`, `assemble` | irreversible or destructive; must be asked for by name |

`export *` sits at `stages` rather than `none` because it writes files to a
path the agent chooses — the database is untouched, but the filesystem is not.
(The gateway's own `list_components()` shells out to `export csv` internally,
to a private temp file; that is not the same capability.)

### Generic passthrough, not one tool per stage

`rux create planes` alone has a dozen flags, and they move. Mirroring them here
would duplicate CLI defaults, which [STANDARDS §4](../../docs/STANDARDS.md)
forbids, and would be stale by the next release. So the gateway exposes the
catalogue instead:

```
list_commands()                    -> what exists, what this server may run
command_help('create planes')      -> the real --help, out of the binary
run_command('create planes', options={'grid-size': 0.05})
```

`options` are long flags without the dashes. They render as `--flag=value` in a
**single** argv element, so a value can never be read as the next flag;
booleans become presence/absence (`{'cuda': True}` → `--cuda`, `False` → not
passed); a list repeats the flag. `arguments` are positionals, and one starting
with `-` is refused rather than passed on. Nothing goes through a shell.

### Jobs, because `rux create mesh` outlives an MCP call

Commands are **submitted**, not awaited. `run_command` returns a job id;
`wait_seconds` (default 10, max 300) collapses the short stages into a single
round trip, and `job_status` / `job_output` / `list_jobs` / `cancel_job` handle
the rest. Output is readable *while* the job runs, and the untruncated log is
always on disk at the job's `log_path`.

**One job at a time.** `ProjectDB` is sqlite and explicitly not thread-safe;
two writers on one project is corruption. A second submit is refused with the
running job named, rather than queued behind it — an agent that is waiting
should know it is waiting.

Over-budget jobs are killed by a **watchdog timer**, not by a deadline checked
inside the output read loop: `rux create mesh` is silent for the whole MIP
solve and would otherwise never trip it. Cancel and timeout both signal the
whole **process group** — `rux create dense` shells out to OpenMVS, and a
surviving grandchild would hold the stdout pipe open long after the job was
reported dead.

A non-zero `rux` exit is a *job result*, not a tool error: the agent gets
`state: "failed"`, the exit status and the output, which is what it needs to
correct itself.

### `rux` prompts; gateways have no TTY

`rux del` and `rux edit perturb-poses` ask for confirmation interactively. The
runner closes stdin for **every** invocation, so an unconfirmed destructive
command aborts instead of hanging. Confirming is explicit and deliberate:

```
run_command('del', arguments=['clouds.scratch'], options={'force': True})
  -> refused: "'del' destroys data and asks for confirmation on a terminal,
      which a gateway does not have. Re-run it with options={'yes': true} …"

run_command('del', arguments=['clouds.scratch'],
            options={'force': True, 'yes': True})     # runs
```

### Deliberately not done

- **No pre-write backup.** A `VACUUM INTO` before `del`/`import` would be cheap
  insurance, but it makes the gateway the owner of project state it otherwise
  never touches. Copy the project yourself before pointing a `full` server at
  it.
- **Job history is in-process** and dies with the server. `rux` already writes
  a pipeline log entry for every stage (`pipeline_log()`), so persisting a
  second history under the project would be a second source of truth.
- **`create gsplat` stays at `stages`.** It trains for minutes to hours, but
  runtime is not risk — it only adds a derived cloud. It gets a much larger
  default timeout (6 h) instead of a tier of its own.
- **No `save_components` yet.** Writing an agent-built inventory back into
  `building_components` with a provenance flag is Phase 4 of #267; on top of
  this it is one more catalogue entry plus a `rux` route, not a new mechanism.

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

### Running a stage

Against a disposable copy of the apartment scan (136 496 points, schema v9),
with the server started at `--write-mode stages`:

```
>>> list_commands()                                        # abridged
{
  "write_mode": "stages",
  "runnable_count": 33,
  "commands": [
    { "command": "create planes", "mode": "stages", "runnable": true,
      "mutates_project": true, "default_timeout_seconds": 3600.0,
      "summary": "Detect and segment planar surfaces" },
    { "command": "create gsplat", "mode": "stages", "runnable": true,
      "default_timeout_seconds": 21600.0,
      "summary": "Train a 3D Gaussian Splatting model (GPU; minutes to hours)" },
    { "command": "del", "mode": "full", "runnable": false,
      "positional": "<path>", "needs_confirmation_option": "yes",
      "summary": "Delete database records (irreversible)" }
  ]
}

>>> analyze_quality()                                      # before
{"flatness_rms": 0.0214, "thickness_p90": 0.0353, "plane_count": 18,
 "labeled_points": 68716, "total_points": 136496}

>>> run_command('del', arguments=['clouds.planes'])
Error executing tool run_command: 'del' needs write mode 'full' but this
gateway runs in 'stages'. Restart it with --write-mode full to allow it.

>>> run_command('create planes')
{
  "job_id": "6a57f8bfb9dd",
  "command": "create planes",
  "state": "succeeded",
  "done": true,
  "argv": ["…/rux", "-p", "…/scratch.rux", "create", "planes"],
  "log_path": "…/jobs/6a57f8bfb9dd.log",
  "elapsed_seconds": 3.2,
  "exit_status": 0,
  "output_tail": "… [Processing: Region Growing]  75%| 101837/136496 …",
  "next": "the project changed: re-read rux://project/summary, and check the
           result with validate_project(), analyze_quality() or render_view()."
}

>>> analyze_quality()                                      # after
{"flatness_rms": 0.0167, "thickness_p90": 0.0278, "plane_count": 35,
 "labeled_points": 78124, "total_points": 136496}
```

That last pair is the loop #267 asks for: the agent acted, and got an objective
number back saying whether it helped.

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

`test_jobs.py` drives real subprocesses rather than mocks — the module's whole
subject is process lifetime (cancel, timeout, a child that outlives its
parent), which a mock cannot exercise.

Integration tests run against a real binary and a real project, and skip
unless both are found:

```bash
RUX_BIN=../../build/apps/rux/rux RUX_PROJECT=/path/to/scan.rux \
  env -u PYTHONPATH .venv/bin/python -m pytest -q -m integration
```

Those are read-only. Running a real *stage* through the write surface is a
further opt-in, because it copies the whole project and then spends real time
in the segmenter. The original is never touched — the copy lands in pytest's
`tmp_path`:

```bash
RUX_BIN=../../build/apps/rux/rux RUX_PROJECT=/path/to/scan.rux \
  RUX_MCP_WRITE_TEST=1 \
  env -u PYTHONPATH .venv/bin/python -m pytest -q -m integration
```

## Implementation notes

- `runner.py` — the only place that spawns `rux`. Distinguishes
  `ProjectMissingError`, `RuxNotFoundError`, `RuxTimeoutError` and `RuxError`,
  each carrying the argv, the exit status and a de-coloured stderr tail. stdin
  is closed for every invocation, so a command that wants a TTY confirmation
  aborts rather than hanging.
- `paths.py` — the `query_db` allowlist, deny-by-default, with refusal messages
  that name the JSON-bearing alternative so an agent can correct itself.
- `commands.py` — the command catalogue: which `rux` commands exist, what tier
  each needs, its timeout band, and the argv builder that validates a request.
  Deny-by-default on the command key, same shape as the `query_db` allowlist.
- `jobs.py` — the background runner: one job at a time, watchdog timeouts,
  process-group termination, on-disk logs, live output.
- `components.py` — the `rux export csv` stopgap for the component inventory.
- `server.py` — tools and resources. Every tool is wrapped so the gateway's own
  exceptions become `ToolError`: the MCP SDK deliberately withholds the text of
  *unexpected* exceptions from the model (a raw `RuntimeError` reaches it as
  "Error executing tool X" and nothing else), so anything an agent could act on
  has to be raised as `ToolError`.
- List-returning tools wrap their result in a dict (`{"count": …, "clouds": …}`).
  A bare list return is exploded by the SDK into one content block per element,
  which reads terribly for a 16-element pose matrix.
