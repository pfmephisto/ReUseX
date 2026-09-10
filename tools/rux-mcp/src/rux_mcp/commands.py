# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""The catalogue of `rux` commands the gateway is willing to run.

Deny-by-default, exactly like the ``query_db`` path allowlist: a command an
agent can invoke has to be named here, and it is only reachable when the server
was started at or above the command's write mode.

The three modes correspond to what is actually at risk, not to how the CLI
happens to be grouped:

``none``
    Read-only. The default. Nothing the gateway runs changes a project or
    writes a file the user did not ask for.
``stages``
    Pipeline stages and exports. These add derived data and are recoverable by
    re-running the stage; exports write files outside the database.
``full``
    Imports, direct database writes (``set``/``del``), pose perturbation and
    assembly. Irreversible or destructive, so it has to be asked for by name.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Iterable, Mapping


class WriteMode(str, Enum):
    """How much of the catalogue a server exposes."""

    none = "none"
    stages = "stages"
    full = "full"

    @property
    def rank(self) -> int:
        return {"none": 0, "stages": 1, "full": 2}[self.value]

    def allows(self, other: "WriteMode") -> bool:
        return self.rank >= other.rank


class CommandNotAllowed(ValueError):
    """The command is unknown, or above this server's write mode."""


#: A flag name as CLI11 would accept it, without the leading dashes.
_FLAG = re.compile(r"^[A-Za-z][A-Za-z0-9-]*$")

#: Characters that must never reach an argv element.
_CONTROL = re.compile(r"[\x00-\x1f\x7f]")

#: A light budget for metadata-only work, in seconds.
QUICK_TIMEOUT = 600.0
#: Reconstruction, ML inference and solver stages measured in minutes.
HEAVY_TIMEOUT = 3600.0
#: `create gsplat` trains a model: minutes on a small scan, hours on a large
#: one.  It stays at the `stages` tier — it only adds a derived cloud, and a
#: separate tier would gate the wrong thing (risk, not runtime) — but a budget
#: sized for a MIP solve would kill it mid-training, so it gets its own band.
TRAINING_TIMEOUT = 21600.0


@dataclass(frozen=True)
class Command:
    """One invocable `rux` command."""

    key: str
    argv: tuple[str, ...]
    mode: WriteMode
    summary: str
    default_timeout: float = QUICK_TIMEOUT
    #: True when the command writes to the bound `.rux` project.
    mutates_project: bool = True
    #: Positional arguments the command requires, for the error message.
    positional_hint: str = ""


def _c(
    key: str,
    mode: WriteMode,
    summary: str,
    *,
    timeout: float = QUICK_TIMEOUT,
    mutates: bool = True,
    positional: str = "",
) -> Command:
    return Command(
        key=key,
        argv=tuple(key.split()),
        mode=mode,
        summary=summary,
        default_timeout=timeout,
        mutates_project=mutates,
        positional_hint=positional,
    )


_READ = WriteMode.none
_STAGES = WriteMode.stages
_FULL = WriteMode.full

CATALOGUE: dict[str, Command] = {
    command.key: command
    for command in (
        # -- read-only, listed so `command_help` can reach them ------------
        _c("info", _READ, "Project summary", mutates=False),
        _c("get", _READ, "Path-addressed database query", mutates=False),
        _c("log", _READ, "Pipeline execution history", mutates=False),
        _c("validate", _READ, "Integrity / stage-contract checks", mutates=False),
        _c("render", _READ, "Off-screen render to PNG", mutates=False),
        _c("analyze quality", _READ, "Flatness / thickness metrics", mutates=False),
        _c("analyze accuracy", _READ, "Ground-truth scoring", mutates=False),
        # -- pipeline stages ------------------------------------------------
        _c(
            "create clouds",
            _STAGES,
            "Back-project depth frames into a fused cloud",
            timeout=HEAVY_TIMEOUT,
        ),
        _c(
            "create dense",
            _STAGES,
            "Dense MVS cloud via OpenMVS",
            timeout=HEAVY_TIMEOUT,
        ),
        _c(
            "create annotate",
            _STAGES,
            "Run ML inference over the stored sensor frames",
            timeout=HEAVY_TIMEOUT,
        ),
        _c(
            "create annotate-360",
            _STAGES,
            "Segment 360 panoramas with SAM3",
            timeout=HEAVY_TIMEOUT,
        ),
        _c("create material", _STAGES, "Create one blank material passport"),
        _c("create project", _STAGES, "Project 2D labels onto the 3D cloud",
           timeout=HEAVY_TIMEOUT),
        _c("create planes", _STAGES, "Detect and segment planar surfaces",
           timeout=HEAVY_TIMEOUT),
        _c("create rooms", _STAGES, "Segment rooms via Leiden clustering"),
        _c("create instances", _STAGES, "Split labels into spatial instances",
           timeout=HEAVY_TIMEOUT),
        _c("create materials", _STAGES, "One material passport per instance"),
        _c(
            "create mesh",
            _STAGES,
            "Cell-complex reconstruction + MIP solve to a watertight mesh",
            timeout=HEAVY_TIMEOUT,
        ),
        _c("create texture", _STAGES, "Texture the mesh from frame colours",
           timeout=HEAVY_TIMEOUT),
        _c("create windows", _STAGES, "Derive window building components"),
        _c("create gsplat", _STAGES, "Train a 3D Gaussian Splatting model "
           "(GPU; minutes to hours)", timeout=TRAINING_TIMEOUT),
        _c(
            "align 360",
            _STAGES,
            "Content-based 6-DoF pose refinement of the 360 panoramas",
            timeout=HEAVY_TIMEOUT,
        ),
        _c(
            "optimize",
            _STAGES,
            "Plane-landmark pose-graph optimization",
            timeout=HEAVY_TIMEOUT,
        ),
        _c(
            "register",
            _STAGES,
            "Joint pairwise registration of the stored poses",
            timeout=HEAVY_TIMEOUT,
        ),
        _c("edit downsample", _STAGES, "Voxel-grid downsample, siblings in sync"),
        # -- exports: write files, never the project ------------------------
        _c("export ply", _STAGES, "Write a PLY point cloud", mutates=False),
        _c("export e57", _STAGES, "Write an E57 point cloud", mutates=False),
        _c("export materialepas", _STAGES, "Write passports as JSON",
           mutates=False),
        _c("export csv", _STAGES, "Write the element CSV", mutates=False),
        _c("export rhino", _STAGES, "Write a Rhino .3dm", mutates=False),
        _c("export semantic-images", _STAGES, "Write coloured label PNGs",
           mutates=False),
        _c("export speckle", _STAGES, "Upload to Speckle", mutates=False,
           timeout=HEAVY_TIMEOUT),
        _c("export colmap", _STAGES, "Write a COLMAP sparse model",
           mutates=False),
        # -- irreversible ---------------------------------------------------
        _c("import rtabmap", _FULL, "Import an RTABMap SLAM database",
           timeout=HEAVY_TIMEOUT, positional="<database.db>"),
        _c("import mushroom", _FULL, "Import a MuSHRoom capture",
           timeout=HEAVY_TIMEOUT, positional="<capture_dir>"),
        _c("import arkitscenes", _FULL, "Import an ARKitScenes scene",
           timeout=HEAVY_TIMEOUT, positional="<scene_dir>"),
        _c("import e57", _FULL, "Import an E57 cloud", timeout=HEAVY_TIMEOUT,
           positional="<cloud.e57>"),
        _c("import ply", _FULL, "Import a PLY cloud", timeout=HEAVY_TIMEOUT,
           positional="<cloud.ply>"),
        _c("import gsplat", _FULL, "Import a Gaussian splat .ply",
           timeout=HEAVY_TIMEOUT, positional="<splat.ply>"),
        _c("import materialepas", _FULL, "Import passports from JSON",
           positional="<passports.json>"),
        _c("import csv", _FULL, "Reimport an edited element CSV",
           positional="<elements.csv>"),
        _c("import 360", _FULL, "Import 360 panoramas", timeout=HEAVY_TIMEOUT,
           positional="<directory>"),
        _c("import photos", _FULL, "Import survey photos", timeout=HEAVY_TIMEOUT,
           positional="<directory>"),
        _c("set", _FULL, "Write one database value",
           positional="<path> [value]"),
        _c("del", _FULL, "Delete database records (irreversible)",
           positional="<path>"),
        _c(
            "edit perturb-poses",
            _FULL,
            "Inject synthetic drift into the stored poses (benchmark tool; "
            "destructive)",
        ),
        _c("assemble", _FULL, "Merge RTABMap databases into one",
           timeout=HEAVY_TIMEOUT, mutates=False, positional="<scan.db> [...]"),
    )
}

#: Commands that ask a yes/no question on a TTY.  The gateway runs `rux` with
#: stdin closed, so an unconfirmed one aborts instead of hanging — which is the
#: safe outcome.  Confirming means passing the flag explicitly.
NEEDS_CONFIRMATION: dict[str, str] = {
    "del": "yes",
    "edit perturb-poses": "yes",
}


@dataclass
class Invocation:
    """A validated argv, ready for :class:`~rux_mcp.runner.RuxRunner`."""

    command: Command
    args: list[str] = field(default_factory=list)
    timeout: float = QUICK_TIMEOUT


def available(mode: WriteMode, *, include_read: bool = True) -> list[Command]:
    """Every command reachable at ``mode``, in catalogue order."""
    return [
        command
        for command in CATALOGUE.values()
        if mode.allows(command.mode)
        and (include_read or command.mode is not WriteMode.none)
    ]


def resolve(key: str, mode: WriteMode) -> Command:
    """Look ``key`` up in the catalogue and check it against ``mode``."""
    cleaned = " ".join(str(key).split())
    command = CATALOGUE.get(cleaned)
    if command is None:
        raise CommandNotAllowed(
            f"unknown command {key!r}. Call list_commands() for what this "
            "server exposes."
        )
    if not mode.allows(command.mode):
        raise CommandNotAllowed(
            f"'{cleaned}' needs write mode '{command.mode.value}' but this "
            f"gateway runs in '{mode.value}'. Restart it with "
            f"--write-mode {command.mode.value} to allow it."
        )
    return command


def _render_value(flag: str, value: Any) -> list[str]:
    """Turn one option into argv elements.

    ``--flag=value`` rather than ``--flag value`` so a value can never be read
    as the next flag, and booleans become presence/absence.
    """
    if isinstance(value, bool):
        return [f"--{flag}"] if value else []
    if value is None:
        # JSON null for a flag the model means to switch on but has no value
        # for; CLI11 reads a bare `--flag` as the flag being present.
        return [f"--{flag}"]
    if isinstance(value, (list, tuple)):
        # CLI11 multi-value options take the flag once per value.
        rendered: list[str] = []
        for item in value:
            rendered.extend(_render_value(flag, item))
        return rendered
    # repr() rather than str() for floats so 1e-07 stays exact rather than
    # being rounded by str()'s shortest-repr rules on older interpreters.
    text = repr(value) if isinstance(value, float) else str(value)
    if _CONTROL.search(text):
        raise CommandNotAllowed(f"value for --{flag} contains a control character")
    return [f"--{flag}={text}"]


def build(
    key: str,
    mode: WriteMode,
    *,
    arguments: Iterable[str] | None = None,
    options: Mapping[str, Any] | None = None,
    timeout: float | None = None,
) -> Invocation:
    """Validate a request and return the argv to run.

    Raises:
        CommandNotAllowed: unknown command, insufficient write mode, or an
            argument that could be misread as a flag.
    """
    command = resolve(key, mode)
    args = list(command.argv)

    for raw in arguments or ():
        text = str(raw)
        if not text:
            raise CommandNotAllowed("positional arguments may not be empty")
        if _CONTROL.search(text):
            raise CommandNotAllowed(
                "positional arguments may not contain control characters"
            )
        if text.startswith("-"):
            raise CommandNotAllowed(
                f"positional argument {text!r} starts with '-' and would be "
                "read as a flag; pass flags through 'options' instead"
            )
        args.append(text)

    for flag, value in (options or {}).items():
        name = str(flag).lstrip("-")
        if not _FLAG.match(name):
            raise CommandNotAllowed(
                f"invalid option name {flag!r}: use the long flag name without "
                "dashes, e.g. 'grid-size'"
            )
        args.extend(_render_value(name, value))

    budget = command.default_timeout if timeout is None else float(timeout)
    if budget <= 0:
        raise CommandNotAllowed("timeout_seconds must be positive")

    return Invocation(command=command, args=args, timeout=budget)


def describe(mode: WriteMode) -> str:
    """Agent-readable rendering of the catalogue at ``mode``."""
    lines = [
        f"rux commands this gateway will run (write mode: {mode.value}).",
        "",
    ]
    for level, title in (
        (WriteMode.none, "read-only"),
        (WriteMode.stages, "pipeline stages and exports"),
        (WriteMode.full, "irreversible: imports, direct writes, deletion"),
    ):
        entries = [c for c in CATALOGUE.values() if c.mode is level]
        allowed = mode.allows(level)
        lines.append(f"[{level.value}] {title}" + ("" if allowed else "  (BLOCKED)"))
        for command in entries:
            marker = " " if allowed else "x"
            hint = f" {command.positional_hint}" if command.positional_hint else ""
            lines.append(f"  {marker} {command.key}{hint} — {command.summary}")
        lines.append("")
    if NEEDS_CONFIRMATION:
        lines.append(
            "These prompt for confirmation on a terminal; the gateway runs rux "
            "with stdin closed, so pass the flag explicitly or the command "
            "aborts without doing anything:"
        )
        for key, flag in NEEDS_CONFIRMATION.items():
            lines.append(f"  {key} -> options={{'{flag}': true}}")
        lines.append("")
    lines.append(
        "Use command_help('<key>') to read the real flag list from the binary, "
        "then run_command('<key>', options={...})."
    )
    return "\n".join(lines)
