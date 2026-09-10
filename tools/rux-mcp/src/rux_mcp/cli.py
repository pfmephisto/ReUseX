# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Entry point for the ``rux-mcp`` server process."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from rux_mcp.commands import WriteMode
from rux_mcp.runner import DEFAULT_TIMEOUT, RuxNotFoundError, RuxRunner
from rux_mcp.server import build_server

#: What each ``--write-mode`` tier adds, for the banner and the help text.
WRITE_MODE_HELP = {
    WriteMode.none: "read-only; no rux command that changes anything is exposed",
    WriteMode.stages: "read-only plus pipeline stages (create/optimize/register/"
    "align/edit downsample) and exports",
    WriteMode.full: "everything catalogued, including imports, set/del and "
    "perturb-poses — irreversible",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rux-mcp",
        description=(
            "Model Context Protocol gateway over one ReUseX .rux project. The "
            "project is bound at startup, never per call. Read-only unless "
            "--write-mode says otherwise."
        ),
    )
    parser.add_argument(
        "-p",
        "--project",
        default=os.environ.get("RUX_PROJECT", "project.rux"),
        help="path to the .rux project (env: RUX_PROJECT, default: ./project.rux)",
    )
    parser.add_argument(
        "--rux-bin",
        default=os.environ.get("RUX_BIN", "rux"),
        help="rux executable to shell out to (env: RUX_BIN, default: rux on PATH)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=float(os.environ.get("RUX_MCP_TIMEOUT", DEFAULT_TIMEOUT)),
        help=(
            "per-call time budget in seconds (env: RUX_MCP_TIMEOUT, default: "
            f"{DEFAULT_TIMEOUT:g}); render_view gets four times this"
        ),
    )
    parser.add_argument(
        "--render-dir",
        default=os.environ.get("RUX_MCP_RENDER_DIR"),
        help=(
            "directory for render_view output (env: RUX_MCP_RENDER_DIR, "
            "default: a private temp directory)"
        ),
    )
    parser.add_argument(
        "--write-mode",
        choices=[mode.value for mode in WriteMode],
        default=os.environ.get("RUX_MCP_WRITE_MODE", WriteMode.none.value),
        help=(
            "how much of the rux command catalogue to expose (env: "
            "RUX_MCP_WRITE_MODE, default: none). "
            + "; ".join(f"{mode.value}: {text}" for mode, text in WRITE_MODE_HELP.items())
        ),
    )
    parser.add_argument(
        "--job-log-dir",
        default=os.environ.get("RUX_MCP_JOB_LOG_DIR"),
        help=(
            "directory for the full output of each background job (env: "
            "RUX_MCP_JOB_LOG_DIR, default: beside the render directory). "
            "Only used above --write-mode none"
        ),
    )
    parser.add_argument(
        "--transport",
        choices=("stdio", "sse", "streamable-http"),
        default=os.environ.get("RUX_MCP_TRANSPORT", "stdio"),
        help="MCP transport (env: RUX_MCP_TRANSPORT, default: stdio)",
    )
    parser.add_argument(
        "--name",
        default="rux",
        help="MCP server name reported to the client (default: rux)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        write_mode = WriteMode(args.write_mode)
    except ValueError:
        # Reachable only via RUX_MCP_WRITE_MODE, which argparse never checks.
        parser.error(
            f"invalid write mode {args.write_mode!r}; expected one of "
            + ", ".join(mode.value for mode in WriteMode)
        )

    project = Path(args.project).expanduser()
    runner = RuxRunner(project=project, binary=args.rux_bin, timeout=args.timeout)

    # Fail fast on a missing binary: without rux there is nothing to serve.
    # A missing *project* is only a warning — the file may appear later, and
    # every tool reports it precisely if it has not.
    try:
        resolved = runner.resolve_binary()
    except RuxNotFoundError as exc:
        print(f"rux-mcp: {exc}", file=sys.stderr)
        return 2
    print(f"rux-mcp: using {resolved}", file=sys.stderr)

    if not project.exists():
        print(
            f"rux-mcp: warning: project {project} does not exist yet; tools "
            "will report this until it does",
            file=sys.stderr,
        )

    render_dir = Path(args.render_dir).expanduser() if args.render_dir else None
    job_log_dir = Path(args.job_log_dir).expanduser() if args.job_log_dir else None
    server = build_server(
        runner,
        render_dir=render_dir,
        job_log_dir=job_log_dir,
        write_mode=write_mode,
        name=args.name,
    )

    # The write mode is the one thing an operator must be able to see at a
    # glance, so it is stated on its own line rather than in a parenthesis.
    print(
        f"rux-mcp: serving {project} over {args.transport}", file=sys.stderr
    )
    print(
        f"rux-mcp: write mode '{write_mode.value}' — "
        f"{WRITE_MODE_HELP[write_mode]}",
        file=sys.stderr,
    )
    if write_mode is WriteMode.full:
        print(
            "rux-mcp: WARNING: an agent connected to this server can delete "
            "records and overwrite poses in the project.",
            file=sys.stderr,
        )
    server.run(transport=args.transport)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
