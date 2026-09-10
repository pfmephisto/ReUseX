# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Entry point for the ``rux-mcp`` server process."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from rux_mcp.runner import DEFAULT_TIMEOUT, RuxNotFoundError, RuxRunner
from rux_mcp.server import build_server


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rux-mcp",
        description=(
            "Read-only Model Context Protocol gateway over one ReUseX .rux "
            "project. The project is bound at startup, never per call."
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
    args = build_parser().parse_args(argv)

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
    server = build_server(runner, render_dir=render_dir, name=args.name)
    print(
        f"rux-mcp: serving {project} over {args.transport} (read-only)",
        file=sys.stderr,
    )
    server.run(transport=args.transport)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
