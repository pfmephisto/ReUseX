# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""MCP gateway over the ReUseX ``rux`` command line interface.

The gateway is a thin adapter: every tool shells out to ``rux`` and returns
what ``rux`` already knows how to print.  No pipeline logic lives here.

It is read-only unless started with ``--write-mode stages`` or ``--write-mode
full``, which unlock a deny-by-default catalogue of ``rux`` commands run as
background jobs — see :mod:`rux_mcp.commands` and :mod:`rux_mcp.jobs`.
"""

from rux_mcp.commands import CommandNotAllowed, WriteMode
from rux_mcp.paths import PathNotAllowed, check_query_path
from rux_mcp.runner import (
    ProjectMissingError,
    RuxError,
    RuxNotFoundError,
    RuxRunner,
    RuxTimeoutError,
)

__version__ = "0.2.0"

__all__ = [
    "CommandNotAllowed",
    "PathNotAllowed",
    "ProjectMissingError",
    "RuxError",
    "RuxNotFoundError",
    "RuxRunner",
    "RuxTimeoutError",
    "WriteMode",
    "__version__",
    "check_query_path",
]
