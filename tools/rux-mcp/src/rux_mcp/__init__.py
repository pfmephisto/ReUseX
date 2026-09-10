# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Read-only MCP gateway over the ReUseX ``rux`` command line interface.

The gateway is a thin adapter: every tool shells out to ``rux`` and returns
what ``rux`` already knows how to print.  No pipeline logic lives here, and no
tool mutates a project — see :mod:`rux_mcp.server` for the tool surface.
"""

from rux_mcp.paths import PathNotAllowed, check_query_path
from rux_mcp.runner import (
    ProjectMissingError,
    RuxError,
    RuxNotFoundError,
    RuxRunner,
    RuxTimeoutError,
)

__version__ = "0.1.0"

__all__ = [
    "PathNotAllowed",
    "ProjectMissingError",
    "RuxError",
    "RuxNotFoundError",
    "RuxRunner",
    "RuxTimeoutError",
    "__version__",
    "check_query_path",
]
