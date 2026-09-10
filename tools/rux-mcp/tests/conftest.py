# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Shared fixtures: a fake ``rux`` on PATH, and discovery of a real one."""

from __future__ import annotations

import stat
from pathlib import Path

import pytest

from rux_mcp.runner import RuxRunner

#: A tiny shell script that impersonates ``rux``.  Tests write the canned
#: stdout/stderr/exit status next to it, so no test needs a real binary or a
#: real database while still exercising the whole subprocess path.
FAKE_RUX = """#!/bin/sh
here=$(dirname "$0")
printf '%s' "$*" >> "$here/calls.txt"
printf '\\n' >> "$here/calls.txt"
[ -f "$here/stderr" ] && cat "$here/stderr" >&2
[ -f "$here/stdout" ] && cat "$here/stdout"
exit $(cat "$here/status" 2>/dev/null || echo 0)
"""


class FakeRux:
    """Handle on the fake ``rux`` script: set replies, read back the calls."""

    def __init__(self, directory: Path) -> None:
        self.dir = directory
        self.path = directory / "rux"
        self.path.write_text(FAKE_RUX)
        self.path.chmod(self.path.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP)

    def reply(self, stdout: str = "", *, stderr: str = "", status: int = 0) -> None:
        (self.dir / "stdout").write_text(stdout)
        (self.dir / "stderr").write_text(stderr)
        (self.dir / "status").write_text(str(status))

    @property
    def calls(self) -> list[str]:
        calls = self.dir / "calls.txt"
        if not calls.exists():
            return []
        return [line for line in calls.read_text().splitlines() if line]


@pytest.fixture
def project(tmp_path: Path) -> Path:
    """An existing (empty) file standing in for a .rux project."""
    path = tmp_path / "scan.rux"
    path.write_bytes(b"")
    return path


@pytest.fixture
def fake_rux(tmp_path: Path) -> FakeRux:
    directory = tmp_path / "bin"
    directory.mkdir()
    return FakeRux(directory)


@pytest.fixture
def runner(project: Path, fake_rux: FakeRux) -> RuxRunner:
    return RuxRunner(project=project, binary=str(fake_rux.path), timeout=10.0)
