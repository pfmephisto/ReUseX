# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Subprocess wrapper around the ``rux`` CLI.

Everything the MCP server does goes through :class:`RuxRunner`, so the three
failure modes an agent must be able to distinguish — the project file is gone,
``rux`` exited non-zero, ``rux`` hung — are handled in exactly one place and
surface as distinct exception types with the stderr tail attached.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

#: Default wall-clock budget for a single ``rux`` invocation, in seconds.
DEFAULT_TIMEOUT = 120.0

#: spdlog writes coloured diagnostics; strip the escapes before quoting them.
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

#: ``rux`` installs spdlog on **stdout**, not stderr, so a warning such as
#: "Project schema is v11 but this build expects v12" is printed ahead of the
#: JSON a ``--json`` command produces.  These lines have to come off before the
#: payload can be parsed.  Pattern: ``[2026-09-10 11:02:52.886] [rux] [warning] …``
_LOG_LINE = re.compile(r"^\[\d{4}-\d{2}-\d{2}[ T][\d:.]+\]\s*\[[^\]]*\]\s*\[[^\]]*\]")


def strip_ansi(raw: bytes | str) -> str:
    """Decode if needed and remove the colour escapes spdlog emits."""
    text = raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw
    return _ANSI.sub("", text)


def clean_stream(raw: bytes | str, limit: int = 4000) -> str:
    """Decode, de-colour and tail-truncate a captured stream."""
    text = strip_ansi(raw).strip()
    if len(text) > limit:
        text = "...\n" + text[-limit:]
    return text


def strip_log_lines(text: str) -> str:
    """Return ``text`` without the spdlog lines ``rux`` prints on stdout."""
    kept = [line for line in text.splitlines() if not _LOG_LINE.match(line.strip())]
    return "\n".join(kept).strip()


class RuxError(RuntimeError):
    """A ``rux`` invocation could not be completed.

    Carries enough context for an agent to correct itself: the argv that was
    run, the exit status, and the tail of stderr.
    """

    def __init__(
        self,
        message: str,
        *,
        command: Sequence[str] | None = None,
        returncode: int | None = None,
        stderr: str = "",
        stdout: str = "",
    ) -> None:
        super().__init__(message)
        self.command = list(command or [])
        self.returncode = returncode
        self.stderr = stderr
        #: stdout captured despite the failure.  ``rux validate`` exits
        #: non-zero when it finds errors but still prints its JSON report, so
        #: callers that asked for JSON can recover it from here.
        self.stdout = stdout

    def __str__(self) -> str:  # pragma: no cover - formatting only
        parts = [super().__str__()]
        if self.command:
            parts.append("command: " + " ".join(self.command))
        if self.returncode is not None:
            parts.append(f"exit status: {self.returncode}")
        if self.stderr:
            parts.append("stderr:\n" + self.stderr)
        return "\n".join(parts)


class ProjectMissingError(RuxError):
    """The configured ``.rux`` project file does not exist."""


class RuxNotFoundError(RuxError):
    """The ``rux`` executable could not be located."""


class RuxTimeoutError(RuxError):
    """A ``rux`` invocation exceeded its time budget."""


@dataclass
class RuxRunner:
    """Invoke ``rux`` against one project.

    The project path is *server* configuration, never a per-call argument: a
    gateway process is bound to a single ``.rux`` file for its lifetime, which
    keeps the tool schemas free of a path an agent could point anywhere.
    """

    project: Path
    binary: str = "rux"
    timeout: float = DEFAULT_TIMEOUT
    env: dict[str, str] | None = None

    def __post_init__(self) -> None:
        self.project = Path(self.project).expanduser()
        self.timeout = float(self.timeout)

    # -- introspection ---------------------------------------------------

    def resolve_binary(self) -> str:
        """Return an executable path for ``rux`` or raise :class:`RuxNotFoundError`."""
        candidate = Path(self.binary).expanduser()
        if candidate.is_absolute() or os.sep in str(self.binary):
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return str(candidate)
            raise RuxNotFoundError(
                f"rux binary '{self.binary}' is not an executable file"
            )
        found = shutil.which(self.binary)
        if found is None:
            raise RuxNotFoundError(
                f"rux binary '{self.binary}' not found on PATH; pass --rux-bin "
                "or set RUX_BIN to the built executable "
                "(e.g. build/apps/rux/rux)"
            )
        return found

    def check_project(self) -> None:
        """Raise :class:`ProjectMissingError` unless the project file exists."""
        if not self.project.exists():
            raise ProjectMissingError(
                f"project file not found: {self.project}. The gateway is bound "
                "to one .rux project; restart it with --project pointing at an "
                "existing file."
            )
        if self.project.is_dir():
            raise ProjectMissingError(
                f"project path is a directory, not a .rux file: {self.project}"
            )

    def argv(self, args: Sequence[str]) -> list[str]:
        """Build the full argument vector for ``args``."""
        return [self.resolve_binary(), "-p", str(self.project), *args]

    # -- execution -------------------------------------------------------

    def run(
        self,
        args: Sequence[str],
        *,
        timeout: float | None = None,
        check_project: bool = True,
    ) -> str:
        """Run ``rux <args>`` and return stdout as text.

        Raises :class:`ProjectMissingError`, :class:`RuxNotFoundError`,
        :class:`RuxTimeoutError` or :class:`RuxError` — never a bare
        ``CalledProcessError``.
        """
        if check_project:
            self.check_project()
        command = self.argv(args)
        budget = self.timeout if timeout is None else float(timeout)
        environ = dict(os.environ)
        if self.env:
            environ.update(self.env)
        try:
            completed = subprocess.run(  # noqa: S603 - fixed argv, no shell
                command,
                capture_output=True,
                timeout=budget,
                check=False,
                env=environ,
                # `rux del` and `rux edit perturb-poses` ask for confirmation
                # on a terminal. With stdin closed the read fails and the
                # command aborts, which is the safe outcome for a gateway.
                stdin=subprocess.DEVNULL,
            )
        except FileNotFoundError as exc:  # binary vanished between which() and exec
            raise RuxNotFoundError(
                f"rux binary disappeared: {exc}", command=command
            ) from exc
        except subprocess.TimeoutExpired as exc:
            raise RuxTimeoutError(
                f"rux timed out after {budget:g}s",
                command=command,
                stderr=clean_stream(exc.stderr or b""),
            ) from exc

        if completed.returncode != 0:
            raise RuxError(
                f"rux exited with status {completed.returncode}",
                command=command,
                returncode=completed.returncode,
                stderr=clean_stream(completed.stderr) or clean_stream(completed.stdout),
                stdout=completed.stdout.decode("utf-8", errors="replace"),
            )
        return completed.stdout.decode("utf-8", errors="replace")

    def run_json(
        self,
        args: Sequence[str],
        *,
        timeout: float | None = None,
    ) -> Any:
        """Run ``rux <args>`` and parse stdout as JSON.

        ``rux`` logs to stdout, so any spdlog lines that preceded the payload
        are removed before parsing.
        """
        raw = self.run(args, timeout=timeout)
        payload = strip_log_lines(_ANSI.sub("", raw))
        if not payload:
            raise RuxError(
                "rux produced no output where JSON was expected",
                command=self.argv(args),
                stdout=raw,
            )
        try:
            return json.loads(payload)
        except json.JSONDecodeError as exc:
            raise RuxError(
                f"rux output was not valid JSON: {exc}",
                command=self.argv(args),
                stderr=clean_stream(payload, limit=800),
                stdout=raw,
            ) from exc
