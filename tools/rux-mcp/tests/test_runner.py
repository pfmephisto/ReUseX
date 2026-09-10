# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""The three failure modes every tool must survive, plus the happy path."""

from __future__ import annotations

from pathlib import Path

import pytest

from rux_mcp.runner import (
    ProjectMissingError,
    RuxError,
    RuxNotFoundError,
    RuxRunner,
    RuxTimeoutError,
)


def test_run_json_parses_stdout(runner, fake_rux):
    fake_rux.reply('{"schema_version": 11}')
    assert runner.run_json(["info", "--json"]) == {"schema_version": 11}


def test_project_path_is_passed_before_the_subcommand(runner, fake_rux, project):
    fake_rux.reply("{}")
    runner.run_json(["info", "--json"])
    assert fake_rux.calls == [f"-p {project} info --json"]


def test_missing_project_raises_before_spawning(tmp_path, fake_rux):
    runner = RuxRunner(
        project=tmp_path / "absent.rux", binary=str(fake_rux.path), timeout=5
    )
    with pytest.raises(ProjectMissingError) as excinfo:
        runner.run(["info", "--json"])
    assert "absent.rux" in str(excinfo.value)
    assert fake_rux.calls == []


def test_project_that_is_a_directory_is_rejected(tmp_path, fake_rux):
    runner = RuxRunner(project=tmp_path, binary=str(fake_rux.path))
    with pytest.raises(ProjectMissingError):
        runner.run(["info"])


def test_missing_binary_is_its_own_error(project):
    runner = RuxRunner(project=project, binary="rux-that-does-not-exist")
    with pytest.raises(RuxNotFoundError) as excinfo:
        runner.run(["info"])
    assert "RUX_BIN" in str(excinfo.value)


def test_nonzero_exit_surfaces_stderr(runner, fake_rux):
    fake_rux.reply("", stderr="Path error: Invalid collection name: nope", status=1)
    with pytest.raises(RuxError) as excinfo:
        runner.run(["get", "nope"])
    error = excinfo.value
    assert error.returncode == 1
    assert "Invalid collection name" in error.stderr


def test_ansi_escapes_are_stripped_from_stderr(runner, fake_rux):
    fake_rux.reply("", stderr="\x1b[31mboom\x1b[0m", status=2)
    with pytest.raises(RuxError) as excinfo:
        runner.run(["info"])
    assert excinfo.value.stderr == "boom"


def test_stdout_is_kept_on_failure_for_validate(runner, fake_rux):
    fake_rux.reply('{"ok": false}', stderr="1 error", status=1)
    with pytest.raises(RuxError) as excinfo:
        runner.run(["validate", "--json"])
    assert excinfo.value.stdout.strip() == '{"ok": false}'


def test_timeout_becomes_a_rux_timeout_error(project, tmp_path):
    slow = tmp_path / "slow-rux"
    slow.write_text("#!/bin/sh\nsleep 5\n")
    slow.chmod(0o755)
    runner = RuxRunner(project=project, binary=str(slow), timeout=0.3)
    with pytest.raises(RuxTimeoutError) as excinfo:
        runner.run(["info"])
    assert "timed out" in str(excinfo.value)


#: `rux` installs spdlog on stdout, so warnings arrive interleaved with the
#: payload rather than on stderr.  Every JSON-parsing tool depends on this.
SPDLOG_WARNING = (
    "[2026-09-10 11:02:52.886] [rux] [warning] Project schema is v11 but "
    "this build expects v12."
)


def test_spdlog_lines_on_stdout_are_stripped_before_parsing(runner, fake_rux):
    fake_rux.reply(SPDLOG_WARNING + '\n{"schema_version": 11}\n')
    assert runner.run_json(["info", "--json"]) == {"schema_version": 11}


def test_several_log_lines_are_stripped(runner, fake_rux):
    fake_rux.reply(
        "\n".join(
            [
                "[2026-09-10 11:02:52.886] [rux] [warning] one",
                "[2026-09-10 11:02:52.887] [rux] [ error ] two",
                "[1, 2, 3]",
            ]
        )
    )
    assert runner.run_json(["get", "frames"]) == [1, 2, 3]


def test_a_scalar_payload_survives_log_stripping(runner, fake_rux):
    fake_rux.reply(SPDLOG_WARNING + "\n255744\n")
    assert runner.run_json(["get", "clouds.cloud.point_count"]) == 255744


def test_log_stripping_does_not_eat_json_that_merely_looks_bracketed(runner, fake_rux):
    fake_rux.reply('["[2026-09-10] not a log line"]')
    assert runner.run_json(["get", "clouds"]) == ["[2026-09-10] not a log line"]


def test_empty_output_where_json_expected(runner, fake_rux):
    fake_rux.reply("")
    with pytest.raises(RuxError) as excinfo:
        runner.run_json(["info", "--json"])
    assert "no output" in str(excinfo.value)


def test_non_json_output_is_reported_as_such(runner, fake_rux):
    fake_rux.reply("Pipeline Execution History (252 entries)")
    with pytest.raises(RuxError) as excinfo:
        runner.run_json(["log"])
    assert "not valid JSON" in str(excinfo.value)


def test_stdout_is_used_for_diagnostics_when_stderr_is_empty(runner, fake_rux):
    """rux logs to stdout, so a failure often leaves stderr blank."""
    fake_rux.reply("Path error: Invalid collection name: nope", status=1)
    with pytest.raises(RuxError) as excinfo:
        runner.run(["get", "nope"])
    assert "Invalid collection name" in str(excinfo.value)


def test_binary_is_never_run_through_a_shell(runner, fake_rux):
    """Arguments are passed as argv, so shell metacharacters stay literal."""
    fake_rux.reply("{}")
    runner.run_json(["get", "clouds.a;rm -rf /"])
    assert Path("/").exists()
    assert fake_rux.calls[-1].endswith("get clouds.a;rm -rf /")
