# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Argument parsing — chiefly that the write surface is off unless asked for."""

from __future__ import annotations

import pytest

from rux_mcp.cli import WRITE_MODE_HELP, build_parser, main
from rux_mcp.commands import WriteMode


def parse(argv, monkeypatch, **env):
    for key in ("RUX_MCP_WRITE_MODE", "RUX_MCP_JOB_LOG_DIR", "RUX_PROJECT", "RUX_BIN"):
        monkeypatch.delenv(key, raising=False)
    for key, value in env.items():
        monkeypatch.setenv(key, value)
    return build_parser().parse_args(argv)


def test_write_mode_defaults_to_none(monkeypatch):
    assert parse([], monkeypatch).write_mode == "none"


def test_write_mode_comes_from_the_environment(monkeypatch):
    assert parse([], monkeypatch, RUX_MCP_WRITE_MODE="stages").write_mode == "stages"


def test_the_flag_beats_the_environment(monkeypatch):
    args = parse(["--write-mode", "none"], monkeypatch, RUX_MCP_WRITE_MODE="full")
    assert args.write_mode == "none"


def test_an_unknown_write_mode_on_the_flag_is_rejected(monkeypatch):
    with pytest.raises(SystemExit):
        parse(["--write-mode", "everything"], monkeypatch)


def test_an_unknown_write_mode_in_the_environment_is_rejected(monkeypatch, capsys):
    """argparse validates choices, but never the default it was handed."""
    monkeypatch.setenv("RUX_MCP_WRITE_MODE", "everything")
    with pytest.raises(SystemExit):
        main([])
    assert "invalid write mode" in capsys.readouterr().err


def test_every_mode_is_explained_in_the_help(monkeypatch):
    help_text = build_parser().format_help()
    for mode in WriteMode:
        assert mode.value in help_text
    assert set(WRITE_MODE_HELP) == set(WriteMode)


def test_job_log_dir_defaults_to_unset(monkeypatch):
    assert parse([], monkeypatch).job_log_dir is None
    assert parse([], monkeypatch, RUX_MCP_JOB_LOG_DIR="/tmp/x").job_log_dir == "/tmp/x"


def test_a_missing_binary_is_fatal_before_anything_is_served(monkeypatch, capsys):
    monkeypatch.delenv("RUX_MCP_WRITE_MODE", raising=False)
    assert main(["--rux-bin", "rux-that-does-not-exist"]) == 2
    assert "not found on PATH" in capsys.readouterr().err
