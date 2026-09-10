# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""The command catalogue: what each write mode unlocks, and how argv is built.

This is the security boundary of the write surface — everything an agent can
make the gateway execute has to come through :func:`rux_mcp.commands.build`,
so the gating and the argv rendering are pinned here rather than left to the
server tests.
"""

from __future__ import annotations

import pytest

from rux_mcp.commands import (
    CATALOGUE,
    NEEDS_CONFIRMATION,
    CommandNotAllowed,
    WriteMode,
    available,
    build,
    describe,
    resolve,
)

NONE, STAGES, FULL = WriteMode.none, WriteMode.stages, WriteMode.full


# -- gating ---------------------------------------------------------------


def test_modes_are_ordered():
    assert FULL.allows(STAGES) and FULL.allows(NONE)
    assert STAGES.allows(NONE)
    assert not STAGES.allows(FULL)
    assert not NONE.allows(STAGES)


def test_read_only_mode_reaches_nothing_that_mutates():
    """The central promise: at write-mode none, nothing changes a project."""
    for command in available(NONE):
        assert command.mode is NONE, command.key
        assert not command.mutates_project, command.key


def test_stages_mode_stops_short_of_the_irreversible():
    keys = {command.key for command in available(STAGES)}
    assert "create planes" in keys
    assert "export ply" in keys
    assert "edit downsample" in keys
    for forbidden in ("import rtabmap", "del", "set", "edit perturb-poses"):
        assert forbidden not in keys


def test_full_mode_reaches_everything_catalogued():
    assert {command.key for command in available(FULL)} == set(CATALOGUE)


def test_available_can_omit_the_read_only_entries():
    keys = {c.key for c in available(STAGES, include_read=False)}
    assert "info" not in keys
    assert "create planes" in keys


@pytest.mark.parametrize(
    "key,mode",
    [
        ("create planes", NONE),
        ("import rtabmap", STAGES),
        ("del", STAGES),
    ],
)
def test_resolve_refuses_a_command_above_the_mode(key, mode):
    with pytest.raises(CommandNotAllowed) as excinfo:
        resolve(key, mode)
    message = str(excinfo.value)
    assert key in message
    # The refusal has to say what would make it work, or the agent guesses.
    assert "--write-mode" in message


def test_resolve_refuses_an_uncatalogued_command():
    with pytest.raises(CommandNotAllowed) as excinfo:
        resolve("view", FULL)
    assert "list_commands()" in str(excinfo.value)


def test_resolve_will_not_be_talked_into_a_bare_shell_string():
    for attempt in ("info; rm -rf /", "info && del", "create planes | tee x"):
        with pytest.raises(CommandNotAllowed):
            resolve(attempt, FULL)


def test_resolve_tolerates_sloppy_whitespace():
    assert resolve("  create   planes ", STAGES).key == "create planes"


# -- argv rendering -------------------------------------------------------


def test_options_render_as_a_single_argv_element():
    """`--flag=value`, so a value can never be read as the next flag."""
    invocation = build("create planes", STAGES, options={"grid-size": 0.05})
    assert invocation.args == ["create", "planes", "--grid-size=0.05"]


def test_a_value_that_looks_like_a_flag_stays_a_value():
    invocation = build("export ply", STAGES, options={"output": "--del"})
    assert invocation.args == ["export", "ply", "--output=--del"]


def test_booleans_become_presence_and_absence():
    on = build("create annotate", STAGES, options={"cuda": True})
    off = build("create annotate", STAGES, options={"cuda": False})
    assert on.args == ["create", "annotate", "--cuda"]
    assert off.args == ["create", "annotate"]


def test_null_is_a_bare_flag():
    assert build("create rooms", STAGES, options={"verbose": None}).args == [
        "create",
        "rooms",
        "--verbose",
    ]


def test_a_list_repeats_the_flag():
    invocation = build("create mesh", STAGES, options={"cloud": ["a", "b"]})
    assert invocation.args == ["create", "mesh", "--cloud=a", "--cloud=b"]


def test_leading_dashes_on_an_option_name_are_forgiven():
    assert build("create rooms", STAGES, options={"--seed": 3}).args == [
        "create",
        "rooms",
        "--seed=3",
    ]


@pytest.mark.parametrize("name", ["", "-", "1st", "grid size", "a;b", "x\ny"])
def test_an_option_name_that_is_not_a_flag_is_refused(name):
    with pytest.raises(CommandNotAllowed) as excinfo:
        build("create planes", STAGES, options={name: 1})
    assert "option name" in str(excinfo.value)


def test_positionals_are_appended_in_order():
    invocation = build(
        "import rtabmap", FULL, arguments=["/data/scan.db"], options={"limit": 10}
    )
    assert invocation.args == ["import", "rtabmap", "/data/scan.db", "--limit=10"]


def test_a_positional_starting_with_a_dash_is_refused():
    with pytest.raises(CommandNotAllowed) as excinfo:
        build("import ply", FULL, arguments=["--force"])
    assert "would be read as a flag" in str(excinfo.value)


def test_an_empty_positional_is_refused():
    with pytest.raises(CommandNotAllowed):
        build("import ply", FULL, arguments=[""])


@pytest.mark.parametrize("payload", ["a\nb", "a\x00b", "a\rb"])
def test_control_characters_are_refused_in_positionals(payload):
    with pytest.raises(CommandNotAllowed) as excinfo:
        build("import ply", FULL, arguments=[payload])
    assert "control characters" in str(excinfo.value)


@pytest.mark.parametrize("payload", ["a\nb", "a\x00b"])
def test_control_characters_are_refused_in_option_values(payload):
    with pytest.raises(CommandNotAllowed) as excinfo:
        build("create planes", STAGES, options={"cloud": payload})
    assert "control character" in str(excinfo.value)


def test_shell_metacharacters_in_a_value_are_left_alone():
    """argv, never a shell — so these are data, not syntax."""
    invocation = build("export ply", STAGES, options={"output": "a; rm -rf /"})
    assert invocation.args[-1] == "--output=a; rm -rf /"


# -- timeouts -------------------------------------------------------------


def test_the_default_timeout_comes_from_the_catalogue():
    assert build("create rooms", STAGES).timeout == CATALOGUE[
        "create rooms"
    ].default_timeout
    assert build("create mesh", STAGES).timeout > build("create rooms", STAGES).timeout


def test_gsplat_gets_more_room_than_a_mip_solve():
    """Training runs for hours; a solver budget would kill it mid-run."""
    assert (
        CATALOGUE["create gsplat"].default_timeout
        > CATALOGUE["create mesh"].default_timeout
    )


def test_an_explicit_timeout_wins():
    assert build("create mesh", STAGES, timeout=30).timeout == 30.0


@pytest.mark.parametrize("bad", [0, -1])
def test_a_nonpositive_timeout_is_refused(bad):
    with pytest.raises(CommandNotAllowed):
        build("create mesh", STAGES, timeout=bad)


# -- catalogue integrity --------------------------------------------------


def test_every_catalogued_key_matches_its_argv():
    for key, command in CATALOGUE.items():
        assert command.key == key
        assert command.argv == tuple(key.split())


def test_the_commands_that_prompt_are_catalogued_and_irreversible():
    for key in NEEDS_CONFIRMATION:
        assert CATALOGUE[key].mode is FULL, key


def test_exports_are_marked_as_leaving_the_project_alone():
    for key, command in CATALOGUE.items():
        if key.startswith("export "):
            assert not command.mutates_project, key


def test_describe_marks_what_this_mode_cannot_run():
    text = describe(STAGES)
    assert "write mode: stages" in text
    assert "(BLOCKED)" in text
    assert "create planes" in text
    # The confirmation rule has to be discoverable without reading the source.
    assert "'yes': true" in text


def test_describe_at_full_blocks_nothing():
    assert "(BLOCKED)" not in describe(FULL)
