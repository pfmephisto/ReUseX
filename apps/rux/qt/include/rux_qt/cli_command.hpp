// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// "Kopiér som rux-kommando" (Stream Q, extra b): the `rux` command line that
// does what a pipeline run in the Qt client did, so the GUI teaches the CLI.
//
// The input is a stage's parameters as the job runner reads them (the JSON
// keys of pipeline/stage_parameters.hpp), the output the equivalent
// `rux -p <project> create <stage> …` (or `rux … optimize …`). The key -> flag
// table lives here, next to nothing that could keep it honest — so
// tests/unit/rux_app/test_cli_command_roundtrip.cpp parses every generated
// command back through the REAL CLI11 setup of each subcommand and checks the
// stage receives the same parameters. A renamed flag fails that test.
//
// Qt-free (rux_qt_core): the light test binary covers the formatting.

#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

enum class CliValueKind { number, integer, boolean, string, integer_list };

/// One parameter of a run, its value already in its canonical text: a number
/// with '.' ("0.05"), "true"/"false", a string as is, a list as "1,2,5".
struct CliParam {
  std::string key;
  CliValueKind kind = CliValueKind::number;
  std::string value;
};

struct CliCommand {
  /// The words after `rux`, e.g. {"-p", "x.rux", "create", "planes", …}.
  std::vector<std::string> args;
  /// The whole line, shell-quoted, starting with `rux`.
  std::string text;
  /// Keys the CLI has no flag for: the command runs them at their default.
  std::vector<std::string> unmapped;
  /// False when @p stage has no CLI subcommand at all.
  bool supported = false;
};

/// The command for a run of @p stage (a JobStage name: "clouds", "planes",
/// "rooms", "instances", "mesh", "optimize") with @p params, in order.
/// A boolean whose value is the CLI's own default emits nothing.
CliCommand build_cli_command(std::string_view stage, std::string_view project,
                             const std::vector<CliParam> &params);

/// The flag @p key of @p stage is passed with ("--grid"), or "" if none.
std::string cli_flag_for(std::string_view stage, std::string_view key);

/// POSIX-shell quoting: bare when safe, else single quotes.
std::string shell_quote(std::string_view word);

/// Split a line the way a POSIX shell splits words: whitespace, '…' and "…"
/// quoting, backslash escapes outside single quotes. For tests and for
/// pasting a command back.
std::vector<std::string> split_shell_words(std::string_view line);

/// Wrap a shell line to @p columns with ` \` continuations (so the wrapped
/// text still pastes as one command), breaking only between words — never
/// inside a quoted word or a flag — and keeping a flag with its value.
/// Continuation lines are indented two spaces. An unquoted word longer than
/// a line (a deep path) is cut with a bare backslash-newline, which a shell
/// joins with nothing between; a quoted one stays whole.
std::string wrap_shell_command(std::string_view line, std::size_t columns);

/// Shortest decimal text that reads back as @p value ("0.05", "25", "1e-06").
std::string format_number(double value);
/// The same for a float: the shortest text that reads back as the FLOAT
/// ("0.07", not the 0.07000000029802322 its double widening would print).
std::string format_number(float value);

} // namespace rux::qt
