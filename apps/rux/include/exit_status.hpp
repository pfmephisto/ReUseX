// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <global-params.hpp>

#include <CLI/Error.hpp>

#include <algorithm>

namespace rux {

/// Translate a `run_subcommand_*` return value into a process exit code.
///
/// `RuxError` values are negative (`GENERIC == -1` … `NOT_IMPLEMENTED == -4`)
/// but a process exit status carries only the low 8 bits, so returning them
/// unchanged would surface to a shell as 255/254/253/252. Mirror them onto
/// small positive codes instead and clamp into 1–125, the range a shell
/// reserves for program status (126/127 mean "not executable"/"not found" and
/// 128+n means "killed by signal n").
inline int exit_code_for(int status) {
  if (status == RuxError::SUCCESS)
    return 0;
  const long code = status < 0 ? -static_cast<long>(status) : status;
  return static_cast<int>(std::clamp<long>(code, 1, 125));
}

/// Hand a `run_subcommand_*` status to CLI11 so that it reaches the process
/// exit code.
///
/// CLI11 takes subcommand callbacks as `std::function<void()>`, so a callback
/// that merely `return`s its status has that status silently discarded and
/// `rux` exits 0 even though the stage failed (issue #299). Wrapping every
/// call in `rux::finish()` closes that gap:
///
/// * `CLI::RuntimeError` derives from `CLI::ParseError`, which `main()`
///   already catches and forwards to `CLI::App::exit()`.
/// * `App::exit()` special-cases `RuntimeError` — it returns the exit code
///   *without printing anything*. The subcommand's own `spdlog::error` line
///   therefore remains the only user-visible diagnostic, unchanged.
///
/// A successful status returns normally, so long-running and interactive
/// subcommands (`view`, `gui`) keep their existing lifecycle: the throw can
/// only happen after `run_subcommand_*` has already returned.
inline void finish(int status) {
  if (status != RuxError::SUCCESS)
    throw CLI::RuntimeError(exit_code_for(status));
}

} // namespace rux
