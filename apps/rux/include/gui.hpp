// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"
#include "gui/Server.hpp"

#include <CLI/CLI.hpp>
#include <memory>

/// Options for the `rux gui` subcommand.
///
/// Holds a rux::gui::ServerOptions rather than restating its fields, so the CLI
/// cannot drift from the library defaults (STANDARDS §4). Only genuinely
/// CLI-shaped state lives alongside it.
struct SubcommandGuiOptions {
  rux::gui::ServerOptions server;
  /// The flag is negative (`--no-browser`) while the option it controls is
  /// positive (`open_browser`), so it cannot simply bind to that field.
  bool no_browser = false;
};

void setup_subcommand_gui(CLI::App &app,
                          std::shared_ptr<RuxOptions> global_opt);

int run_subcommand_gui(SubcommandGuiOptions const &opt,
                       const RuxOptions &global_opt);
