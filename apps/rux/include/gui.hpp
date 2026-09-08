// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <cstdint>
#include <memory>
#include <string>

/// Options for the `rux gui` subcommand. Defaults mirror
/// rux::gui::ServerOptions rather than redefining them (STANDARDS §4).
struct SubcommandGuiOptions {
  std::string bind_address = "127.0.0.1";
  uint16_t port = 8420;
  unsigned threads = 0;
  fs::path asset_dir;
  bool no_browser = false;
};

void setup_subcommand_gui(CLI::App &app,
                          std::shared_ptr<RuxOptions> global_opt);

int run_subcommand_gui(SubcommandGuiOptions const &opt,
                       const RuxOptions &global_opt);
