// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "rux/view.hpp"
#include <spdlog/spdlog.h>

void setup_subcommand_view(CLI::App &app) {

  auto opt = std::make_shared<SubcommandViewOptions>();
  auto *sub =
      app.add_subcommand("view", "This tool allows for viewing of the 3D "
                                 "annotated point clouds.");

  sub->callback([opt]() {
    spdlog::trace("calling viewer subcommand");
    return run_subcommand_view(*opt);
  });
}

int run_subcommand_view(SubcommandViewOptions const &opt) { return 0; }
