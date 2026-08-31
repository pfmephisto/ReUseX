// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>

#include <memory>

/// Options for the MuSHRoom dataset import subcommand.
struct SubcommandImportMushroomOptions {
  fs::path capture_dir; ///< e.g. <room>/iphone/long_capture
};

void setup_subcommand_import_mushroom(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_mushroom(SubcommandImportMushroomOptions const &opt,
                                   const RuxOptions &global_opt);
