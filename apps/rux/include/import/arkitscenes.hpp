// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>

#include <memory>

/// Options for the ARKitScenes dataset import subcommand.
struct SubcommandImportArkitscenesOptions {
  fs::path scene_dir; ///< e.g. .../raw/Validation/41069021
};

void setup_subcommand_import_arkitscenes(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_import_arkitscenes(
    SubcommandImportArkitscenesOptions const &opt,
    const RuxOptions &global_opt);
