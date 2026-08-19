// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

/// CLI options for `rux create materials` — generate one material passport per
/// instance and link it back onto the instance.
struct SubcommandCreateMaterialsOptions {
  /// Name of the instance-label cloud in ProjectDB.
  std::string instances_cloud_name = "instances";

  /// Name of the semantic-label cloud, used to name passports by class.
  std::string semantic_cloud_name = "labels";

  /// Optional project id to associate the created passports with.
  std::string project_id;

  /// Regenerate passports for instances that are already linked.
  bool clear = false;
};

void setup_subcommand_create_materials(CLI::App &parent,
                                       std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_materials(SubcommandCreateMaterialsOptions const &opt,
                                    const RuxOptions &global_opt);
