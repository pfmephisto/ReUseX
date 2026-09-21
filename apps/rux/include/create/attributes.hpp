// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <cstddef>
#include <memory>
#include <string>

// CLI options for `rux create attributes`. Defaults MIRROR
// reusex::vision::DescribeConfig; they are not a second source of truth
// (STANDARDS §4). The struct is initialized from the library defaults in the
// .cpp so a drift would show up there, not here.
struct SubcommandAttributesOptions {
  std::string api_url;
  std::string model;
  std::string api_key; // empty => fall back to $REUSEX_VLM_API_KEY
  std::string instances_cloud;
  std::string point_cloud;
  std::string prompt;
  bool skip_existing = false;
  int crop_padding = 8;
  int min_view_points = 20;
  int connect_timeout_s = 10;
  int total_timeout_s = 120;
};

void setup_subcommand_create_attributes(CLI::App &app,
                                        std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_attributes(SubcommandAttributesOptions const &opt,
                              const RuxOptions &global_opt);
