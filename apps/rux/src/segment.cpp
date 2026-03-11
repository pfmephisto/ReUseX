// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segment.hpp"
#include "segment/planes.hpp"
#include "segment/rooms.hpp"
#include "spdmon/spdmon.hpp"

#include <fmt/format.h>
#include <fmt/std.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <pcl/common/colors.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <ranges>
namespace fs = std::filesystem;
// using namespace ReUseX;

void setup_subcommand_segment(CLI::App &app) {

  auto opt = std::make_shared<SubcommandSegmentOptions>();

  auto *sub = app.add_subcommand(
      "segment", "Segment point clouds using various segmentation methods.");

  setup_subcommand_segment_planes(*sub);
  setup_subcommand_segment_rooms(*sub);

  sub->callback([opt]() {
    spdlog::trace("calling segment subcommand");
    return run_subcommand_segment(*opt);
  });
}

int run_subcommand_segment(SubcommandSegmentOptions const &opt) {
  spdlog::trace("running segment subcommand");

  return RuxError::SUCCESS;
}
