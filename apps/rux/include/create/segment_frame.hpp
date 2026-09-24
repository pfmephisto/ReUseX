// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "RuxOptions.hpp"

#include <CLI/CLI.hpp>

#include <filesystem>
#include <string>
#include <vector>

struct SubcommandSegmentFrameOptions {
  std::filesystem::path net_path;
  int frame_id = -1;
  std::vector<std::string> texts;
  /// Each box: "pos:x1,y1,x2,y2" or "neg:x1,y1,x2,y2"
  std::vector<std::string> boxes;
  float confidence = 0.5f;
  bool use_cuda = false;
  bool save = true;
};

void setup_subcommand_create_segment_frame(CLI::App &app,
                                           std::shared_ptr<RuxOptions> opt);

int run_subcommand_create_segment_frame(
    const SubcommandSegmentFrameOptions &opt, const RuxOptions &global_opt);
