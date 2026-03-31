// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/annotate.hpp"

#include "spdmon.hpp"
#include <reusex/utils/fmt_formatter.hpp>
#include <reusex/vision/annotate.hpp>

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <filesystem>

namespace fs = std::filesystem;

void setup_subcommand_create_annotate(CLI::App &app) {

  // Create the option and subcommand objects.
  auto opt = std::make_shared<SubcommandAnnotateOptions>();
  auto *sub = app.add_subcommand(
      "annotate", "Run semantic segmentation inference on sensor frames using ML models (YOLO/SAM2)");

  sub->add_option("database", opt->database_path_in,
                  "Path to the ReUseX project database file (.rux).")
      ->required()
      ->check(CLI::ExistingFile);

  sub->add_option("-n, --net", opt->net_path,
                  "Path to the YOLOv8 model file (ONNX or PT format)")
      //->check(CLI::ExistingFile)
      ->default_val(opt->net_path);

  sub->add_flag("-c, --cuda", opt->isCuda, "Use CUDA for YOLOv8 inference")
      ->default_val(opt->isCuda);

  sub->callback([opt]() {
    spdlog::trace("calling run_subcommand_annotate");
    return run_subcommand_annotate(*opt);
  });
}

int run_subcommand_annotate(SubcommandAnnotateOptions const &opt) {
  return ReUseX::vision::annotate(opt.database_path_in, opt.net_path, opt.isCuda);
}
