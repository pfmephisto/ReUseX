// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

struct SubcommandAnnotateOptions {

  fs::path net_path = fs::current_path() / "yolov8x-seg.torchscript";

  bool isCuda{false};

  // Dataloader configuration
  size_t batch_size = 16;        // Batch size for inference (recommended: 8-64)
  bool shuffle = false;          // Shuffle dataset (rarely needed for inference)
  size_t num_workers = 4;        // Number of worker threads (recommended: 2-4)
  size_t prefetch_batches = 8;   // Batches to prefetch (recommended: 2-3x workers)
};

// Function declarations.
void setup_subcommand_create_annotate(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_annotate(SubcommandAnnotateOptions const &opt, const RuxOptions &global_opt);
