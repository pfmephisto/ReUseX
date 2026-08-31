// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "../global-params.hpp"

#include <CLI/CLI.hpp>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

struct SubcommandAnnotateOptions {

  fs::path net_path = fs::current_path() / "yolov8x-seg.torchscript";

  bool isCuda{false};

  // Dataloader configuration
  size_t batch_size = 16; // Batch size for inference (recommended: 8-64)
  bool shuffle = false;   // Shuffle dataset (rarely needed for inference)
  size_t num_workers = 4; // Number of worker threads (recommended: 2-4)
  size_t prefetch_batches =
      8; // Batches to prefetch (recommended: 2-3x workers)
  bool skip_annotated = false; // Skip already-annotated frames (resume mode)

  // Shuffle RNG seed. Fixed by default so shuffled runs are reproducible
  // (docs/STANDARDS.md §6). --random-seed opts into entropy instead.
  uint32_t seed = 42;
  bool random_seed = false; // Seed from std::random_device (non-deterministic)
};

// Function declarations.
void setup_subcommand_create_annotate(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_annotate(SubcommandAnnotateOptions const &opt,
                            const RuxOptions &global_opt);
