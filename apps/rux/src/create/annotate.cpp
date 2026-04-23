// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/annotate.hpp"
#include "validation.hpp"

#include "spdmon.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/utils/fmt_formatter.hpp>
#include <reusex/vision/annotate.hpp>

#include <fmt/format.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <filesystem>

namespace fs = std::filesystem;

void setup_subcommand_create_annotate(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt) {

  // Create the option and subcommand objects.
  auto opt = std::make_shared<SubcommandAnnotateOptions>();
  auto *sub = app.add_subcommand("annotate",
                                 "Run ML inference on sensor frames");

  sub->footer(R"(
DESCRIPTION:
  Performs semantic segmentation on RGB images from sensor frames using
  deep learning models (YOLO, SAM2). Detects and segments objects, walls,
  floors, ceilings, and other architectural elements. Results are saved
  as per-pixel segmentation labels aligned with sensor frames.

EXAMPLES:
  rux create annotate --net yolo11l.pt        # YOLO PyTorch model
  rux create annotate --net model.engine      # TensorRT optimized
  rux create annotate --net model.pt --cuda   # Force CUDA backend
  rux -p project.rux create annotate          # Custom project path

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor frames with RGB
  2. rux create annotate --net model   # Run ML inference
  3. rux create clouds                 # Reconstruct with labels
  4. rux view                          # Visualize segmented cloud

NOTES:
  - Requires sensor frames with color images (run 'rux import rtabmap' first)
  - Supported formats: .pt (PyTorch), .engine (TensorRT), .onnx (ONNX)
  - Backend auto-detected from file extension
  - CUDA flag forces GPU acceleration (enabled by default for .engine)
  - Output saved as segmentation images per sensor frame in project DB
)");

  sub->add_option("-n, --net", opt->net_path,
                  "Path to the YOLOv8 model file (ONNX or PT format)")
      //->check(CLI::ExistingFile)
      ->default_val(opt->net_path);

  sub->add_flag("-c, --cuda", opt->isCuda, "Use CUDA for YOLOv8 inference")
      ->default_val(opt->isCuda);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_annotate");
    return run_subcommand_annotate(*opt, *global_opt);
  });
}

int run_subcommand_annotate(SubcommandAnnotateOptions const &opt,
                            const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    reusex::ProjectDB db(project_path);

    // Pre-flight validation: check for sensor frames
    auto validation = rux::validation::validate_annotate_prerequisites(db);
    if (!validation) {
      spdlog::error("{}", validation.error_message);
      spdlog::info("Resolution: {}", validation.resolution_hint);
      return RuxError::INVALID_ARGUMENT;
    }

    return reusex::vision::annotate(project_path, opt.net_path,
                                    opt.isCuda);

  } catch (const std::exception &e) {
    spdlog::error("Annotation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
