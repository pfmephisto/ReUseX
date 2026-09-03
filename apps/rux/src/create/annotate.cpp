// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/annotate.hpp"
#include "validation.hpp"

#include "spdmon.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <reusex/utils/fmt_formatter.hpp>
#include <reusex/vision/BackendFactory.hpp>
#include <reusex/vision/annotate.hpp>

#include <fmt/format.h>

#include <cstdint>
#include <optional>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <filesystem>

namespace fs = std::filesystem;

void setup_subcommand_create_annotate(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt) {

  // Create the option and subcommand objects.
  auto opt = std::make_shared<SubcommandAnnotateOptions>();
  auto *sub =
      app.add_subcommand("annotate", "Run ML inference on sensor frames");

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

PERFORMANCE TUNING:
  For GPU memory-constrained systems:
    --batch-size 8 --workers 2-4 --prefetch 4-8

  For CPU-limited systems:
    --workers 1-2 --prefetch 2-4

  For high-end systems (16+ cores, high VRAM):
    --batch-size 32-64 --workers 8-16 --prefetch 16-48
)");

  sub->add_option("-n, --net", opt->net_path,
                  "Path to the YOLOv8 model file (ONNX or PT format)")
      //->check(CLI::ExistingFile)
      ->default_val(opt->net_path);

  sub->add_flag("-c, --cuda", opt->isCuda, "Use CUDA for YOLOv8 inference")
      ->default_val(opt->isCuda);

  sub->add_option(
         "-b, --batch-size", opt->batch_size,
         "Batch size for inference (recommended: 8-64 based on GPU memory)")
      ->check(CLI::Range(1, 1024))
      ->default_val(opt->batch_size);

  sub->add_option(
         "-w, --workers", opt->num_workers,
         "Number of worker threads for data loading (recommended: 2-4)")
      ->check(CLI::Range(1, 64))
      ->default_val(opt->num_workers);

  sub->add_option("-p, --prefetch", opt->prefetch_batches,
                  "Number of batches to prefetch (recommended: 2-3x workers)")
      ->check(CLI::Range(1, 256))
      ->default_val(opt->prefetch_batches);

  sub->add_flag("--shuffle", opt->shuffle,
                "Shuffle dataset before processing (default: false)")
      ->default_val(opt->shuffle);

  sub->add_option("--seed", opt->seed,
                  "Seed for deterministic shuffle order (default: 42). "
                  "Ignored unless --shuffle is set.")
      ->default_val(opt->seed);

  sub->add_flag("--random-seed", opt->random_seed,
                "Seed the shuffle from entropy (non-deterministic); "
                "overrides --seed")
      ->default_val(opt->random_seed);

  sub->add_flag("--skip-annotated", opt->skip_annotated,
                "Skip frames that already have segmentation labels (resume "
                "interrupted runs)")
      ->default_val(opt->skip_annotated);

  sub->add_flag("--video", opt->video,
                "Use the stateful video-tracker path (SAM 3.1). Processes "
                "frames in temporal order on a single thread (forces "
                "--shuffle off, --workers 1, --batch-size 1). Requires a "
                "SAM 3.1 model directory.")
      ->default_val(opt->video);

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

    // Detect the model type up-front so we can validate/route the video path.
    const auto model_type =
        reusex::vision::BackendFactory::detect_model(opt.net_path);
    const bool is_sam3p1 = (model_type == reusex::vision::Model::sam3p1);

    // A SAM 3.1 directory always routes to the video path (annotate.cpp does the
    // same automatically); --video with a non-SAM-3.1 model is an error.
    bool video = opt.video || is_sam3p1;
    if (opt.video && !is_sam3p1) {
      spdlog::error("--video requires a SAM 3.1 model directory (with "
                    "tracker-memory-encoder + tracker-memory-attention "
                    "engines); '{}' is not one.",
                    opt.net_path.string());
      return RuxError::INVALID_ARGUMENT;
    }
    if (is_sam3p1 && !opt.video) {
      spdlog::info("SAM 3.1 model detected; using the stateful video-tracker "
                   "path (implicit --video).");
    }

    // Video mode forces ordered single-threaded processing. Warn if the user set
    // conflicting dataloader options, then override them.
    size_t batch_size = opt.batch_size;
    size_t num_workers = opt.num_workers;
    bool shuffle = opt.shuffle;
    if (video) {
      if (opt.shuffle)
        spdlog::warn("--shuffle is incompatible with --video (frames must be "
                     "processed in order); disabling shuffle.");
      if (opt.batch_size != 1)
        spdlog::warn("--batch-size {} is ignored in --video mode; forcing "
                     "batch size 1.",
                     opt.batch_size);
      if (opt.num_workers != 1)
        spdlog::warn("--workers {} is ignored in --video mode; forcing a "
                     "single worker.",
                     opt.num_workers);
      shuffle = false;
      batch_size = 1;
      num_workers = 1;
    }

    // Build config from CLI options. --random-seed opts into entropy;
    // otherwise the (default 42) fixed seed keeps shuffled runs reproducible.
    reusex::vision::AnnotationConfig config{
        .use_cuda = opt.isCuda,
        .batch_size = batch_size,
        .shuffle = shuffle,
        .num_workers = num_workers,
        .prefetch_batches = opt.prefetch_batches,
        .skip_annotated = opt.skip_annotated,
        .seed =
            opt.random_seed ? std::nullopt : std::optional<uint32_t>(opt.seed),
        .video = video};

    return reusex::vision::annotate(project_path, opt.net_path, config);

  } catch (const std::exception &e) {
    spdlog::error("Annotation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
