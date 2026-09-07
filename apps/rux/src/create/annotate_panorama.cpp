// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/annotate_panorama.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/vision/model_factory.hpp>
#include <reusex/vision/segment_panorama.hpp>

#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <unordered_map>

namespace fs = std::filesystem;

namespace {

// Colorize a CV_32S label map (-1 background): each distinct label gets a
// distinct hue. Returns a BGR image the same size as the labels.
cv::Mat colorize_labels(const cv::Mat &labels) {
  cv::Mat hsv(labels.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  std::unordered_map<int, int> hue; // label -> hue
  int next = 0;
  for (int y = 0; y < labels.rows; ++y) {
    const int *lp = labels.ptr<int>(y);
    auto *hp = hsv.ptr<cv::Vec3b>(y);
    for (int x = 0; x < labels.cols; ++x) {
      if (lp[x] < 0)
        continue;
      auto it = hue.find(lp[x]);
      if (it == hue.end())
        it = hue.emplace(lp[x], (next++ * 47) % 180).first;
      hp[x] = cv::Vec3b(static_cast<uchar>(it->second), 220, 230);
    }
  }
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}

// Dump [ equirect | colorized overlay ] for one panorama, downscaled for size.
void dump_segmentation_figure(const cv::Mat &equirect, const cv::Mat &labels,
                              const std::string &dir,
                              const std::string &filename) {
  cv::Mat color = colorize_labels(labels);
  cv::Mat overlay;
  cv::addWeighted(equirect, 0.5, color, 0.5, 0.0, overlay);
  // paint color only where a label exists (background keeps the photo)
  cv::Mat mask = labels >= 0; // CV_8U
  cv::Mat blended = equirect.clone();
  overlay.copyTo(blended, mask);

  cv::Mat eq = equirect;
  const int target_w = 2048;
  if (eq.cols > target_w) {
    const double s = static_cast<double>(target_w) / eq.cols;
    cv::resize(eq, eq, cv::Size(), s, s);
    cv::resize(blended, blended, eq.size());
  }
  cv::Mat side;
  cv::vconcat(std::vector<cv::Mat>{eq, blended}, side);

  fs::create_directories(dir);
  const std::string stem =
      (fs::path(dir) / filename).replace_extension().string();
  cv::imwrite(stem + "_segmentation.jpg", side);
}

} // namespace

void setup_subcommand_create_annotate_panorama(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {

  auto opt = std::make_shared<SubcommandAnnotatePanoramaOptions>();
  auto *sub = app.add_subcommand(
      "annotate-360", "Segment 360 panoramas with SAM3 (perspective-tiled)");

  sub->footer(R"(
DESCRIPTION:
  Semantically segments 360 equirectangular panoramas using the SAM3 model.
  Each panorama is sliced into overlapping perspective (pinhole) tiles that
  the segmenter consumes at its native resolution; the per-tile label maps are
  stitched back into a single equirectangular label image and stored in the
  project database.

EXAMPLES:
  rux create annotate-360 --net /models/sam3/          # Default 8 tiles
  rux create annotate-360 --net /models/sam3/ --n-yaw 12 --fov 100
  rux -p office.rux create annotate-360 --net /models/sam3/ --skip-annotated

WORKFLOW:
  1. rux import rtabmap scan.db          # Import SLAM data (with timestamps)
  2. rux import 360 /path/to/360-images/ # Import panoramas
  3. rux create annotate-360 --net sam3/ # Segment panoramas with SAM3
  4. rux get panoramas                   # List imported panoramas

NOTES:
  - Requires panoramic images (run 'rux import 360' first)
  - --net must point to a SAM3 TensorRT engine directory
  - Output stored as a CV_32S equirect label map per panorama (-1 background)
  - Class ids are stable across tiles (one model instance per panorama batch)
)");

  sub->add_option("-n, --net", opt->net_path,
                  "Path to the SAM3 TensorRT engine directory")
      ->required();

  sub->add_option("--n-yaw", opt->n_yaw,
                  "Number of equator tiles around the sphere (default: 8)")
      ->check(CLI::Range(1, 64))
      ->default_val(opt->n_yaw);

  sub->add_option("--fov", opt->fov_deg,
                  "Per-tile horizontal field of view in degrees (default: 90)")
      ->check(CLI::Range(10.0, 179.0))
      ->default_val(opt->fov_deg);

  sub->add_option("--tile", opt->tile,
                  "Tile size in pixels; matches SAM3 native input "
                  "(default: 1008)")
      ->check(CLI::Range(64, 4096))
      ->default_val(opt->tile);

  sub->add_option("--confidence", opt->confidence,
                  "Detection confidence threshold (default: 0.5)")
      ->check(CLI::Range(0.0, 1.0))
      ->default_val(opt->confidence);

  sub->add_flag("--skip-annotated", opt->skip_annotated,
                "Skip panoramas that already have segmentation labels "
                "(resume interrupted runs)")
      ->default_val(opt->skip_annotated);

  sub->add_option("--figures", opt->figures_dir,
                  "Dump colorized segmentation figures to this directory");
  sub->add_option("--figures-limit", opt->figures_limit,
                  "Cap number of panoramas figured")
      ->default_val(opt->figures_limit);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_create_annotate_panorama");
    return run_subcommand_create_annotate_panorama(*opt, *global_opt);
  });
}

int run_subcommand_create_annotate_panorama(
    SubcommandAnnotatePanoramaOptions const &opt,
    const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    reusex::ProjectDB db(project_path);

    auto panoramas = db.list_panoramic_images();
    if (panoramas.empty()) {
      spdlog::warn(
          "No panoramic images in project. Run 'rux import 360' first.");
      return RuxError::SUCCESS;
    }
    spdlog::info("Segmenting {} panoramas with SAM3: {}", panoramas.size(),
                 opt.net_path.string());

    // Create the SAM3 model inside the vision library (where the backend
    // compile-defines are set); calling BackendFactory from the app layer would
    // report every backend as "not compiled".
    auto model =
        reusex::vision::create_model_from_path(opt.net_path, /*use_cuda=*/true);

    reusex::vision::SegmentPanoramaOptions seg_opts;
    seg_opts.n_yaw = opt.n_yaw;
    seg_opts.fov_deg = opt.fov_deg;
    seg_opts.tile = opt.tile;
    seg_opts.confidence = opt.confidence;

    int log_id = db.log_pipeline_start(
        "annotate_360",
        fmt::format(R"({{"panorama_count":{},"n_yaw":{},"fov":{},"tile":{}}})",
                    panoramas.size(), opt.n_yaw, opt.fov_deg, opt.tile));

    int processed = 0;
    int skipped = 0;
    int failed = 0;

    try {
      for (const auto &pano : panoramas) {
        if (opt.skip_annotated && db.has_panorama_segmentation(pano.id)) {
          spdlog::debug("Skipping already segmented panorama {} ({})", pano.id,
                        pano.filename);
          ++skipped;
          continue;
        }

        cv::Mat equirect = db.panoramic_image(pano.id);
        if (equirect.empty()) {
          spdlog::warn("Panorama {} ({}) decoded empty; skipping", pano.id,
                       pano.filename);
          ++failed;
          continue;
        }

        cv::Mat labels =
            reusex::vision::segment_panorama(*model, equirect, seg_opts);
        db.save_panorama_segmentation(pano.id, labels);
        spdlog::info("Segmented panorama {} ({}) [{}x{}]", pano.id,
                     pano.filename, equirect.cols, equirect.rows);

        if (!opt.figures_dir.empty() && processed < opt.figures_limit) {
          try {
            dump_segmentation_figure(equirect, labels, opt.figures_dir,
                                     pano.filename);
          } catch (const std::exception &e) {
            spdlog::warn("{}: figure dump failed: {}", pano.filename, e.what());
          }
        }
        ++processed;
      }

      db.log_pipeline_end(log_id, true);
    } catch (...) {
      db.log_pipeline_end(log_id, false, "annotate_360 failed");
      throw;
    }

    spdlog::info("Panorama segmentation complete: {} processed, {} skipped, "
                 "{} failed",
                 processed, skipped, failed);

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Panorama annotation failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
