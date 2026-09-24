// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/panorama.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/exif.hpp>
#include <reusex/io/insta360_x4.hpp>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace {

/// Collect all JPEG and .insp panorama files from a list of paths.
std::vector<fs::path>
collect_panorama_files(const std::vector<fs::path> &input_paths) {
  std::vector<fs::path> files;

  auto accept = [](const fs::path &p) {
    auto ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == ".jpg" || ext == ".jpeg" || ext == ".insp";
  };

  for (const auto &path : input_paths) {
    if (fs::is_regular_file(path)) {
      if (accept(path)) {
        files.push_back(path);
      } else {
        spdlog::warn(
            "Skipping unsupported file (expected .jpg/.jpeg/.insp): {}",
            path.string());
      }
    } else if (fs::is_directory(path)) {
      for (const auto &entry : fs::directory_iterator(path)) {
        if (entry.is_regular_file() && accept(entry.path()))
          files.push_back(entry.path());
      }
    } else {
      spdlog::warn("Path does not exist or is not a file/directory: {}",
                   path.string());
    }
  }

  std::sort(files.begin(), files.end());
  return files;
}

/// Read an entire file into a byte vector.
std::vector<uint8_t> read_file_bytes(const fs::path &path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file)
    throw std::runtime_error("Cannot open file: " + path.string());

  auto size = file.tellg();
  file.seekg(0);

  std::vector<uint8_t> data(static_cast<size_t>(size));
  file.read(reinterpret_cast<char *>(data.data()), size);
  return data;
}

} // anonymous namespace

void setup_subcommand_import_panorama(CLI::App &app,
                                      std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandImportPanoramaOptions>();
  auto *sub = app.add_subcommand("360", "Import 360 panoramic images");

  sub->footer(R"(
DESCRIPTION:
  Imports 360 panoramic images into a ReUseX project database.
  Accepts pre-stitched equirect JPEGs and Insta360 X4 dual-fisheye .insp
  files (stitched automatically using the static X4 calibration).
  Each image is matched to the nearest sensor frame by EXIF timestamp.

EXAMPLES:
  rux import 360 /path/to/photos/        # Import all JPEGs/.insp from dir
  rux import 360 img.insp img2.jpg       # Mix of .insp and .jpg
  rux -p office.rux import 360 ./360/    # Custom project path
  rux import 360 --no-stitch ./360/      # Skip .insp stitching

WORKFLOW:
  1. rux import rtabmap scan.db           # Import SLAM data (with timestamps)
  2. rux import 360 /path/to/360-images/  # Match panoramas to sensor frames
  3. rux get panoramas                    # List imported panoramas

NOTES:
  - Supports .jpg/.jpeg (equirect guard: width==2*height) and .insp (Insta360)
  - .insp files are stitched to equirect before storage (no original preserved)
  - Requires sensor frames with timestamps (import RTABMap data first)
  - Reads EXIF DateTimeOriginal to determine capture time
  - Use --no-stitch to import equirects only (skip .insp files)
)");

  sub->add_option("paths", opt->input_paths,
                  "JPEG, .insp, or directories containing panoramic images")
      ->required()
      ->check(CLI::ExistingPath);

  sub->add_flag("--no-stitch", opt->no_stitch,
                "Skip .insp dual-fisheye stitching; import only pre-stitched "
                "equirect JPEGs");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_panorama");
    rux::finish(run_subcommand_import_panorama(*opt, *global_opt));
  });
}

int run_subcommand_import_panorama(SubcommandImportPanoramaOptions const &opt,
                                   const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing 360 panoramic images to project: {}",
               project_path.string());

  reusex::ProjectDB db(project_path);

  // Collect panorama files (.jpg/.jpeg equirects and .insp dual-fisheye)
  auto files = collect_panorama_files(opt.input_paths);
  if (files.empty()) {
    spdlog::warn(
        "No panorama files (.jpg/.jpeg/.insp) found in the specified paths");
    return RuxError::SUCCESS;
  }
  spdlog::info("Found {} panorama file(s) to import", files.size());

  // Check that sensor frames exist and have timestamps
  const auto frame_ids = db.sensor_frame_ids();
  bool has_timestamps = false;
  for (const auto &fid : frame_ids) {
    if (db.sensor_frame_timestamp(fid) >= 0.0) {
      has_timestamps = true;
      break;
    }
  }

  if (frame_ids.empty()) {
    spdlog::warn("No sensor frames in project. Panoramas will be imported "
                 "without spatial matching.");
  } else if (!has_timestamps) {
    spdlog::warn("Sensor frames have no timestamps. Re-import RTABMap data "
                 "to enable timestamp matching. Panoramas will be imported "
                 "without spatial matching.");
  }

  int log_id = db.log_pipeline_start(
      "import_360", fmt::format(R"({{"file_count":{}}})", files.size()));

  int imported = 0;
  int matched = 0;

  auto match_frame = [&](double timestamp, const std::string &label) -> int {
    if (timestamp < 0.0 || !has_timestamps) {
      spdlog::info("Imported {} (no EXIF timestamp)", label);
      return -1;
    }
    const int nid = db.nearest_sensor_frame_by_timestamp(timestamp);
    if (nid >= 0) {
      const double dt = std::abs(timestamp - db.sensor_frame_timestamp(nid));
      spdlog::info("Imported {} -> node {} (dt={:.1f}s)", label, nid, dt);
      ++matched;
    } else {
      spdlog::info("Imported {} (no matching sensor frame)", label);
    }
    return nid;
  };

  try {
    for (const auto &file : files) {
      if (reusex::io::is_insta360_dual_fisheye(file)) {
        if (opt.no_stitch) {
          spdlog::debug("Skipping .insp (--no-stitch): {}",
                        file.filename().string());
          continue;
        }

        // Stitch .insp → equirect, store as <stem>.jpg
        const std::string filename = file.stem().string() + ".jpg";
        if (db.has_panoramic_image(filename)) {
          spdlog::debug("Skipping already imported: {}", filename);
          continue;
        }

        const double timestamp = reusex::io::read_exif_timestamp(file);
        const int node_id = match_frame(timestamp, file.filename().string());

        const cv::Mat dual = cv::imread(file.string());
        if (dual.empty()) {
          spdlog::warn("Cannot read .insp file: {}", file.filename().string());
          continue;
        }

        cv::Mat equirect_img;
        try {
          equirect_img = reusex::io::stitch_insta360_x4(dual);
        } catch (const std::exception &ex) {
          spdlog::warn("Stitch failed for {}: {}", file.filename().string(),
                       ex.what());
          continue;
        }

        std::vector<uint8_t> jpeg_data;
        cv::imencode(".jpg", equirect_img, jpeg_data,
                     {cv::IMWRITE_JPEG_QUALITY, 90});
        db.save_panoramic_image(filename, jpeg_data, timestamp, node_id);
        ++imported;

      } else {
        // Regular JPEG: apply equirect guard (width == 2×height)
        const std::string filename = file.filename().string();
        if (db.has_panoramic_image(filename)) {
          spdlog::debug("Skipping already imported: {}", filename);
          continue;
        }

        const double timestamp = reusex::io::read_exif_timestamp(file);
        const int node_id = match_frame(timestamp, filename);
        const auto jpeg_data = read_file_bytes(file);
        db.save_panoramic_image(filename, jpeg_data, timestamp, node_id);
        ++imported;
      }
    }

    db.log_pipeline_end(log_id, true);
  } catch (...) {
    db.log_pipeline_end(log_id, false, "import_360 failed");
    throw;
  }

  spdlog::info("Import complete: {} panoramic images ({} matched to sensor "
               "frames)",
               imported, matched);

  return RuxError::SUCCESS;
}
