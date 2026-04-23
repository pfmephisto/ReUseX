// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/panorama.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/exif.hpp>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace {

/// Collect all JPEG files from a list of paths (files or directories).
std::vector<fs::path>
collect_jpeg_files(const std::vector<fs::path> &input_paths) {
  std::vector<fs::path> files;

  for (const auto &path : input_paths) {
    if (fs::is_regular_file(path)) {
      auto ext = path.extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext == ".jpg" || ext == ".jpeg") {
        files.push_back(path);
      } else {
        spdlog::warn("Skipping non-JPEG file: {}", path.string());
      }
    } else if (fs::is_directory(path)) {
      for (const auto &entry : fs::directory_iterator(path)) {
        if (!entry.is_regular_file())
          continue;
        auto ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".jpg" || ext == ".jpeg") {
          files.push_back(entry.path());
        }
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
  auto *sub =
      app.add_subcommand("360", "Import 360 panoramic images");

  sub->footer(R"(
DESCRIPTION:
  Imports 360 panoramic JPEG images into a ReUseX project database.
  Each image is matched to the nearest sensor frame by EXIF timestamp,
  enabling spatial association between panoramic images and SLAM poses.

EXAMPLES:
  rux import 360 /path/to/photos/       # Import all JPEGs from directory
  rux import 360 img1.jpg img2.jpg      # Import specific files
  rux -p office.rux import 360 ./360/   # Custom project path

WORKFLOW:
  1. rux import rtabmap scan.db          # Import SLAM data (with timestamps)
  2. rux import 360 /path/to/360-images/ # Match panoramas to sensor frames
  3. rux get panoramas                   # List imported panoramas

NOTES:
  - Requires sensor frames with timestamps (import RTABMap data first)
  - Reads EXIF DateTimeOriginal to determine capture time
  - Images stored as raw JPEG blobs (no re-encoding)
  - Each panorama is matched to the closest sensor frame by timestamp
  - Supports .jpg and .jpeg extensions (case-insensitive)
)");

  sub->add_option("paths", opt->input_paths,
                  "JPEG files or directories containing panoramic images")
      ->required()
      ->check(CLI::ExistingPath);

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_panorama");
    return run_subcommand_import_panorama(*opt, *global_opt);
  });
}

int run_subcommand_import_panorama(SubcommandImportPanoramaOptions const &opt,
                                   const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing 360 panoramic images to project: {}",
               project_path.string());

  reusex::ProjectDB db(project_path);

  // Collect all JPEG files
  auto files = collect_jpeg_files(opt.input_paths);
  if (files.empty()) {
    spdlog::warn("No JPEG files found in the specified paths");
    return RuxError::SUCCESS;
  }
  spdlog::info("Found {} JPEG files to import", files.size());

  // Check that sensor frames exist and have timestamps
  auto frame_ids = db.sensor_frame_ids();
  bool has_timestamps = false;
  if (!frame_ids.empty()) {
    // Check if at least one frame has a timestamp
    for (const auto &fid : frame_ids) {
      if (db.sensor_frame_timestamp(fid) >= 0.0) {
        has_timestamps = true;
        break;
      }
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
      "import_360",
      fmt::format(R"({{"file_count":{}}})", files.size()));

  int imported = 0;
  int matched = 0;

  try {
    for (const auto &file : files) {
      std::string filename = file.filename().string();

      // Skip if already imported
      if (db.has_panoramic_image(filename)) {
        spdlog::debug("Skipping already imported: {}", filename);
        continue;
      }

      // Read EXIF timestamp
      double timestamp = reusex::io::read_exif_timestamp(file);

      // Find nearest sensor frame
      int node_id = -1;
      if (timestamp >= 0.0 && has_timestamps) {
        node_id = db.nearest_sensor_frame_by_timestamp(timestamp);
        if (node_id >= 0) {
          double frame_ts = db.sensor_frame_timestamp(node_id);
          double dt = std::abs(timestamp - frame_ts);
          spdlog::info("Imported {} -> node {} (dt={:.1f}s)", filename,
                       node_id, dt);
          ++matched;
        } else {
          spdlog::info("Imported {} (no matching sensor frame)", filename);
        }
      } else {
        spdlog::info("Imported {} (no EXIF timestamp)", filename);
      }

      // Read raw JPEG bytes
      auto jpeg_data = read_file_bytes(file);

      // Store in database
      db.save_panoramic_image(filename, jpeg_data, timestamp, node_id);
      ++imported;
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
