// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "import/photos.hpp"

#include <reusex/core/MaterialPassport.hpp>
#include <reusex/core/ProjectDB.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>

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

/// Base64-encode a byte buffer.
std::string base64_encode(const std::vector<uint8_t> &data) {
  static constexpr char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string result;
  result.reserve(((data.size() + 2) / 3) * 4);

  size_t i = 0;
  for (; i + 2 < data.size(); i += 3) {
    uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
                 (static_cast<uint32_t>(data[i + 1]) << 8) |
                 static_cast<uint32_t>(data[i + 2]);
    result.push_back(table[(n >> 18) & 0x3F]);
    result.push_back(table[(n >> 12) & 0x3F]);
    result.push_back(table[(n >> 6) & 0x3F]);
    result.push_back(table[n & 0x3F]);
  }

  if (i + 1 == data.size()) {
    uint32_t n = static_cast<uint32_t>(data[i]) << 16;
    result.push_back(table[(n >> 18) & 0x3F]);
    result.push_back(table[(n >> 12) & 0x3F]);
    result.push_back('=');
    result.push_back('=');
  } else if (i + 2 == data.size()) {
    uint32_t n = (static_cast<uint32_t>(data[i]) << 16) |
                 (static_cast<uint32_t>(data[i + 1]) << 8);
    result.push_back(table[(n >> 18) & 0x3F]);
    result.push_back(table[(n >> 12) & 0x3F]);
    result.push_back(table[(n >> 6) & 0x3F]);
    result.push_back('=');
  }

  return result;
}

/// Parse an ISO 8601 UTC timestamp like "2026-04-22T10:52:04Z" to epoch
/// seconds. Returns -1.0 on failure.
double parse_iso8601_utc(const std::string &ts) {
  std::tm tm = {};
  std::istringstream ss(ts);
  ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
  if (ss.fail())
    return -1.0;
  return static_cast<double>(timegm(&tm));
}

/// Parse timestamp from a filename like "2026-04-22T10-52-05.043_9fa66e5c.jpg".
/// Converts the filename-safe format (dashes instead of colons) to ISO 8601
/// and then parses it. Returns -1.0 on failure.
double parse_filename_timestamp(const std::string &stem) {
  // Expected: "2026-04-22T10-52-05.043_9fa66e5c"
  //           YYYY-MM-DDTHH-MM-SS.mmm_ID
  if (stem.size() < 19)
    return -1.0;

  // Extract date-time portion: "2026-04-22T10-52-05"
  // Convert to "2026-04-22T10:52:05"
  std::string dt = stem.substr(0, 19);
  // Positions 13 and 16 should be '-' (hour-minute, minute-second separators)
  if (dt.size() >= 17 && dt[13] == '-' && dt[16] == '-') {
    dt[13] = ':';
    dt[16] = ':';
  }
  dt += "Z";

  return parse_iso8601_utc(dt);
}

/// Extract photo ID from filename stem like "2026-04-22T10-52-05.043_9fa66e5c".
/// Returns the part after the last underscore.
std::string extract_id_from_filename(const std::string &stem) {
  auto pos = stem.rfind('_');
  if (pos != std::string::npos && pos + 1 < stem.size())
    return stem.substr(pos + 1);
  return stem;
}

} // anonymous namespace

void setup_subcommand_import_photos(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandImportPhotosOptions>();
  auto *sub = app.add_subcommand("photos", "Import manual survey photos");

  sub->footer(R"(
DESCRIPTION:
  Imports manual survey photos into a ReUseX project database. Each photo
  creates a material passport linked to the nearest sensor frame by timestamp.
  Photos are stored as base64-encoded images in the passport's images field.

  Supports .jpg/.json file pairs where the JSON sidecar contains metadata
  (capturedAt, id, user label). Falls back to parsing timestamp and ID from
  the filename if no JSON sidecar is found.

EXAMPLES:
  rux import photos ./manual_photos/          # Import all JPEGs from directory
  rux import photos img1.jpg img2.jpg         # Import specific files
  rux -p office.rux import photos ./photos/   # Custom project path
  rux import photos ./photos/ --project-id p1 # Link to specific project

WORKFLOW:
  1. rux import rtabmap scan.db               # Import SLAM data (with timestamps)
  2. rux import photos ./manual_photos/       # Import survey photos
  3. rux get materials                        # List created passports

NOTES:
  - Requires sensor frames with timestamps (import RTABMap data first)
  - Each JPEG creates one material passport
  - Passport ID is set to sensor frame node_id for spatial linking
  - JSON sidecar (same base name, .json) provides capturedAt and id
  - Without sidecar, timestamp and ID are parsed from filename
  - Supports .jpg and .jpeg extensions (case-insensitive)
)");

  sub->add_option("paths", opt->input_paths,
                  "JPEG files or directories containing survey photos")
      ->required()
      ->check(CLI::ExistingPath);

  sub->add_option("--project-id", opt->project_id,
                  "Project identifier (optional)");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling run_subcommand_import_photos");
    return run_subcommand_import_photos(*opt, *global_opt);
  });
}

int run_subcommand_import_photos(SubcommandImportPhotosOptions const &opt,
                                 const RuxOptions &global_opt) {
  fs::path project_path = global_opt.project_db;
  spdlog::info("Importing survey photos to project: {}",
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
    for (const auto &fid : frame_ids) {
      if (db.sensor_frame_timestamp(fid) >= 0.0) {
        has_timestamps = true;
        break;
      }
    }
  }

  if (frame_ids.empty()) {
    spdlog::warn("No sensor frames in project. Photos will be imported "
                 "without spatial matching.");
  } else if (!has_timestamps) {
    spdlog::warn("Sensor frames have no timestamps. Re-import RTABMap data "
                 "to enable timestamp matching.");
  }

  int log_id = db.log_pipeline_start(
      "import_photos",
      fmt::format(R"({{"file_count":{}}})", files.size()));

  int imported = 0;
  int matched = 0;

  try {
    for (const auto &file : files) {
      std::string filename = file.filename().string();
      std::string stem = file.stem().string();

      // Try to find matching JSON sidecar
      fs::path json_path = file;
      json_path.replace_extension(".json");

      std::string photo_id;
      std::string captured_at;
      double timestamp = -1.0;

      if (fs::is_regular_file(json_path)) {
        // Parse JSON sidecar
        std::ifstream jf(json_path);
        if (jf.is_open()) {
          try {
            auto j = nlohmann::json::parse(jf);
            if (j.contains("capturedAt") && j["capturedAt"].is_string())
              captured_at = j["capturedAt"].get<std::string>();
            if (j.contains("id") && j["id"].is_string())
              photo_id = j["id"].get<std::string>();
          } catch (const nlohmann::json::exception &e) {
            spdlog::warn("Failed to parse JSON sidecar {}: {}",
                         json_path.string(), e.what());
          }
        }
      }

      // Fallback: parse from filename if JSON didn't provide values
      if (captured_at.empty()) {
        // Try to reconstruct ISO timestamp from filename
        double ts = parse_filename_timestamp(stem);
        if (ts >= 0.0) {
          // Reconstruct the ISO string from the filename
          std::string dt = stem.substr(0, 19);
          if (dt.size() >= 17 && dt[13] == '-' && dt[16] == '-') {
            dt[13] = ':';
            dt[16] = ':';
          }
          captured_at = dt + "Z";
        }
      }
      if (photo_id.empty())
        photo_id = extract_id_from_filename(stem);

      // Parse timestamp for sensor frame matching
      if (!captured_at.empty())
        timestamp = parse_iso8601_utc(captured_at);

      // Find nearest sensor frame
      int node_id = -1;
      if (timestamp >= 0.0 && has_timestamps) {
        node_id = db.nearest_sensor_frame_by_timestamp(timestamp);
        if (node_id >= 0) {
          double frame_ts = db.sensor_frame_timestamp(node_id);
          double dt = std::abs(timestamp - frame_ts);
          spdlog::info("{} -> node {} (dt={:.1f}s)", filename, node_id, dt);
          ++matched;
        } else {
          spdlog::info("{} (no matching sensor frame)", filename);
        }
      } else {
        spdlog::info("{} (no timestamp available)", filename);
      }

      // Read raw JPEG bytes and encode as base64
      auto jpeg_data = read_file_bytes(file);
      std::string b64 = base64_encode(jpeg_data);

      // Build material passport
      reusex::core::MaterialPassport passport;
      passport.metadata.document_guid = photo_id;
      passport.metadata.creation_date = captured_at;
      passport.description.images.push_back(std::move(b64));

      // Store passport with node_id as the row id (links to sensor frame)
      if (node_id >= 0) {
        db.add_material_passport(passport, opt.project_id,
                                 std::to_string(node_id));
      } else {
        db.add_material_passport(passport, opt.project_id);
      }
      ++imported;
    }

    db.log_pipeline_end(log_id, true);
  } catch (...) {
    db.log_pipeline_end(log_id, false, "import_photos failed");
    throw;
  }

  spdlog::info("Import complete: {} photos ({} matched to sensor frames)",
               imported, matched);

  return RuxError::SUCCESS;
}
