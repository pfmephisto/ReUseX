// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/panoramas.hpp"
#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "io/exif.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::io {

namespace {

namespace fs = std::filesystem;

/// Parse JPEG image dimensions (width, height) from the SOF0/SOF1/SOF2
/// marker without decoding the image. Reads at most ~64 KB of the file.
/// Returns {0, 0} if the file is not a valid JPEG or no SOF marker is found.
std::pair<int, int> read_jpeg_dimensions(const fs::path &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    return {0, 0};

  // SOI: FF D8
  uint8_t soi[2];
  if (!f.read(reinterpret_cast<char *>(soi), 2))
    return {0, 0};
  if (soi[0] != 0xFF || soi[1] != 0xD8)
    return {0, 0};

  // Scan marker segments until an SOF is found or the 64 KB scan limit is hit.
  constexpr std::streamoff kMaxScan = 65536;
  while (f.good() && f.tellg() < kMaxScan) {
    // Find next FF byte.
    uint8_t b = 0;
    while (f.read(reinterpret_cast<char *>(&b), 1) && b != 0xFF) {
    }
    if (!f.good())
      return {0, 0};
    // Skip FF padding bytes to reach the marker byte.
    while (f.read(reinterpret_cast<char *>(&b), 1) && b == 0xFF) {
    }
    if (!f.good())
      return {0, 0};
    const uint8_t marker = b;

    // SOF0 (baseline DCT), SOF1 (extended sequential), SOF2 (progressive DCT)
    // all carry [len(2), precision(1), height(2), width(2), ...].
    if (marker == 0xC0 || marker == 0xC1 || marker == 0xC2) {
      uint8_t buf[7]; // len_hi, len_lo, precision, h_hi, h_lo, w_hi, w_lo
      if (!f.read(reinterpret_cast<char *>(buf), 7))
        return {0, 0};
      const int h = (static_cast<int>(buf[3]) << 8) | buf[4];
      const int w = (static_cast<int>(buf[5]) << 8) | buf[6];
      return {w, h};
    }

    // All other segments: read 2-byte length (inclusive) and skip past them.
    uint8_t lb[2];
    if (!f.read(reinterpret_cast<char *>(lb), 2))
      return {0, 0};
    const int len = (static_cast<int>(lb[0]) << 8) | lb[1];
    if (len < 2)
      return {0, 0};
    f.seekg(len - 2, std::ios::cur);
  }
  return {0, 0};
}

std::vector<uint8_t> read_file_bytes(const fs::path &path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file)
    throw std::runtime_error("Cannot open file: " + path.string());
  const auto size = file.tellg();
  file.seekg(0);
  std::vector<uint8_t> data(static_cast<std::size_t>(size));
  file.read(reinterpret_cast<char *>(data.data()), size);
  return data;
}

} // anonymous namespace

std::size_t import_panoramas(ProjectDB &db, const fs::path &dir,
                             const ImportPanoramasOptions &opts) {
  if (opts.skip)
    return 0;

  // Resolve scan directories: prefer known hint subdirs; fall back to dir.
  std::vector<fs::path> scan_dirs;
  for (const auto &hint : opts.subdir_hints) {
    const auto candidate = dir / hint;
    if (fs::is_directory(candidate))
      scan_dirs.push_back(candidate);
  }
  if (scan_dirs.empty())
    scan_dirs.push_back(dir);

  // Collect JPEG candidates from scan dirs (one level deep, non-recursive).
  std::vector<fs::path> candidates;
  for (const auto &d : scan_dirs) {
    for (const auto &entry : fs::directory_iterator(d)) {
      if (!entry.is_regular_file())
        continue;
      auto ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext == ".jpg" || ext == ".jpeg")
        candidates.push_back(entry.path());
    }
  }
  std::sort(candidates.begin(), candidates.end());

  // Apply equirect guard: width == 2 * height (2:1 aspect ratio).
  std::vector<fs::path> equirect;
  for (const auto &p : candidates) {
    const auto [w, h] = read_jpeg_dimensions(p);
    if (w > 0 && h > 0 && w == 2 * h) {
      equirect.push_back(p);
    } else {
      reusex::debug("import_panoramas: skipping {} — not equirect ({}x{}, "
                    "need width==2*height)",
                    p.filename().string(), w, h);
    }
  }

  if (equirect.empty()) {
    reusex::warn("import_panoramas: no 2:1 equirect JPEGs found in {} "
                 "(checked {} candidate(s)); no panoramas imported",
                 dir.string(), candidates.size());
    return 0;
  }
  reusex::info("import_panoramas: {} equirect JPEG(s) found in {}",
               equirect.size(), dir.string());

  // Check whether any sensor frame carries a timestamp for matching.
  const auto frame_ids = db.sensor_frame_ids();
  bool has_timestamps = false;
  for (const auto id : frame_ids) {
    if (db.sensor_frame_timestamp(id) >= 0.0) {
      has_timestamps = true;
      break;
    }
  }
  if (!frame_ids.empty() && !has_timestamps) {
    reusex::warn("import_panoramas: sensor frames carry no timestamps — "
                 "panoramas will be imported unlinked (node_id=NULL) and will "
                 "be skipped by 'rux align 360' (which requires a linked "
                 "sensor frame as alignment seed)");
  }

  std::size_t imported = 0;
  std::size_t matched = 0;
  std::size_t unlinked = 0;

  for (const auto &file : equirect) {
    const std::string filename = file.filename().string();

    if (db.has_panoramic_image(filename)) {
      reusex::debug("import_panoramas: skipping already imported: {}",
                    filename);
      continue;
    }

    const double timestamp = reusex::io::read_exif_timestamp(file);

    int node_id = -1;
    if (timestamp >= 0.0 && has_timestamps) {
      node_id = db.nearest_sensor_frame_by_timestamp(timestamp);
      if (node_id >= 0) {
        const double frame_ts = db.sensor_frame_timestamp(node_id);
        const double dt = std::abs(timestamp - frame_ts);
        reusex::info("import_panoramas: {} -> sensor frame {} (dt={:.1f}s)",
                     filename, node_id, dt);
        ++matched;
      } else {
        reusex::warn("import_panoramas: {} — no sensor frame matched by "
                     "timestamp; imported unlinked (node_id=NULL); 'rux align "
                     "360' will skip this panorama",
                     filename);
        ++unlinked;
      }
    } else {
      reusex::warn("import_panoramas: {} — no EXIF timestamp; imported "
                   "unlinked (node_id=NULL); 'rux align 360' will skip this "
                   "panorama",
                   filename);
      ++unlinked;
    }

    const auto jpeg_data = read_file_bytes(file);
    db.save_panoramic_image(filename, jpeg_data, timestamp, node_id);
    ++imported;
  }

  reusex::info("import_panoramas: {} imported ({} linked to a sensor frame, "
               "{} unlinked)",
               imported, matched, unlinked);
  return imported;
}

} // namespace reusex::io
