// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// Controls auto-discovery and import of companion 360° panoramas.
struct ImportPanoramasOptions {
  /// When true, skip panorama discovery entirely (--no-panoramas CLI flag).
  bool skip = false;

  /// Subdirectory names searched first for companion panoramas.
  /// If any of these directories exist under the capture root, only those
  /// directories are scanned (non-recursively). If none exist, the root
  /// directory itself is scanned one level deep as a fallback.
  std::vector<std::string> subdir_hints = {"360", "panoramas", "pano"};
};

/// Auto-discover and import companion 360° equirect JPEGs from a capture
/// directory into the project database.
///
/// Discovery order: for each name in @p opts.subdir_hints, check whether
/// @p dir/<name> is a directory and collect it; if any hint directories are
/// found those are scanned exclusively. If none exist, @p dir itself is
/// scanned one level deep as a fallback (non-recursive, to avoid picking up
/// RGB-D frame images that live in known subdirectories).
///
/// Equirect guard: only files whose JPEG header reports width == 2 × height
/// are treated as panoramas. Non-equirect JPEGs are skipped at debug level.
///
/// Each candidate is matched to the nearest sensor frame by EXIF timestamp.
/// If sensor frames carry no timestamps, or a file has no EXIF timestamp, the
/// panorama is imported with node_id=NULL and a warn-level log is emitted —
/// such panoramas are stored in the DB but are skipped by `rux align 360`
/// (which requires a linked sensor frame as an alignment seed).
///
/// @param db   Open, writable project database.
/// @param dir  Capture root directory to search.
/// @param opts Discovery and filtering options.
/// @returns    Number of panoramas newly imported (0 if none found).
std::size_t import_panoramas(ProjectDB &db, const std::filesystem::path &dir,
                             const ImportPanoramasOptions &opts = {});

} // namespace reusex::io
