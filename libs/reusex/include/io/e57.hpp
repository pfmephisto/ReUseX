// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/types.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

// ── Import ────────────────────────────────────────────────────────────────────

/// Import point cloud data from an E57 file into a ProjectDB.
///
/// Each scan in the E57 file is stored as a separate named XYZRGB point cloud.
/// Color, normals, and intensity data are extracted when available. If color is
/// absent but intensity is present, intensity is mapped to grayscale RGB. Normals
/// are stored as a separate cloud with a "_normals" suffix.
///
/// Cloud naming: uses the scan name from the E57 header when set, otherwise
/// "{file_stem}_scan_{N}".
///
/// @param db Target project database (must be open in write mode).
/// @param e57_path Path to the source .e57 file.
void import_e57(ProjectDB &db, const std::filesystem::path &e57_path);

// ── Export ────────────────────────────────────────────────────────────────────

/// A single scan position to write into an E57 file.
struct E57ScanExport {
    std::string    name;     ///< Scan name written into the E57 header.
    CloudConstPtr  cloud;    ///< XYZRGB point cloud (required).
    CloudNConstPtr normals;  ///< Surface normals (nullptr = omit normals).
};

/// Export one or more XYZRGB clouds as separate scan positions in an E57 file.
///
/// Each entry in @p scans becomes one Data3D block. RGB color is always written.
/// Normals are written using the E57_EXT_surface_normals extension when provided.
///
/// @param path  Output .e57 file path (created or overwritten).
/// @param scans One or more scans to embed in the file.
void export_e57(const std::filesystem::path &path,
                const std::vector<E57ScanExport> &scans);

} // namespace reusex::io
