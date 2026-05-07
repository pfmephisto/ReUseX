// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/types.hpp"

#include <filesystem>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

// ── Import ────────────────────────────────────────────────────────────────────

/// Import a point cloud from a PLY file into a ProjectDB.
///
/// Detects available fields and extracts all data present:
/// - XYZ position (required)
/// - RGB color → stored as XYZRGB cloud named after the file stem
/// - Normals (normal_x/y/z or nx/ny/nz) → stored as a separate cloud
///   with a "_normals" suffix
/// - Semantic labels → stored as a separate cloud with a "_labels" suffix
/// - Intensity without color → mapped to grayscale RGB
///
/// @param db Target project database (must be open in write mode).
/// @param ply_path Path to the source .ply file.
void import_ply(ProjectDB &db, const std::filesystem::path &ply_path);

// ── Export ────────────────────────────────────────────────────────────────────

/// Export a point cloud to a binary PLY file.
///
/// When @p normals is provided it must have the same number of points as
/// @p cloud; both are written as a single PLY element with merged fields
/// (x, y, z, rgb, normal_x, normal_y, normal_z).
///
/// @param path    Output .ply file path (created or overwritten).
/// @param cloud   XYZRGB point cloud to export.
/// @param normals Surface normals to merge into the output (nullptr = skip).
void export_ply(const std::filesystem::path &path,
                const Cloud &cloud,
                const CloudN *normals = nullptr);

} // namespace reusex::io
