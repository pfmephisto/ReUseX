// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>

namespace reusex {
class ProjectDB;
}

namespace reusex::io {

/// Import a MuSHRoom dataset capture directory into a ReUseX project.
///
/// MuSHRoom (Ren et al., WACV 2024, CC-BY-4.0) provides room-scale RGB-D
/// sequences from consumer devices (iPhone LiDAR / Azure Kinect) together
/// with Faro laser-scanned ground-truth meshes — the same sensor class as
/// ReUseX's own iPad scans, which makes it the primary external benchmark
/// for reconstruction quality (issue #221 / #224).
///
/// Expects a capture directory (e.g. <room>/iphone/long_capture, or its
/// sdf_dataset_*_interp_4 subdirectory) containing the sdfstudio-style
/// export: meta_data.json + NNNNNN_rgb.png + NNNNNN_sensor_depth.npy.
///
/// meta_data.json stores OpenCV-convention camera-to-world poses in
/// *normalized* unit-box coordinates plus a `worldtogt` similarity
/// transform back to the metric ground-truth-mesh frame. The importer
/// applies that transform (keeping rotations orthonormal), so imported
/// poses and reconstructed clouds live directly in the GT mesh frame —
/// ready for accuracy comparison against gt_mesh.ply. Depth is stored as
/// normalized float32 .npy and converted to metric CV_16UC1 millimeters.
///
/// @param db           Open project database (frames are written to it).
/// @param capture_dir  Path to the capture directory described above.
/// @returns            Number of sensor frames imported.
/// @throws std::runtime_error on missing/malformed files.
std::size_t import_mushroom(ProjectDB &db,
                            const std::filesystem::path &capture_dir);

} // namespace reusex::io
