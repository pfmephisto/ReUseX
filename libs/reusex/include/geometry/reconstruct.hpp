// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace ReUseX {
class ProjectDB;
}

namespace ReUseX::geometry {

/// Parameters for point cloud reconstruction from stored sensor frames.
struct ReconstructionParams {
  float resolution = 0.05f;
  float min_distance = 0.0f;
  float max_distance = 4.0f;
  int sampling_factor = 4;
  int confidence_threshold = 2;
};

/// Generate merged point clouds from sensor frames stored in a ProjectDB.
///
/// For each sensor frame the function reads depth, confidence, color, pose,
/// and intrinsics; applies depth filters; performs custom pinhole
/// back-projection; estimates normals; and accumulates into a single cloud.
/// Post-processing includes voxelisation, NaN removal, label majority voting,
/// statistical outlier removal, and radius outlier removal.
///
/// Results are saved into the ProjectDB as "cloud", "normals", and "labels".
///
/// @param db     Project database (read/write).
/// @param params Reconstruction parameters.
void reconstruct_point_clouds(ProjectDB &db,
                              const ReconstructionParams &params);

} // namespace ReUseX::geometry
