// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// Parameters for Multi-View Stereo dense reconstruction (OpenMVS).
struct DensifyParams {
  /// Name to save the dense point cloud under in ProjectDB.
  std::string output_name = "dense";

  /// Source LiDAR cloud (name in ProjectDB) used to seed PatchMatch and the
  /// downstream 3D-point views. Empty disables seeding.
  std::string seed_cloud_name = "cloud";

  /// Sub-sample stride when copying the seed cloud (uniform).
  int seed_max_points = 100'000;

  /// 0 = full image resolution, 1 = half, 2 = quarter, ...
  int resolution_level = 1;

  /// Upper bound on max(width, height) for any image. Larger images are
  /// downsampled until they fit. Smaller is faster but loses detail.
  int max_resolution = 2000;

  /// Lower bound on max(width, height). Images below this are not used.
  int min_resolution = 640;

  /// Number of neighbor views considered per image during depth-map
  /// estimation. 0 = all calibrated images.
  int num_views = 4;

  /// Per-frame stride when subsampling RTABMap frames. 1 = use every frame,
  /// 2 = every other, etc. Use this to bound runtime on dense capture
  /// sessions.
  int frame_stride = 1;

  /// Geometric consistency between neighboring depth maps. Slower but
  /// produces cleaner output.
  bool geometric_consistency = true;

  /// CUDA device selection for PatchMatch depth-map estimation.
  ///   -2 = force CPU PatchMatch (safest, no CUDA init needed)
  ///   -1 = let OpenMVS pick the best GPU
  ///  >=0 = use the specified CUDA device index
  /// Default is -2 because some build environments expose CUDA at link
  /// time but cuInit() fails at runtime; the in-process MVS then crashes
  /// during depth-map estimation. Override with --gpu-index 0 (or -1) on
  /// a machine with a working CUDA runtime to get the GPU PatchMatch.
  int gpu_index = -2;

  /// Maximum concurrent OpenMP threads used during depth-map estimation.
  /// 0 = let OpenMVS pick (typically all CPU cores). The dominant memory
  /// driver is concurrent PatchMatch — each worker holds image pyramids,
  /// candidate depth/normal buffers, and neighbor-view caches in RAM. Set
  /// to a small number (e.g. 2 or 4) on memory-constrained machines.
  int max_threads = 0;

  /// Apply PCL's statistical outlier removal to the merged output after
  /// fusion. Drops local statistical outliers but is not aggressive
  /// enough to catch far-outlier *patches* (groups of bad points that
  /// have each other as neighbours).
  bool outlier_filter = true;
  int outlier_mean_k = 50;
  double outlier_stddev_thresh = 2.0;

  /// If the seed cloud (LiDAR) is present, clip the dense output to its
  /// percentile-based bounding box expanded by `bbox_margin` (relative
  /// expansion: 0.25 = 25% on each side). This catches the large
  /// triangulation-failure patches that SOR misses — they sit far
  /// outside the LiDAR volume, so a generous box around the LiDAR
  /// extent removes them while keeping anything plausibly part of the
  /// scene.
  bool bbox_clip = true;
  double bbox_margin = 0.25;

  /// Process at most this many frames per dense-reconstruction call,
  /// then merge the resulting point clouds. This is the only way to
  /// bound peak fusion RAM without lowering image resolution — fusion
  /// holds every depth map in memory at once, so smaller chunks →
  /// smaller fusion working set. Successive chunks are consecutive
  /// frames from the (frame_stride-filtered) sensor-frame list.
  /// 0 = disabled, fuse the whole scene at once. Reasonable values for
  /// 640×480 indoor scans are 30–80 frames depending on available RAM.
  int chunk_size = 0;

  /// Cap on the number of points kept after fusion (OPTDENSE.nMaxPointsFuse).
  /// 0 = unlimited. Setting this throttles the final output size.
  int max_fuse_points = 0;
};

/// Run OpenMVS Multi-View Stereo in-process to produce a dense point cloud
/// from the project's sensor frames.
///
/// This is the C++ library equivalent of the historical COLMAP
/// `patch_match_stereo` + `stereo_fusion` pipeline, but linked into
/// `libreusex` rather than invoked as a subprocess.
///
/// The function constructs an `MVS::Scene` directly from ProjectDB sensor
/// frames (one MVS Platform per intrinsics group, one Pose per frame), runs
/// `Scene::DenseReconstruction()`, and converts the resulting point cloud
/// into a `pcl::PointCloud<PointXYZRGB>` saved under @p params.output_name.
///
/// Decoded JPEGs are staged in a temp directory because OpenMVS's depth
/// estimator loads pixels through `cv::imread` regardless of API surface.
/// The temp directory is removed on success.
///
/// @throws std::runtime_error if no sensor frames are usable, the scene
///         cannot be built, or `DenseReconstruction` returns failure.
void densify_from_images(ProjectDB &db, const DensifyParams &params);

} // namespace reusex::geometry
