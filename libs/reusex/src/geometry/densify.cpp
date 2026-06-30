// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// NOTE: OpenMVS is AGPL-3.0-or-later. Linking it into libreusex makes the
// combined work effectively AGPL-3.0-or-later (compatible with our
// GPL-3.0-or-later upgrade clause).

#include "geometry/densify.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/processing_observer.hpp"
#include "types.hpp"

// OpenMVS pulls in <Windows.h>-style macros; isolate it.
// MVS.h MUST come before UtilCUDA.h: the latter is entirely wrapped in
// `#ifdef _USE_CUDA`, and `_USE_CUDA` is defined by OpenMVS/ConfigLocal.h
// which is reached via MVS.h. Including UtilCUDA.h first sets its header
// guard with an empty body, then Mesh.h (pulled in by MVS.h) references
// SEACAVE::CUDA::* types that were never declared.
#include <OpenMVS/MVS.h>
#include <OpenMVS/Common/UtilCUDA.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace reusex::geometry {

namespace {

namespace fs = std::filesystem;

Eigen::Matrix4d to_matrix4d(const std::array<double, 16> &m) {
  Eigen::Matrix4d out;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out(r, c) = m[r * 4 + c];
  return out;
}

struct IntrinsicsKey {
  int width, height;
  int64_t fx_q, fy_q, cx_q, cy_q;
};
bool operator==(const IntrinsicsKey &a, const IntrinsicsKey &b) {
  return a.width == b.width && a.height == b.height && a.fx_q == b.fx_q &&
         a.fy_q == b.fy_q && a.cx_q == b.cx_q && a.cy_q == b.cy_q;
}
struct IntrinsicsKeyHash {
  std::size_t operator()(const IntrinsicsKey &k) const noexcept {
    auto mix = [](std::size_t h, std::size_t v) {
      return h ^ (v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
    };
    std::size_t h = std::hash<int>{}(k.width);
    h = mix(h, std::hash<int>{}(k.height));
    h = mix(h, std::hash<int64_t>{}(k.fx_q));
    h = mix(h, std::hash<int64_t>{}(k.fy_q));
    h = mix(h, std::hash<int64_t>{}(k.cx_q));
    h = mix(h, std::hash<int64_t>{}(k.cy_q));
    return h;
  }
};

IntrinsicsKey to_key(const core::SensorIntrinsics &i) {
  // Quantize to whole pixels. Sub-pixel intrinsic variation (autofocus,
  // floating-point noise on phone captures) would otherwise produce
  // ~1 camera per frame, which murders SelectNeighborViews scaling and
  // bloats Scene::platforms. 1-pixel rounding still captures genuinely
  // different cameras (different lenses or resolutions) without false
  // splits.
  auto q = [](double v) -> int64_t {
    return static_cast<int64_t>(std::llround(v));
  };
  return IntrinsicsKey{i.width, i.height, q(i.fx), q(i.fy), q(i.cx), q(i.cy)};
}

/// Populate OpenMVS's global dense-reconstruction config with sensible
/// defaults. These match the values DensifyPointCloud.cpp would assign via
/// command-line parsing — without parsing, globals are zero-initialized and
/// the algorithm misbehaves.
void initOpenMVSDefaults(const DensifyParams &p, unsigned numThreads) {
  using namespace MVS;
  OPTDENSE::nResolutionLevel =
      static_cast<unsigned>(std::max(0, p.resolution_level));
  // 0 from the caller means "no cap" — feed OpenMVS its own default
  // (3200) which behaves as the upper bound when input frames are huge.
  OPTDENSE::nMaxResolution =
      p.max_resolution > 0 ? static_cast<unsigned>(p.max_resolution) : 3200u;
  OPTDENSE::nMinResolution =
      static_cast<unsigned>(std::max(1, p.min_resolution));
  OPTDENSE::nSubResolutionLevels = 2;
  OPTDENSE::nMinViews = 2;
  OPTDENSE::nMaxViews = 12;
  // Fusion bounds — leaving any of these at 0 means "unbounded" for the
  // corresponding loop in DenseFuseDepthMaps and blew up to 100+ GB on
  // dense indoor scans. Use upstream defaults except nMinPixelsFuse,
  // which we lower (5 → 2) because the upstream value rejects every
  // candidate on indoor scans with imperfect pose/intrinsic consistency.
  OPTDENSE::nMinPixelsFuse = 2;
  OPTDENSE::nMinViewsFuse = 2;
  OPTDENSE::nMaxViewsFuse = 32;
  OPTDENSE::nMaxFuseDepth = 100;
  OPTDENSE::nMaxPointsFuse =
      p.max_fuse_points > 0 ? static_cast<unsigned>(p.max_fuse_points) : 1000u;
  OPTDENSE::nMinViewsFilter = 1;
  OPTDENSE::nMinViewsFilterAdjust = 1;
  OPTDENSE::nMinViewsTrustPoint = 2;
  OPTDENSE::nNumViews = static_cast<unsigned>(std::max(0, p.num_views));
  OPTDENSE::bAddCorners = false;
  OPTDENSE::bInitSparse = true;
  OPTDENSE::bRemoveDmaps = true;
  OPTDENSE::fViewMinScore = 0.f;
  OPTDENSE::fViewMinScoreRatio = 0.3f;
  OPTDENSE::fMinArea = 0.05f;
  OPTDENSE::fMinAngle = 3.f;
  OPTDENSE::fOptimAngle = 12.f;
  OPTDENSE::fMaxAngle = 65.f;
  OPTDENSE::fWeightPointInsideROI = 0.7f;
  OPTDENSE::fDescriptorMinMagnitudeThreshold = 0.01f;
  OPTDENSE::fDepthReprojectionErrorThreshold = 0.01f;
  OPTDENSE::fDepthDiffThreshold = 0.01f;
  OPTDENSE::fNormalDiffThreshold = 25.f;
  OPTDENSE::fPairwiseMul = 0.3f;
  OPTDENSE::fOptimizerEps = 0.001f;
  OPTDENSE::nOptimizerMaxIters = 80;
  OPTDENSE::nSpeckleSize = 100;
  OPTDENSE::nIpolGapSize = 7;
  OPTDENSE::nIgnoreMaskLabel = -1;
  OPTDENSE::nOptimize = (p.geometric_consistency ? OPTDENSE::OPTIMIZE : 0u);
  // FuseDepthMaps (FUSE_FILTER) was previously suspected of unbounded
  // RAM growth; heaptrack revealed that was actually our spdmon progress
  // hook, not FuseDepthMaps. FUSE_FILTER yields a much higher point
  // count than FUSE_DENSEFILTER on indoor LiDAR-seeded scans and now
  // has bounded RAM with the upstream OPTDENSE limits we set above.
  OPTDENSE::nFuseFilter = OPTDENSE::FUSE_FILTER;
  OPTDENSE::nEstimateColors = 2;  // estimate colors during fusion
  // OpenMVS upstream default for nEstimateNormals is 2 ("estimate
  // normals"). Leaving it at 0 caused DenseFuseDepthMaps to filter out
  // every candidate and return 0 points (because the fusion graph
  // traversal compares pixel normals and skips disagreements). With 2
  // we get normals estimated during the depth pass and fusion works.
  OPTDENSE::nEstimateNormals = 2;
  OPTDENSE::fNCCThresholdKeep = 0.55f;
  OPTDENSE::nEstimationIters = 4;
  OPTDENSE::nEstimationGeometricIters = (p.geometric_consistency ? 2u : 0u);
  OPTDENSE::fEstimationGeometricWeight = 0.1f;
  OPTDENSE::nRandomIters = 6;
  OPTDENSE::nRandomMaxScale = 2;
  OPTDENSE::fRandomDepthRatio = 0.003f;
  OPTDENSE::fRandomAngle1Range = 16.f;
  OPTDENSE::fRandomAngle2Range = 10.f;
  OPTDENSE::fRandomSmoothDepth = 0.02f;
  OPTDENSE::fRandomSmoothNormal = 13.f;
  OPTDENSE::fRandomSmoothBonus = 0.93f;
  (void)numThreads;
}

/// Initialise SEACAVE's WORKING_FOLDER to @p folder so OpenMVS resolves
/// the relative image paths it stores against that root.
void setWorkingFolder(const fs::path &folder) {
  WORKING_FOLDER = SEACAVE::String(folder.string() + "/");
  INIT_WORKING_FOLDER;
}

/// Active progress observer for the dense-reconstruction stage. Set by
/// densify_from_images() before invoking Scene::DenseReconstruction() and
/// torn down after. The OpenMVS log listener consults it to tick once per
/// completed depth-map.
core::ProgressObserver *g_dense_observer = nullptr;

/// Listener forwarded to OpenMVS's SEACAVE::Log so its diagnostics
/// (configuration warnings, depth-map failures, etc.) reach the ReUseX
/// log handler instead of being silently swallowed.
///
/// OpenMVS messages come with a trailing newline and varying severity
/// embedded in the body ("error:", "warning:"). We strip the newline,
/// classify so progress noise sits at debug while genuine problems
/// surface at warn/error, and drive the ReUseX progress observer by
/// counting "Depth-map for image N estimated" lines.
void openmvs_log_forward(const SEACAVE::String &raw) {
  std::string_view msg{raw.c_str(), raw.length()};
  while (!msg.empty() &&
         (msg.back() == '\n' || msg.back() == '\r' || msg.back() == ' '))
    msg.remove_suffix(1);
  if (msg.empty())
    return;
  // Strip OpenMVS's own "HH:MM:SS " timestamp prefix; spdlog already
  // stamps every line so we'd otherwise get two timestamps per message.
  const auto is_digit = [](char c) { return c >= '0' && c <= '9'; };
  if (msg.size() >= 9 && is_digit(msg[0]) && is_digit(msg[1]) &&
      msg[2] == ':' && is_digit(msg[3]) && is_digit(msg[4]) && msg[5] == ':' &&
      is_digit(msg[6]) && is_digit(msg[7]) && msg[8] == ' ') {
    msg.remove_prefix(9);
  }

  // NB: We no longer tick a spdmon progress observer from this callback.
  // Doing so triggered spdmon::BaseProgress::RenderProgress → fmt
  // memory_buffer growth from inside OMP worker threads, which heaptrack
  // measured at 157 GB peak on a 238-frame scan. OpenMVS prints its own
  // \r-based "Estimated depth-maps N (X%, ETA …)..." line to std::cout,
  // which costs nothing — that's the progress the user sees now.
  const bool is_depth_done =
      msg.find("Depth-map for image") != std::string_view::npos &&
      msg.find("estimated") != std::string_view::npos;

  // Cheap substring scan — OpenMVS only labels severity in a handful of
  // ways. Anything we don't recognise is treated as low-priority chatter.
  const bool is_error = msg.find("error:") != std::string_view::npos ||
                        msg.find("ERROR:") != std::string_view::npos ||
                        msg.find("CUDA error") != std::string_view::npos;
  const bool is_warning =
      !is_error && (msg.find("warning:") != std::string_view::npos ||
                    msg.find("WARNING") != std::string_view::npos);

  // Per-frame chatter ("Depth-map for image N estimated", "Reference
  // image N paired with K views") fires hundreds of times during a scan
  // and overwhelms the spdmon status line at debug level. Route it to
  // trace so it's still visible under -vvv, while -v / -vv keep a clean
  // progress bar. Phase markers (Preparing/Selecting/etc.) stay at
  // debug; warnings and errors always surface.
  const bool is_per_frame_chatter =
      !is_error && !is_warning &&
      (is_depth_done || msg.find("Reference image") != std::string_view::npos);

  if (is_error)
    reusex::error("{}", msg);
  else if (is_warning)
    reusex::warn("{}", msg);
  else if (is_per_frame_chatter)
    reusex::trace("{}", msg);
  else
    reusex::debug("{}", msg);
}

/// RAII guard that installs a ProgressObserver for the dense stage and
/// publishes it to the OpenMVS log listener for the duration of the scope.
struct DenseProgressScope {
  core::ProgressObserver observer;
  explicit DenseProgressScope(std::size_t total)
      : observer(core::Stage::dense_reconstruction, total) {
    g_dense_observer = &observer;
  }
  ~DenseProgressScope() { g_dense_observer = nullptr; }
};

/// RAII guard that silences std::cout for the duration of its scope.
/// OpenMVS's SEACAVE::Util::Progress writes its own "Estimated depth-maps
/// N (X%, ETA ...)..." line directly to std::cout with carriage returns;
/// that bypasses spdlog and clashes with our spdmon-driven progress bar.
/// Sinking cout into a discarded stringstream while DenseReconstruction
/// runs leaves our bar as the only one painting.
class CoutSilencer {
    public:
  CoutSilencer() : prev_(std::cout.rdbuf(sink_.rdbuf())) {}
  ~CoutSilencer() { std::cout.rdbuf(prev_); }
  CoutSilencer(const CoutSilencer &) = delete;
  CoutSilencer &operator=(const CoutSilencer &) = delete;

    private:
  std::ostringstream sink_;
  std::streambuf *prev_;
};

void attachOpenMVSLog() {
  // Open() is idempotent across re-entry; the static singleton survives
  // across calls so we keep the listener attached for the process.
  static bool attached = false;
  if (attached)
    return;
  SEACAVE::Log::GetInstance().Open();
  SEACAVE::Log::GetInstance().RegisterListener(
      SEACAVE::Log::ClbkRecordMsg::from<&openmvs_log_forward>());
  attached = true;
}

} // anonymous namespace

void densify_from_images(ProjectDB &db, const DensifyParams &p) {
  using namespace MVS;

  auto all_frame_ids = db.sensor_frame_ids();
  if (all_frame_ids.empty())
    throw std::runtime_error(
        "densify_from_images: ProjectDB has no sensor frames");

  // ── Workspace setup ──────────────────────────────────────────────────
  const fs::path workspace =
      fs::temp_directory_path() /
      fmt::format("rux-densify-{}", static_cast<long>(::getpid()));
  fs::create_directories(workspace);
  attachOpenMVSLog();

  // -2 = CPU PatchMatch, -1 = best GPU, >=0 = specific CUDA device.
  // Surfaced via the --gpu-index flag in rux create dense. SEACAVE::CUDA only
  // exists when OpenMVS was built with CUDA (_USE_CUDA, set by its headers);
  // a CPU/ROCm OpenMVS always runs the CPU PatchMatch.
#ifdef _USE_CUDA
  SEACAVE::CUDA::desiredDeviceID = p.gpu_index;
#else
  (void)p.gpu_index;
#endif

  reusex::info("OpenMVS densify workspace: {}", workspace.string());

  // ── Apply --frame-stride and partition into chunks ──────────────────
  const int stride = std::max(1, p.frame_stride);
  std::vector<int> effective_frames;
  effective_frames.reserve(all_frame_ids.size() / stride + 1);
  for (std::size_t fi = 0; fi < all_frame_ids.size();
       fi += static_cast<std::size_t>(stride))
    effective_frames.push_back(all_frame_ids[fi]);

  const std::size_t total_frames = effective_frames.size();
  const std::size_t chunk_size =
      (p.chunk_size > 0 && static_cast<std::size_t>(p.chunk_size) < total_frames)
          ? static_cast<std::size_t>(p.chunk_size)
          : total_frames;
  std::vector<std::vector<int>> chunks;
  for (std::size_t i = 0; i < total_frames; i += chunk_size) {
    chunks.emplace_back(effective_frames.begin() + i,
                        effective_frames.begin() +
                            std::min(i + chunk_size, total_frames));
  }
  if (chunks.size() > 1) {
    reusex::info("Processing {} frames in {} chunks of up to {}",
                 total_frames, chunks.size(), chunk_size);
  } else if (total_frames > 500) {
    // SelectNeighborViews scales as O(N²) per image pair. Above a few
    // hundred frames it dominates wall-clock and gives no progress
    // visibility while it runs. Encourage the user to chunk.
    reusex::warn(
        "Large scan ({} frames) without --chunk-size: SelectNeighborViews "
        "is O(N²) and will spend many minutes on image-pair scoring "
        "before depth-map estimation even starts. Consider passing "
        "--chunk-size 100 to split the work; total wall-clock will be "
        "similar but progress will be visible per chunk.",
        total_frames);
  }

  // Helper: append an MVS::PointCloud into a PCL XYZRGB cloud.
  const auto append_mvs_points = [](const PointCloud &src, Cloud &dst) {
    const bool has_colors = src.colors.size() == src.points.size();
    dst.reserve(dst.size() + src.points.size());
    for (std::size_t i = 0; i < src.points.size(); ++i) {
      const auto &p3 = src.points[i];
      PointT pt;
      pt.x = p3.x;
      pt.y = p3.y;
      pt.z = p3.z;
      if (has_colors) {
        const auto &c = src.colors[i];
        pt.r = c.r;
        pt.g = c.g;
        pt.b = c.b;
      } else {
        pt.r = pt.g = pt.b = 200;
      }
      pt.a = 255;
      dst.push_back(pt);
    }
  };

  Cloud out;
  bool debug_intrinsics_printed = false;
  bool debug_camera_printed = false;

  for (std::size_t ci = 0; ci < chunks.size(); ++ci) {
    const auto &chunk_frames = chunks[ci];
    const bool chunked = chunks.size() > 1;

    // Per-chunk workspace so on-disk artefacts (dmaps, images) don't
    // collide between chunks and can be cleaned up between iterations.
    const fs::path chunk_ws =
        chunked ? (workspace / fmt::format("chunk_{:03d}", ci)) : workspace;
    const fs::path images_dir = chunk_ws / "images";
    fs::create_directories(images_dir);
    setWorkingFolder(chunk_ws);

    if (chunked)
      reusex::info("=== Chunk {}/{}: {} frames ===", ci + 1, chunks.size(),
                   chunk_frames.size());

    // ── Build the MVS::Scene ───────────────────────────────────────────
    // Pass the user's thread cap into the Scene ctor; 0 means "let
    // OpenMVS pick" (= all CPU threads, the default). Limiting threads
    // is the single largest RAM lever on depth-map estimation.
    Scene scene(p.max_threads > 0 ? static_cast<unsigned>(p.max_threads)
                                  : 0u);
    initOpenMVSDefaults(p, scene.nMaxThreads);
    if (ci == 0)
      reusex::info("OpenMVS using up to {} threads", scene.nMaxThreads);

    // Single-platform model: for a monocular capture (one physical
    // camera moved through the scene) OpenMVS expects 1 Platform + 1
    // Camera + N Poses + N Images, NOT one platform per frame. We pick
    // the first frame's intrinsics as the canonical camera; sub-pixel
    // per-frame variation is treated as noise. Multi-platform Scenes
    // also appear to trigger heap corruption in Scene::~Scene.
    bool platform_initialized = false;
    std::size_t used = 0, skipped = 0;

    for (int node_id : chunk_frames) {
      if (!db.has_sensor_frame(node_id)) {
        ++skipped;
        continue;
      }

    cv::Mat color = db.sensor_frame_image(node_id);
    if (color.empty()) {
      ++skipped;
      continue;
    }
    auto intr = db.sensor_frame_intrinsics(node_id);
    if (intr.width <= 0 || intr.height <= 0 || intr.fx <= 0 || intr.fy <= 0) {
      reusex::warn("Node {}: invalid intrinsics, skipping", node_id);
      ++skipped;
      continue;
    }
    if (!debug_intrinsics_printed) {
      reusex::info("[densify-debug] raw intrinsics for node {}:", node_id);
      reusex::info(
          "[densify-debug]   fx={:.3f} fy={:.3f} cx={:.3f} cy={:.3f} w={} h={}",
          intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height);
      reusex::info("[densify-debug]   image actual size = {}×{}", color.cols,
                   color.rows);
      debug_intrinsics_printed = true;
    }
    // If stored intrinsics dimensions disagree with the image, scale them
    // to the actual image — same heuristic as reconstruct_point_clouds().
    if (color.cols != intr.width || color.rows != intr.height) {
      const double sx = static_cast<double>(color.cols) / intr.width;
      const double sy = static_cast<double>(color.rows) / intr.height;
      intr.fx *= sx;
      intr.fy *= sy;
      intr.cx *= sx;
      intr.cy *= sy;
      intr.width = color.cols;
      intr.height = color.rows;
    }

    if (!platform_initialized) {
      Platform platform;
      platform.name = "platform_0";
      Platform::Camera cam;
      // OpenMVS stores intrinsics on a platform camera in NORMALIZED form
      // with the pixel-center-at-integer convention (see Util.inl:ScaleK).
      // Image::UpdateCamera() runs ScaleK(K, max(w,h)) to restore pixel
      // coords, which means:
      //   fx_pixel = fx_norm * s
      //   cx_pixel = (cx_norm + 0.5) * s - 0.5
      // We invert this so the round-trip recovers the raw intrinsics.
      const double norm =
          static_cast<double>(std::max(intr.width, intr.height));
      cam.K = KMatrix::IDENTITY;
      cam.K(0, 0) = intr.fx / norm;
      cam.K(1, 1) = intr.fy / norm;
      cam.K(0, 2) = (intr.cx + 0.5) / norm - 0.5;
      cam.K(1, 2) = (intr.cy + 0.5) / norm - 0.5;
      cam.R = RMatrix::IDENTITY;
      cam.C = CMatrix(0, 0, 0);
      platform.cameras.emplace_back(cam);
      scene.platforms.emplace_back(platform);
      platform_initialized = true;
    }
    const uint32_t platform_id = 0;

    // Compose T_wc = T_wb * T_bc, then split into Pose::R (world→camera)
    // and Pose::C (camera center in world). Matches the convention in
    // reconstruct.cpp: world_pt = worldTf * (localTf * cam_pt).
    Eigen::Matrix4d T_wb = to_matrix4d(db.sensor_frame_pose(node_id));
    Eigen::Matrix4d T_bc = to_matrix4d(intr.local_transform);
    Eigen::Matrix4d T_wc = T_wb * T_bc;
    Eigen::Matrix3d R_wc = T_wc.block<3, 3>(0, 0);
    Eigen::Vector3d C_w = T_wc.block<3, 1>(0, 3);
    Eigen::Matrix3d R_cw = R_wc.transpose();

    Platform::Pose mvspose;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        mvspose.R(r, c) = R_cw(r, c);
    mvspose.C = CMatrix(C_w.x(), C_w.y(), C_w.z());

    Platform &platform = scene.platforms[platform_id];
    const uint32_t pose_id = static_cast<uint32_t>(platform.poses.size());
    platform.poses.emplace_back(mvspose);

    // Stage the JPEG. OpenMVS opens images by path through cv::imread;
    // pass an absolute path to skip its (fragile) WORKING_FOLDER lookup.
    const fs::path abs_path = images_dir / fmt::format("{:08d}.jpg", node_id);
    if (!cv::imwrite(abs_path.string(), color))
      throw std::runtime_error("Failed to stage image " + abs_path.string());

    Image image;
    image.platformID = platform_id;
    image.cameraID = 0;
    image.poseID = pose_id;
    image.ID = static_cast<uint32_t>(node_id);
    image.name = SEACAVE::String(abs_path.string());
    image.width = static_cast<uint32_t>(color.cols);
    image.height = static_cast<uint32_t>(color.rows);
    image.scale = 1.f;
    image.UpdateCamera(scene.platforms);
    scene.images.emplace_back(image);
    ++used;
  }

    if (scene.images.empty()) {
      if (chunked) {
        reusex::warn("Chunk {}/{} had no usable frames — skipping",
                     ci + 1, chunks.size());
        std::error_code ec;
        fs::remove_all(chunk_ws, ec);
        continue;
      }
      fs::remove_all(workspace);
      throw std::runtime_error(
          "densify_from_images: no usable sensor frames after filtering");
    }
    scene.nCalibratedImages = static_cast<unsigned>(scene.images.size());
    reusex::info("Built MVS scene: {} images, {} cameras (skipped {})", used,
                 scene.platforms.size(), skipped);

  // ── Optional LiDAR seed point cloud ─────────────────────────────────
  // OpenMVS's SelectNeighborViews uses point→image co-visibility to score
  // candidate neighbor images. With no SfM step we have to compute this
  // ourselves: project each LiDAR point into every camera and record
  // which views actually see it. A point with empty pointViews is
  // discarded; one with bogus pointViews wrecks neighbor selection.
  if (!p.seed_cloud_name.empty() && db.has_point_cloud(p.seed_cloud_name)) {
    try {
      auto seed = db.point_cloud_xyzrgb(p.seed_cloud_name);
      if (seed && !seed->empty()) {
        // Pre-compute world→camera transform per image (R_cw, t_cw, K).
        struct CamInfo {
          Eigen::Matrix3d R_cw;
          Eigen::Vector3d t_cw;
          double fx, fy, cx, cy;
          int w, h;
        };
        std::vector<CamInfo> cams;
        cams.reserve(scene.images.size());
        for (const Image &img : scene.images) {
          CamInfo c;
          // img.camera.R is already world→camera (set up earlier via
          // UpdateCamera). img.camera.C is camera centre in world.
          for (int r = 0; r < 3; ++r)
            for (int cc = 0; cc < 3; ++cc)
              c.R_cw(r, cc) = img.camera.R(r, cc);
          Eigen::Vector3d C(img.camera.C.x, img.camera.C.y, img.camera.C.z);
          c.t_cw = -c.R_cw * C;
          c.fx = img.camera.K(0, 0);
          c.fy = img.camera.K(1, 1);
          c.cx = img.camera.K(0, 2);
          c.cy = img.camera.K(1, 2);
          c.w = static_cast<int>(img.width);
          c.h = static_cast<int>(img.height);
          cams.emplace_back(c);
        }
        if (!debug_camera_printed) {
          const auto &c0 = cams.front();
          reusex::info("[densify-debug] camera 0:");
          reusex::info(
              "[densify-debug]   K = "
              "[[{:.2f},0,{:.2f}],[0,{:.2f},{:.2f}],[0,0,1]]  w×h={}×{}",
              c0.fx, c0.cx, c0.fy, c0.cy, c0.w, c0.h);
          reusex::info("[densify-debug]   C_w (cam centre in world) = [{:.3f}, "
                       "{:.3f}, {:.3f}]",
                       -(c0.R_cw.transpose() * c0.t_cw).x(),
                       -(c0.R_cw.transpose() * c0.t_cw).y(),
                       -(c0.R_cw.transpose() * c0.t_cw).z());
          reusex::info("[densify-debug]   forward axis (R_cw row 2) = [{:.3f}, "
                       "{:.3f}, {:.3f}]",
                       c0.R_cw(2, 0), c0.R_cw(2, 1), c0.R_cw(2, 2));
          const auto &p0 = (*seed)[0];
          reusex::info(
              "[densify-debug] LiDAR pt 0 (world) = [{:.3f}, {:.3f}, {:.3f}]",
              p0.x, p0.y, p0.z);
          Eigen::Vector3d Pw(p0.x, p0.y, p0.z);
          Eigen::Vector3d Pc = c0.R_cw * Pw + c0.t_cw;
          reusex::info(
              "[densify-debug]   in camera 0 frame = [{:.3f}, {:.3f}, {:.3f}]",
              Pc.x(), Pc.y(), Pc.z());
          debug_camera_printed = true;
        }

        const std::size_t total = seed->size();
        const std::size_t cap =
            p.seed_max_points > 0 ? static_cast<std::size_t>(p.seed_max_points)
                                  : total;
        const std::size_t seed_stride =
            total <= cap ? 1 : (total + cap - 1) / cap;

        std::size_t seeded = 0;
        std::size_t orphaned = 0;
        for (std::size_t i = 0; i < total; i += seed_stride) {
          const auto &pt = (*seed)[i];
          if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
              !std::isfinite(pt.z))
            continue;
          Eigen::Vector3d Pw(pt.x, pt.y, pt.z);

          PointCloud::ViewArr views;
          for (PointCloud::View vi = 0;
               vi < static_cast<PointCloud::View>(cams.size()); ++vi) {
            const CamInfo &c = cams[vi];
            Eigen::Vector3d Pc = c.R_cw * Pw + c.t_cw;
            if (Pc.z() <= 0.01) // behind camera or right on it
              continue;
            const double u = c.fx * Pc.x() / Pc.z() + c.cx;
            const double v = c.fy * Pc.y() / Pc.z() + c.cy;
            if (u < 0.0 || u >= c.w || v < 0.0 || v >= c.h)
              continue;
            views.emplace_back(vi);
          }
          if (views.empty()) {
            ++orphaned;
            continue;
          }

          scene.pointcloud.points.emplace_back(
              PointCloud::Point(pt.x, pt.y, pt.z));
          PointCloud::Color col;
          col.r = pt.r;
          col.g = pt.g;
          col.b = pt.b;
          scene.pointcloud.colors.emplace_back(col);
          scene.pointcloud.pointViews.emplace_back(views);
          ++seeded;
        }
        reusex::info("Seeded {} LiDAR points with real visibility "
                     "(dropped {} with no visible camera)",
                     seeded, orphaned);
      }
    } catch (const std::exception &e) {
      reusex::warn("Could not load LiDAR seed cloud '{}': {} (continuing "
                   "without seed)",
                   p.seed_cloud_name, e.what());
    }
  }

  // ── Pick view neighbors and run dense reconstruction ────────────────
  reusex::info("Selecting view neighbors");
  scene.SelectNeighborViews();

  // The seed pointcloud was only there to bootstrap SelectNeighborViews
  // with realistic point→image visibility. Once each image has its
  // `neighbors` list populated, DenseReconstruction expects the
  // pointcloud to start empty so it can be filled from depth-map fusion.
  // Leaving the seed in place crashes the fusion stage after depth maps
  // reach 100%.
  if (!scene.pointcloud.IsEmpty()) {
    reusex::debug("Releasing {} seed points before dense reconstruction",
                  scene.pointcloud.points.size());
    scene.pointcloud.Release();
  }

    if (ci == 0)
      reusex::info("Running OpenMVS dense reconstruction "
                   "(resolution_level={}, max_resolution={}, "
                   "geom_consistency={})",
                   p.resolution_level, p.max_resolution,
                   p.geometric_consistency);
    {
      // OpenMVS prints its own "\r"-based progress to std::cout — let it
      // through. We previously routed it through a spdmon LoggerProgress
      // wrapper which allocated unbounded fmt buffers (157 GB peak in
      // heaptrack); the OpenMVS line is functionally equivalent and
      // costs essentially zero.
      if (!scene.DenseReconstruction(/*nFusionMode=*/0,
                                     /*bCrop2ROI=*/false)) {
        if (chunked) {
          reusex::warn("Chunk {}/{} DenseReconstruction failed — skipping",
                       ci + 1, chunks.size());
          // Clean up this chunk's workspace before moving on.
          std::error_code ec;
          fs::remove_all(chunk_ws, ec);
          continue;
        }
        fs::remove_all(workspace);
        throw std::runtime_error(
            "OpenMVS DenseReconstruction failed (see prior log)");
      }
    }
    reusex::debug("Chunk {}/{} produced {} points", ci + 1, chunks.size(),
                  scene.pointcloud.points.size());
    append_mvs_points(scene.pointcloud, out);
    // scene goes out of scope at the end of the iteration; its destructor
    // releases all OpenMVS-side state. Calling Release() explicitly here
    // used to cause a double-free + glibc free(): invalid size SIGABRT.
    if (chunked) {
      std::error_code ec;
      fs::remove_all(chunk_ws, ec);
    }
  } // ── end of per-chunk loop ─────────────────────────────────────────

  reusex::info("OpenMVS produced {} dense points across {} chunk(s)",
               out.size(), chunks.size());

  // ── Bbox clip vs LiDAR seed cloud ───────────────────────────────────
  // OpenMVS leaves far-outlier patches dozens of metres away from the
  // real scene — points whose depth estimates triangulated poorly and
  // landed in empty space. SOR can't catch them because they cluster
  // among themselves, so each one sees many "neighbours" at low mean
  // distance. The LiDAR seed cloud bounds where the real scene is, so
  // clipping to its (margin-expanded) AABB drops the patches without
  // touching anything plausibly inside the scene.
  if (p.bbox_clip && !p.seed_cloud_name.empty() &&
      db.has_point_cloud(p.seed_cloud_name) && !out.empty()) {
    try {
      auto seed = db.point_cloud_xyzrgb(p.seed_cloud_name);
      if (seed && seed->size() > 100) {
        // Use percentile bounds (p1..p99) for robustness against any
        // stray seed-cloud outliers.
        std::vector<float> xs, ys, zs;
        xs.reserve(seed->size());
        ys.reserve(seed->size());
        zs.reserve(seed->size());
        for (const auto &pt : *seed) {
          if (std::isfinite(pt.x) && std::isfinite(pt.y) &&
              std::isfinite(pt.z)) {
            xs.push_back(pt.x);
            ys.push_back(pt.y);
            zs.push_back(pt.z);
          }
        }
        std::sort(xs.begin(), xs.end());
        std::sort(ys.begin(), ys.end());
        std::sort(zs.begin(), zs.end());
        const auto pct = [](const std::vector<float> &v, double p) {
          return v[static_cast<std::size_t>(p * (v.size() - 1))];
        };
        Eigen::Vector4f lo(pct(xs, 0.01f), pct(ys, 0.01f), pct(zs, 0.01f), 1.f);
        Eigen::Vector4f hi(pct(xs, 0.99f), pct(ys, 0.99f), pct(zs, 0.99f), 1.f);
        const Eigen::Vector4f span = hi - lo;
        const float m = static_cast<float>(std::max(0.0, p.bbox_margin));
        lo -= span * m;
        hi += span * m;
        lo[3] = 1.f;
        hi[3] = 1.f;

        pcl::CropBox<PointT> crop;
        auto in_cloud = std::make_shared<Cloud>(std::move(out));
        crop.setInputCloud(in_cloud);
        crop.setMin(lo);
        crop.setMax(hi);
        Cloud clipped;
        crop.filter(clipped);
        reusex::info("Bbox clip (LiDAR p1..p99 + {:.0f}% margin) kept "
                     "{} of {} points ({:.1f}%)",
                     100.0 * p.bbox_margin, clipped.size(), in_cloud->size(),
                     100.0 * clipped.size() /
                         std::max<std::size_t>(1, in_cloud->size()));
        out = std::move(clipped);
      }
    } catch (const std::exception &e) {
      reusex::warn("Bbox clip skipped — could not load seed cloud '{}': {}",
                   p.seed_cloud_name, e.what());
    }
  }

  // ── Statistical outlier removal ─────────────────────────────────────
  // Local SOR pass for any in-volume noise the bbox clip can't catch.
  if (p.outlier_filter && !out.empty()) {
    pcl::StatisticalOutlierRemoval<PointT> sor;
    auto in_cloud = std::make_shared<Cloud>(std::move(out));
    sor.setInputCloud(in_cloud);
    sor.setMeanK(std::max(1, p.outlier_mean_k));
    sor.setStddevMulThresh(p.outlier_stddev_thresh);
    Cloud filtered;
    sor.filter(filtered);
    reusex::info("SOR filter kept {} of {} points ({:.1f}%)",
                 filtered.size(), in_cloud->size(),
                 100.0 * filtered.size() /
                     std::max<std::size_t>(1, in_cloud->size()));
    out = std::move(filtered);
  }

  const std::string params_json = fmt::format(
      R"({{"resolution_level":{},"max_resolution":{},"geom_consistency":{},"chunk_size":{},"outlier_filter":{}}})",
      p.resolution_level, p.max_resolution, p.geometric_consistency,
      p.chunk_size, p.outlier_filter);
  db.save_point_cloud(p.output_name, out, "densify", params_json);

  std::error_code ec;
  fs::remove_all(workspace, ec);
  if (ec)
    reusex::warn("Failed to clean up {}: {}", workspace.string(), ec.message());
}

} // namespace reusex::geometry
