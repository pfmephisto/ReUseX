// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/arkitscenes.hpp"
#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace reusex::io {

namespace {

/// Rodrigues rotation vector -> 3x3 rotation matrix. angle = ||v||; axis =
/// v/angle; identity when angle ~ 0.
Eigen::Matrix3d rodrigues_to_matrix(double rx, double ry, double rz) {
  const Eigen::Vector3d v(rx, ry, rz);
  const double angle = v.norm();
  if (angle < 1e-12)
    return Eigen::Matrix3d::Identity();
  const Eigen::Vector3d axis = v / angle;
  return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

/// Locate the frames directory: either @p dir itself holds `lowres_wide.traj`
/// or a single-level subdirectory does (some downloads nest under
/// `<video_id>/` or `<video_id>_frames/`). If several subdirectories qualify
/// the lexicographically smallest is chosen, so the result never depends on
/// directory-iteration order.
std::filesystem::path resolve_frames_dir(const std::filesystem::path &dir) {
  if (std::filesystem::exists(dir / "lowres_wide.traj"))
    return dir;
  if (std::filesystem::is_directory(dir)) {
    std::vector<std::filesystem::path> candidates;
    for (const auto &entry : std::filesystem::directory_iterator(dir)) {
      if (entry.is_directory() &&
          std::filesystem::exists(entry.path() / "lowres_wide.traj"))
        candidates.push_back(entry.path());
    }
    if (!candidates.empty()) {
      std::sort(candidates.begin(), candidates.end());
      if (candidates.size() > 1)
        reusex::warn("import_arkitscenes: {} subdirectories of {} contain "
                     "lowres_wide.traj; using {}",
                     candidates.size(), dir.string(),
                     candidates.front().filename().string());
      return candidates.front();
    }
  }
  throw std::runtime_error(fmt::format(
      "import_arkitscenes: no lowres_wide.traj under {} (expected the scene "
      "frames dir or a single-level subdirectory containing it)",
      dir.string()));
}

/// Nearest-timestamp lookup over a sorted keys vector. Returns the index of the
/// closest key and its absolute difference to @p ts.
std::pair<std::size_t, double> nearest(const std::vector<double> &sorted_keys,
                                       double ts) {
  const auto it = std::lower_bound(sorted_keys.begin(), sorted_keys.end(), ts);
  std::size_t best = 0;
  double best_dt = std::numeric_limits<double>::max();
  auto consider = [&](std::size_t i) {
    const double dt = std::abs(sorted_keys[i] - ts);
    if (dt < best_dt) {
      best_dt = dt;
      best = i;
    }
  };
  if (it != sorted_keys.end())
    consider(static_cast<std::size_t>(it - sorted_keys.begin()));
  if (it != sorted_keys.begin())
    consider(static_cast<std::size_t>(it - sorted_keys.begin() - 1));
  return {best, best_dt};
}

/// Tolerance for the `.pincam` nearest-timestamp fallback. Intrinsics come one
/// per RGB frame (same ~60 Hz cadence), so the exact-name lookup almost always
/// hits; this only covers the odd missing file, and intrinsics drift far more
/// slowly than pose.
constexpr double kIntrinsicsTsTolerance = 0.02; // 20 ms

} // namespace

bool parse_frame_timestamp(const std::filesystem::path &file, double &ts) {
  std::string stem = file.stem().string(); // drops ".png"
  const auto us = stem.find_last_of('_');
  const std::string tok =
      (us == std::string::npos) ? stem : stem.substr(us + 1);
  try {
    std::size_t consumed = 0;
    ts = std::stod(tok, &consumed);
    return consumed == tok.size();
  } catch (const std::exception &) {
    return false;
  }
}

ArkitTrajectory load_arkit_trajectory(const std::filesystem::path &traj_path) {
  ArkitTrajectory traj;
  {
    std::ifstream in(traj_path);
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty())
        continue;
      std::istringstream ss(line);
      double ts, rx, ry, rz, tx, ty, tz;
      if (!(ss >> ts >> rx >> ry >> rz >> tx >> ty >> tz))
        continue;
      traj.push_back({ts, arkit_traj_to_optical_world(rx, ry, rz, tx, ty, tz)});
    }
  }
  if (traj.empty())
    throw std::runtime_error(fmt::format(
        "load_arkit_trajectory: no poses parsed from {}", traj_path.string()));

  std::sort(traj.begin(), traj.end(),
            [](const ArkitPoseSample &a, const ArkitPoseSample &b) {
              return a.ts < b.ts;
            });
  return traj;
}

std::optional<std::array<double, 16>>
interpolate_pose(const ArkitTrajectory &traj, double ts, double max_gap) {
  if (traj.empty())
    return std::nullopt;
  // No extrapolation: a query outside the captured span has no bracketing pair.
  if (ts < traj.front().ts || ts > traj.back().ts)
    return std::nullopt;

  const auto hi = std::lower_bound(
      traj.begin(), traj.end(), ts,
      [](const ArkitPoseSample &s, double t) { return s.ts < t; });
  // ts <= back().ts guarantees hi != end().
  if (hi->ts == ts)
    return hi->pose;
  // ts > front().ts and hi->ts > ts guarantee hi != begin().
  const auto lo = std::prev(hi);

  const double gap = hi->ts - lo->ts;
  if (gap > max_gap)
    return std::nullopt;
  if (gap <= 0.0) // duplicate timestamps: nothing to interpolate across
    return lo->pose;
  const double u = (ts - lo->ts) / gap;

  auto to_affine = [](const std::array<double, 16> &m) {
    Eigen::Matrix4d M;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        M(r, c) = m[static_cast<std::size_t>(r * 4 + c)];
    return M;
  };
  const Eigen::Matrix4d A = to_affine(lo->pose);
  const Eigen::Matrix4d B = to_affine(hi->pose);

  // SLERP the rotation, lerp the camera position — both are components of the
  // camera->world pose, so the interpolant is again a valid camera->world pose.
  const Eigen::Quaterniond qa(Eigen::Matrix3d(A.block<3, 3>(0, 0)));
  const Eigen::Quaterniond qb(Eigen::Matrix3d(B.block<3, 3>(0, 0)));
  const Eigen::Quaterniond q =
      qa.normalized().slerp(u, qb.normalized()); // shortest arc
  const Eigen::Vector3d p =
      (1.0 - u) * A.block<3, 1>(0, 3) + u * B.block<3, 1>(0, 3);

  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.block<3, 3>(0, 0) = q.toRotationMatrix();
  out.block<3, 1>(0, 3) = p;

  std::array<double, 16> pose{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      pose[static_cast<std::size_t>(r * 4 + c)] = out(r, c);
  return pose;
}

ArkitPincam parse_pincam(const std::string &line) {
  std::istringstream ss(line);
  ArkitPincam p;
  if (!(ss >> p.width >> p.height >> p.fx >> p.fy >> p.cx >> p.cy))
    throw std::runtime_error(fmt::format(
        "parse_pincam: expected 'width height fx fy cx cy', got: '{}'", line));
  return p;
}

std::array<double, 16> arkit_traj_to_optical_world(double rx, double ry,
                                                   double rz, double tx,
                                                   double ty, double tz) {
  // A .traj line encodes the world->camera extrinsic E = [R(rodrigues)|t]
  // (matching Apple's TrajStringToMatrix, which builds E then inverts it), so
  // camera->world = E^-1.
  Eigen::Matrix4d E = Eigen::Matrix4d::Identity();
  E.block<3, 3>(0, 0) = rodrigues_to_matrix(rx, ry, rz);
  E.block<3, 1>(0, 3) = Eigen::Vector3d(tx, ty, tz);

  const Eigen::Matrix4d c2w = E.inverse(); // camera->world

  // ARKitScenes' camera frame is already the OpenCV optical convention
  // (x-right, y-down, z-forward) — the same frame reconstruct.cpp back-projects
  // into — so c2w IS the optical-to-world pose and NO axis flip is applied.
  // (Nerfstudio's ARKitScenes parser flips Y/Z only because its internal
  // convention is OpenGL/y-up; empirically an inserted flip here fans the walls
  // into a spiral, while the un-flipped pose yields crisp, GT-aligned walls.)
  const Eigen::Matrix4d pose_mat = c2w; // optical-to-world

  std::array<double, 16> pose{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      pose[static_cast<std::size_t>(r * 4 + c)] = pose_mat(r, c);
  return pose;
}

std::size_t import_arkitscenes(ProjectDB &db,
                               const std::filesystem::path &scene_dir) {
  const auto frames_dir = resolve_frames_dir(scene_dir);

  const auto rgb_dir = frames_dir / "lowres_wide";
  const auto depth_dir = frames_dir / "lowres_depth";
  const auto conf_dir = frames_dir / "confidence";
  const auto intr_dir = frames_dir / "lowres_wide_intrinsics";
  const auto traj_path = frames_dir / "lowres_wide.traj";

  if (!std::filesystem::exists(depth_dir))
    throw std::runtime_error(
        fmt::format("import_arkitscenes: missing lowres_depth/ under {}",
                    frames_dir.string()));
  if (!std::filesystem::exists(traj_path))
    throw std::runtime_error(
        fmt::format("import_arkitscenes: missing lowres_wide.traj under {}",
                    frames_dir.string()));

  // ── Load trajectory: sorted camera->world poses by timestamp ──────
  const ArkitTrajectory traj = load_arkit_trajectory(traj_path);

  // ── Enumerate .pincam intrinsics for nearest-match fallback ───────
  std::vector<double> pincam_ts;
  std::vector<std::filesystem::path> pincam_path;
  if (std::filesystem::is_directory(intr_dir)) {
    std::vector<std::pair<double, std::filesystem::path>> entries;
    for (const auto &e : std::filesystem::directory_iterator(intr_dir)) {
      if (!e.is_regular_file() || e.path().extension() != ".pincam")
        continue;
      // .pincam files share the `<vid>_<ts>` stem of the depth/RGB frame, so
      // parse the timestamp after the last underscore (a bare std::stod would
      // stop at the video-id prefix and collapse every key to the same value).
      double ts = 0;
      if (!parse_frame_timestamp(e.path(), ts))
        continue;
      entries.emplace_back(ts, e.path());
    }
    std::sort(entries.begin(), entries.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    for (auto &[ts, p] : entries) {
      pincam_ts.push_back(ts);
      pincam_path.push_back(std::move(p));
    }
  }

  // ── Enumerate depth frames sorted by timestamp ────────────────────
  std::vector<std::pair<double, std::filesystem::path>> depth_frames;
  for (const auto &e : std::filesystem::directory_iterator(depth_dir)) {
    if (!e.is_regular_file() || e.path().extension() != ".png")
      continue;
    double ts = 0;
    if (!parse_frame_timestamp(e.path(), ts)) {
      reusex::debug("import_arkitscenes: skipping depth file with "
                    "unparseable timestamp {}",
                    e.path().filename().string());
      continue;
    }
    depth_frames.emplace_back(ts, e.path());
  }
  std::sort(depth_frames.begin(), depth_frames.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  if (depth_frames.empty())
    throw std::runtime_error(fmt::format(
        "import_arkitscenes: no depth PNGs found in {}", depth_dir.string()));

  std::size_t imported = 0;
  std::size_t no_pose = 0;
  int node_id = 1;
  for (const auto &[ts, depth_path] : depth_frames) {
    // ── Pose interpolated to this frame's own timestamp ───────────
    // The trajectory is ~10 Hz against ~60 Hz depth, so nearest-pose matching
    // would give five frames in six a pose up to ~17 ms stale. SLERP/lerp
    // between the bracketing samples removes that temporal error; frames
    // outside the trajectory span (or across a tracking-loss gap) are skipped.
    const auto interpolated = interpolate_pose(traj, ts);
    if (!interpolated) {
      reusex::debug("import_arkitscenes: frame ts {:.3f} has no bracketing "
                    "trajectory pair (span {:.3f}..{:.3f}, max gap {:.2f} s); "
                    "skipping",
                    ts, traj.front().ts, traj.back().ts, kArkitMaxPoseGap);
      ++no_pose;
      continue;
    }
    const std::array<double, 16> &pose = *interpolated;

    // ── RGB ───────────────────────────────────────────────────────
    const std::string frame_name = depth_path.stem().string(); // <vid>_<ts>
    const auto rgb_path = rgb_dir / (frame_name + ".png");
    const cv::Mat color = cv::imread(rgb_path.string(), cv::IMREAD_COLOR);
    if (color.empty()) {
      reusex::warn("import_arkitscenes: skipping frame {} — unreadable RGB {}",
                   frame_name, rgb_path.string());
      continue;
    }

    // ── Depth (already CV_16UC1 millimeters) ──────────────────────
    cv::Mat depth = cv::imread(depth_path.string(), cv::IMREAD_UNCHANGED);
    if (depth.empty()) {
      reusex::warn(
          "import_arkitscenes: skipping frame {} — unreadable depth {}",
          frame_name, depth_path.string());
      continue;
    }
    if (depth.type() != CV_16UC1) {
      reusex::warn("import_arkitscenes: skipping frame {} — depth {} is not "
                   "CV_16UC1 (type {})",
                   frame_name, depth_path.string(), depth.type());
      continue;
    }

    // ── Confidence (CV_8UC1 {0,1,2}) ──────────────────────────────
    const auto conf_path = conf_dir / (frame_name + ".png");
    cv::Mat confidence = cv::imread(conf_path.string(), cv::IMREAD_UNCHANGED);
    if (confidence.empty() || confidence.rows != depth.rows ||
        confidence.cols != depth.cols || confidence.type() != CV_8UC1) {
      // No usable per-pixel confidence: fall back to fully-confident so the
      // frame is still reconstructed, but warn — this disables the
      // confidence-weighted depth filtering in reconstruct.
      reusex::warn("import_arkitscenes: frame {} — missing/invalid confidence "
                   "{}, treating all pixels as fully confident",
                   frame_name, conf_path.string());
      confidence = cv::Mat(depth.rows, depth.cols, CV_8UC1, cv::Scalar(2));
    }

    // ── Intrinsics: exact <ts>.pincam, else nearest ───────────────
    ArkitPincam pin;
    bool have_pin = false;
    {
      // The .pincam file shares the depth/RGB frame's `<vid>_<ts>` stem, so the
      // exact filename is just the frame name with a .pincam extension.
      const auto exact = intr_dir / (frame_name + ".pincam");
      std::ifstream in;
      if (std::filesystem::exists(exact)) {
        in.open(exact);
      } else if (!pincam_ts.empty()) {
        const auto [pi, pdt] = nearest(pincam_ts, ts);
        if (pdt <= kIntrinsicsTsTolerance)
          in.open(pincam_path[pi]);
      }
      if (in.is_open()) {
        std::string line;
        if (std::getline(in, line)) {
          try {
            pin = parse_pincam(line);
            have_pin = true;
          } catch (const std::exception &e) {
            reusex::warn("import_arkitscenes: frame {} — {}", frame_name,
                         e.what());
          }
        }
      }
    }
    if (!have_pin) {
      reusex::warn("import_arkitscenes: skipping frame {} — no intrinsics "
                   "(.pincam) within tolerance",
                   frame_name);
      continue;
    }

    core::SensorIntrinsics intr;
    intr.fx = pin.fx;
    intr.fy = pin.fy;
    intr.cx = pin.cx;
    intr.cy = pin.cy;
    // width/height must be the resolution the .pincam's fx/fy/cx/cy are defined
    // at (reconstruct rescales intrinsics from intr.width/height to the depth
    // size). Use the pincam's own dims, not the RGB image size — they coincide
    // on canonical lowres data but diverge if a differently-sized intrinsics
    // file is matched.
    intr.width = static_cast<int>(pin.width);
    intr.height = static_cast<int>(pin.height);
    // identity local_transform (pose already optical-to-world)

    db.save_sensor_frame(node_id, color, depth, confidence, pose, intr, ts);
    ++imported;
    ++node_id;
  }

  if (imported == 0)
    throw std::runtime_error(
        fmt::format("import_arkitscenes: no frames could be imported from {}",
                    frames_dir.string()));

  reusex::info("import_arkitscenes: imported {} of {} depth frames from {} "
               "(interpolated poses; {} outside the trajectory span)",
               imported, depth_frames.size(), frames_dir.string(), no_pose);
  return imported;
}

} // namespace reusex::io
